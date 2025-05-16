# File: GEMstack/onboard/planning/creep_planning.py
from typing import List, Tuple
import math
from ..component import Component
from ...state import AllState, VehicleState, EntityRelation, EntityRelationEnum, Path, Trajectory, Route, \
    ObjectFrameEnum
from ...utils import serialization
from ...mathutils import transforms
import numpy as np
from .parking_motion_planning import longitudinal_plan as lp_fn, longitudinal_brake as lb_fn

DEBUG = False  # Set to False to disable debug output


class CreepTrajectoryPlanner(Component):
    """Follows the given route. Brakes if the ego–vehicle must yield
    (e.g. to a pedestrian) or if the end of the route is near; otherwise,
    it accelerates (or cruises) toward a desired speed.
    """

    def __init__(self):
        self.route_index = None
        self.time_prev = None
        self.accel_gain = 5
        self.target_vel = 2.0
        self.decel_gain = 2.0
        self.full_brake = 8.0
        self.stop_dist_threshold = 10  # Distance in meters to start linear deceleration

    def state_inputs(self):
        return ['all']

    def state_outputs(self) -> List[str]:
        return ['trajectory']

    def rate(self):
        return 10.0
    
    def is_near_route_end(self, given_route, pos_param):
        route_len = given_route.length()
        remaining_dist = route_len - pos_param

        if DEBUG:
            print(f"[DEBUG] is_near_route_end: Route length = {route_len}, "
                  f"Current position = {pos_param}, "
                  f"Distance remaining = {remaining_dist}")

        if remaining_dist <= self.stop_dist_threshold:
            if remaining_dist > 0.1:
                req_decel = (self.target_vel ** 2) / (2 * remaining_dist)
                linear_scale = remaining_dist / self.stop_dist_threshold
                speed_mod = self.target_vel * linear_scale
                if DEBUG:
                    print(f"[DEBUG] Linear decel active: {remaining_dist:.2f}m left, "
                          f"required deceleration = {req_decel:.2f} m/s², "
                          f"adjusted speed = {speed_mod:.2f} m/s")
                return True, speed_mod
            else:
                return True, 0.0

        return False, self.target_vel

    def update(self, full_state: AllState):
        ego = full_state.vehicle  # type: VehicleState
        nav_path = full_state.route  # type: Route
        current_time = full_state.t

        if self.time_prev is None:
            self.time_prev = current_time
        delta_time = current_time - self.time_prev

        ego_x = ego.pose.x
        ego_y = ego.pose.y
        ego_v = ego.v

        if self.route_index is None:
            self.route_index = 0.0

        _, new_param = nav_path.closest_point_local(
            [ego_x, ego_y],
            (self.route_index - 5.0, self.route_index + 5.0)
        )
        self.route_index = new_param
        near_end, speed_setpoint = self.is_near_route_end(nav_path, new_param)
        trimmed_path = nav_path.trim(new_param, new_param + 10.0)

        duplicate_detected = False
        unique_pts = set()
        for j in range(len(trimmed_path.points)):
            if tuple(trimmed_path.points[j]) in unique_pts:
                duplicate_detected = True
                break
            unique_pts.add(tuple(trimmed_path.points[j]))

        brake_trigger = any(
            rel.type == EntityRelationEnum.STOPPING_AT and rel.obj1 == ''
            for rel in full_state.relations
        )
        yield_trigger = any(
            rel.type == EntityRelationEnum.YIELDING and rel.obj1 == ''
            for rel in full_state.relations
        ) if not brake_trigger else False

        if near_end:
            accelerate = (not brake_trigger and not yield_trigger and ego_v < speed_setpoint)
        else:
            accelerate = (not brake_trigger and not yield_trigger and ego_v < self.target_vel)

        if duplicate_detected:
            trajectory = lb_fn(trimmed_path, 0.0, 0.0, 0.0)
        elif brake_trigger:
            trajectory = lb_fn(trimmed_path, self.full_brake, ego_v)
        elif yield_trigger:
            trajectory = lb_fn(trimmed_path, self.decel_gain, ego_v)
        elif near_end:
            trajectory = lp_fn(trimmed_path, self.accel_gain,
                               self.decel_gain, speed_setpoint, ego_v)
        elif accelerate:
            trajectory = lp_fn(trimmed_path, self.accel_gain,
                               self.decel_gain, self.target_vel, ego_v)
        else:
            trajectory = lp_fn(trimmed_path, 0.0, self.decel_gain, self.target_vel, ego_v)

        self.time_prev = current_time
        return trajectory

