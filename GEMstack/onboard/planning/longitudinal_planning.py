from typing import List
from ..component import Component
from ...state import (
    AllState,
    VehicleState,
    EntityRelation,
    EntityRelationEnum,
    Path,
    Trajectory,
    Route,
    ObjectFrameEnum,
)
from ...utils import serialization, settings
from ...mathutils.transforms import vector_madd, vector_sub, normalize_vector
import math
import numpy as np

DELTA_T = 0.05

def longitudinal_plan(path : Path, acceleration : float, deceleration : float, max_speed : float, current_speed : float) -> Trajectory:
    """Generates a longitudinal trajectory for a path with a
    trapezoidal velocity profile. 
    
    1. accelerates from current speed toward max speed
    2. travel along max speed
    3. if at any point you can't brake before hitting the end of the path,
       decelerate with accel = -deceleration until velocity goes to 0.
    """
    path_normalized = path.arc_length_parameterize()
    # TODO: actually do something to points and times

    points = [p for p in path_normalized.points]
    path_length = [t for t in path_normalized.times][-1]
    direction = normalize_vector(vector_sub(points[-1], points[0]))

    input_speed = current_speed
    current_pos = 0
    current_time = 0
    positions = []
    times = []
    velocities = []
    if current_speed < max_speed and acceleration != 0:
        time_to_max_speed = (max_speed - current_speed) / acceleration
        times_to_max = np.arange(0, time_to_max_speed + DELTA_T, DELTA_T)
        positions_until_max_speed = current_speed * times_to_max + (1/2)*acceleration*(times_to_max **2)
        velocities_until_max = current_speed + acceleration*(times_to_max)
        positions.append(positions_until_max_speed)
        times.append(times_to_max)
        velocities.append(velocities_until_max)
        current_pos = positions_until_max_speed[-1]
        current_time = times_to_max[-1]

    current_speed = max_speed
    if current_pos < path_length:
        s = path_length - current_pos
        time_to_path_end = s / current_speed
        times_to_end = np.arange(0, time_to_path_end + DELTA_T, DELTA_T)
        positions_until_end = (current_speed * times_to_end) + current_pos
        times_to_end += current_time
        velocities_to_end = np.ones(len(positions_until_end)) * current_speed
        positions.append(positions_until_end)
        times.append(times_to_end)
        velocities.append(velocities_to_end)

    positions = np.concatenate(positions)
    available_distance = path_length - positions

    velocities = np.concatenate(velocities)
    braking_distance = (velocities**2)/(2*deceleration)

    times = np.concatenate(times)

    can_brake = available_distance > braking_distance
    velocities = velocities[can_brake]
    positions = positions[can_brake]
    times = times[can_brake]

    if len(velocities) != 0:
        current_speed = velocities[-1]
        current_time = times[-1]
        current_pos = positions[-1]
    else:
        current_speed = input_speed
        current_time = 0
        current_pos = 0

    time_to_stop = current_speed / deceleration
    times_to_stop = np.arange(0, time_to_stop + DELTA_T, DELTA_T)
    positions_to_stop = (current_speed * times_to_stop - (1/2)*deceleration*(times_to_stop**2)) + current_pos
    velocities_to_stop = current_speed + (-deceleration * times_to_stop)
    times_to_stop += current_time

    positions = np.concatenate([positions, positions_to_stop])
    times = np.concatenate([times, times_to_stop])

    positions = positions.tolist()
    positions = [vector_madd(points[0], direction, position) for position in positions]

    return Trajectory(path.frame, positions, times.tolist())

def longitudinal_brake(
    path: Path, deceleration: float, current_speed: float
) -> Trajectory:
    """Generates a longitudinal trajectory for braking along a path."""
    path_normalized = path.arc_length_parameterize()
    # TODO: actually do something to points and times

    points = [p for p in path_normalized.points]
    times = []
    # Compute the time needed to decelerate to stop
    decel_time = max(current_speed / deceleration, DELTA_T)
    times = np.arange(0.0, decel_time + DELTA_T, DELTA_T)
    s = current_speed * times - 0.5 * deceleration * times**2
    s[s < 0] = 0
    zeros = np.zeros(len(s))
    points = list(zip(s, zeros))

    return Trajectory(path.frame, points, times.tolist())


class YieldTrajectoryPlanner(Component):
    """Follows the given route.  Brakes if you have to yield or
    you are at the end of the route, otherwise accelerates to
    the desired speed.
    """

    def __init__(self):
        self.route_progress = None
        self.t_last = None
        self.acceleration = 0.5 #settings.get("control.longitudinal_planning_yielding.acceleration")
        self.desired_speed = 1.0 #settings.get("control.longitudinal_planning_yielding.desired_speed")
        self.deceleration = 2.0 #settings.get("control.longitudinal_planning_yielding.deceleration")

    def state_inputs(self):
        return ["all"]

    def state_outputs(self) -> List[str]:
        return ["trajectory"]

    def rate(self):
        return 10.0

    def update(self, state: AllState):
        vehicle = state.vehicle  # type: VehicleState
        route = state.route  # type: Route
        t = state.t

        if self.t_last is None:
            self.t_last = t
        dt = t - self.t_last

        curr_x = vehicle.pose.x
        curr_y = vehicle.pose.y
        curr_v = vehicle.v

        # figure out where we are on the route
        if self.route_progress is None:
            self.route_progress = 0.0
        closest_dist, closest_parameter = state.route.closest_point_local(
            (curr_x, curr_y), [self.route_progress - 5.0, self.route_progress + 5.0]
        )
        self.route_progress = closest_parameter

        # extract out a 10m segment of the route
        route_with_lookahead = route.trim(closest_parameter, closest_parameter + 10.0)

        # parse the relations indicated
        should_brake = False
        for r in state.relations:
            if r.type == EntityRelationEnum.YIELDING and r.obj1 == "":
                # yielding to something, brake
                should_brake = True
        should_accelerate = not should_brake and curr_v < self.desired_speed

        # choose whether to accelerate, brake, or keep at current velocity
        if should_accelerate:
            traj = longitudinal_plan(
                route_with_lookahead,
                self.acceleration,
                self.deceleration,
                self.desired_speed,
                curr_v,
            )
        elif should_brake:
            traj = longitudinal_brake(route_with_lookahead, self.deceleration, curr_v)
        else:
            traj = longitudinal_plan(
                route_with_lookahead, 0.0, self.deceleration, self.desired_speed, curr_v
            )

        return traj