# File: GEMstack/onboard/planning/creep_planning.py
from typing import List, Tuple
import math
from ..component import Component
from ...state import AllState, VehicleState, EntityRelation, EntityRelationEnum, Path, Trajectory, Route, \
    ObjectFrameEnum
from ...utils import serialization
from ...mathutils import transforms
import numpy as np
from .parking_motion_planning import longitudinal_plan, longitudinal_brake


DEBUG = False  # Set to False to disable debug output


'''
# OTHER CREEP CONTROL CODE

def inverse_speed_function(distance_to_object):
    
#     max_creep_speed = 1.0  # Maximum speed when creeping (m/s)
#     min_creep_speed = 0.2  # Minimum speed to maintain (m/s)
#     safe_distance = 5.0    # Distance at which to start slowing down (m)
    
#     # Almost stop when very close (≤0.25m)
#     if distance_to_object <= 0.25:
#         return 0.0  # Practically zero speed
    
#     # Slow creep when close (≤1.5m)
#     if distance_to_object <= 3.0:
#         return min_creep_speed
#     speed = min_creep_speed + (max_creep_speed - min_creep_speed) * (distance_to_object / safe_distance)
#     return min(speed, max_creep_speed)

def pid_speed_control(distance_to_object, target_distance, current_speed, prev_error, integral, dt, 
                     kp=0.5, ki=0.1, kd=0.05, min_speed=0.2, max_speed=1.0):
    if distance_to_object <= 0.25:
        return 0.0, 0.0, 0.0
    error = distance_to_object - target_distance    
    integral += error * dt
    integral = max(-5.0, min(integral, 5.0))  # Prevent integral windup
    derivative = (error - prev_error) / dt if dt > 0 else 0
    p_term = kp * error
    i_term = ki * integral
    d_term = kd * derivative
    speed_adjustment = p_term + i_term + d_term
    base_speed = 0.5  # Base creep speed
    target_speed = base_speed + speed_adjustment
    target_speed = max(min_speed, min(target_speed, max_speed))
    return target_speed, error, integral


'''



class CreepTrajectoryPlanner(Component):
    """Follows the given route. Brakes if the ego–vehicle must yield
    (e.g. to a pedestrian) or if the end of the route is near; otherwise,
    it accelerates (or cruises) toward a desired speed.
    """

    def __init__(self):
        self.route_progress = None
        self.t_last = None
        self.acceleration = 5
        self.desired_speed = 2.0
        self.deceleration = 2.0
        self.emergency_brake = 8.0
        # Parameters for end-of-route linear deceleration
        self.end_stop_distance = 12.5  # Distance in meters to start linear deceleration
        # Did 15, 10, 7.5, 17.5, 20, 12.5
    def state_inputs(self):
        return ['all']

    def state_outputs(self) -> List[str]:
        return ['trajectory']

    def rate(self):
        return 10.0
    
    def check_end_of_route(self, route, current_parameter):
        """Check if vehicle is approaching the end of the route and calculate
        appropriate linear deceleration parameters.
        
        Args:
            route: The complete route
            current_parameter: Current position along the route
            
        Returns:
            (bool, float): Tuple containing:
                - Whether the vehicle should start decelerating
                - Adjusted speed if deceleration needed, otherwise desired_speed
        """
        route_length = route.length()
        
        distance_remaining = route_length - current_parameter
        
        if DEBUG:
            print(f"[DEBUG] check_end_of_route: Route length = {route_length}, " 
                  f"Current position = {current_parameter}, "
                  f"Distance remaining = {distance_remaining}")
        
        if distance_remaining <= self.end_stop_distance:
            if distance_remaining > 0.1:  # Avoid division by very small numbers
                required_decel = (self.desired_speed ** 2) / (2 * distance_remaining)
                
                linear_factor = distance_remaining / self.end_stop_distance
                adjusted_speed = self.desired_speed * linear_factor
                if DEBUG:
                    print(f"[DEBUG] End deceleration active: {distance_remaining:.2f}m remaining, " 
                          f"required deceleration = {required_decel:.2f} m/s², "
                          f"speed adjusted to {adjusted_speed:.2f} m/s")
                
                return True, adjusted_speed
            else:
                return True, 0.0
        
        return False, self.desired_speed

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
        if self.route_progress is None:
            self.route_progress = 0.0
        _, closest_parameter = route.closest_point_local(
            [curr_x, curr_y],
            (self.route_progress - 5.0, self.route_progress + 5.0)
        )
        self.route_progress = closest_parameter
        approaching_end, target_speed = self.check_end_of_route(route, closest_parameter)
        route_with_lookahead = route.trim(closest_parameter, closest_parameter + 10.0)

        stay_braking = False
        pointSet = set()
        for i in range(len(route_with_lookahead.points)):
            if tuple(route_with_lookahead.points[i]) in pointSet:
                stay_braking = True
                break
            pointSet.add(tuple(route_with_lookahead.points[i]))

        should_brake = any(
            r.type == EntityRelationEnum.STOPPING_AT and r.obj1 == ''
            for r in state.relations
        )
        should_decelerate = any(
            r.type == EntityRelationEnum.YIELDING and r.obj1 == ''
            for r in state.relations
        ) if should_brake == False else False

        if approaching_end:
            should_accelerate = (not should_brake and not should_decelerate and curr_v < target_speed)
        else:
            should_accelerate = (not should_brake and not should_decelerate and curr_v < self.desired_speed)

        if stay_braking:
            traj = longitudinal_brake(route_with_lookahead, 0.0, 0.0, 0.0)
        elif should_brake:
            traj = longitudinal_brake(route_with_lookahead, self.emergency_brake, curr_v)
        elif should_decelerate:
            traj = longitudinal_brake(route_with_lookahead, self.deceleration, curr_v)
        elif approaching_end:
            # Use linear deceleration to stop at end of route
            traj = longitudinal_plan(route_with_lookahead, self.acceleration,
                                     self.deceleration, target_speed, curr_v)
        elif should_accelerate:
            traj = longitudinal_plan(route_with_lookahead, self.acceleration,
                                     self.deceleration, self.desired_speed, curr_v)
        else:
            # Maintain current speed if not accelerating or braking.
            traj = longitudinal_plan(route_with_lookahead, 0.0, self.deceleration, self.desired_speed, curr_v)

        self.t_last = t
        return traj