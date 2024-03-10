from typing import List
from ..component import Component
from ...state import AllState, VehicleState, EntityRelation, EntityRelationEnum, Path, Trajectory, Route, ObjectFrameEnum
from ...utils import serialization, settings
from ...mathutils.transforms import vector_madd, vector_dist

import os

DELTA_T = 0.05

def longitudinal_plan(path: Path, acceleration: float, deceleration: float, max_speed: float, current_speed: float) -> Trajectory:
    
    # Normalize the path for consistent parameterization
    normalized_path = path.arc_length_parameterize()

    # Extract start and end points for easier reference
    start_point, end_point = normalized_path.points[0], normalized_path.points[-1]

    # Initialize trajectory points and times with the start point at time 0
    trajectory_points, trajectory_times = [start_point], [0.0]

    # Initialize current state
    current_point, current_time = trajectory_points[0], trajectory_times[0]

    # Flag to indicate if braking is required
    brake = False

    # Simulation loop until reaching the end of the path or completion of braking
    while current_point[0] < end_point[0] and abs(end_point[0] - current_point[0]) > 0.0001:
        # Increment time
        current_time += DELTA_T

        # Calculate stopping distance based on current speed and deceleration
        stopping_distance = (current_speed ** 2) / (2 * deceleration)

        # Calculate distance to the end point
        distance_to_end_point = vector_dist(current_point, end_point)

        # Check if braking is needed
        if not brake and distance_to_end_point <= stopping_distance:
            brake = True

        # Update trajectory based on acceleration or braking
        if not brake:
            # Accelerate to achieve max speed if not already at max speed
            if abs(max_speed - current_speed) > 0.0001:
                current_x = current_point[0] + current_speed * DELTA_T + 0.5 * acceleration * (DELTA_T ** 2)
                current_point = (current_x, 0)
                current_speed += acceleration * DELTA_T
            else:
                # Stay at max speed
                current_speed = max_speed
                current_point = (current_point[0] + current_speed * DELTA_T, 0)
        else:
            # Brake if current speed is greater than 0
            if current_speed > 0:
                current_x = current_point[0] + current_speed * DELTA_T - 0.5 * deceleration * (DELTA_T ** 2)
                current_point = (current_x, 0)
                current_speed -= deceleration * DELTA_T
            else:
                # Finish braking if current speed is already 0
                break

        # Append current state to the trajectory
        trajectory_points.append(current_point)
        trajectory_times.append(current_time)

    # Create and return the resulting trajectory
    result_trajectory = Trajectory(path.frame, trajectory_points, trajectory_times)
    return result_trajectory

def longitudinal_brake(path: Path, deceleration: float, current_speed: float) -> Trajectory:
    """Generates a longitudinal trajectory for braking along a path."""
    # Normalize the path for consistent parameterization
    path_normalized = path.arc_length_parameterize()

    # Initialize time and position arrays with initial values
    times, points = [0], [path_normalized.points[0]]

    # Initialize current state
    current_position = points[0][0]
    # Case when starting at rest
    if current_speed == 0:
        while times[-1] <= path_normalized.times[-1]:
            times.append(times[-1] + DELTA_T)
            points.append((current_position, 0))

    # Braking phase
    while times[-1] <= 10 and current_speed > 0:
        # Decelerate and update speed
        current_speed -= deceleration * DELTA_T
        current_speed = max(0, current_speed)

        # Update position based on current speed
        displacement = current_speed * DELTA_T
        current_position += displacement

        # Append current state to the trajectory
        times.append(times[-1] + DELTA_T)
        points.append((current_position, 0))

    # Create and return the resulting trajectory
    braking_trajectory = Trajectory(path.frame, points, times)
    return braking_trajectory

class YieldTrajectoryPlanner(Component):
    """Follows the given route.  Brakes if you have to yield or
    you are at the end of the route, otherwise accelerates to
    the desired speed.
    """
    def __init__(self):
        self.route_progress = None
        self.t_last = None
        self.acceleration = 0.5
        self.desired_speed = 1.0
        self.deceleration = 2.0

    def state_inputs(self):
        return ['all']

    def state_outputs(self) -> List[str]:
        return ['trajectory']

    def rate(self):
        return 10.0

    def update(self, state : AllState):
        vehicle = state.vehicle # type: VehicleState
        route = state.route   # type: Route
        t = state.t

        if self.t_last is None:
            self.t_last = t
        dt = t - self.t_last
  
        curr_x = vehicle.pose.x
        curr_y = vehicle.pose.y
        curr_v = vehicle.v

        #figure out where we are on the route
        if self.route_progress is None:
            self.route_progress = 0.0
        closest_dist,closest_parameter = state.route.closest_point_local((curr_x,curr_y),[self.route_progress-5.0,self.route_progress+5.0])
        self.route_progress = closest_parameter

        #extract out a 10m segment of the route
        route_with_lookahead = route.trim(closest_parameter,closest_parameter+10.0)

        #parse the relations indicated
        should_brake = False
        for r in state.relations:
            if r.type == EntityRelationEnum.YIELDING and r.obj1 == '':
                #yielding to something, brake
                should_brake = True
        should_accelerate = (not should_brake and curr_v < self.desired_speed)

        #choose whether to accelerate, brake, or keep at current velocity
        if should_accelerate:
            traj = longitudinal_plan(route_with_lookahead, self.acceleration, self.deceleration, self.desired_speed, curr_v)
        elif should_brake:
            traj = longitudinal_brake(route_with_lookahead, self.deceleration, curr_v)
        else:
            traj = longitudinal_plan(route_with_lookahead, 0.0, self.deceleration, self.desired_speed, curr_v)

        return traj 
