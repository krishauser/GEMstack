from typing import List
from ..component import Component
from ...state import AllState, VehicleState, EntityRelation, EntityRelationEnum, Path, Trajectory, Route, ObjectFrameEnum
from ...utils import serialization
from ...mathutils.transforms import vector_madd

# def longitudinal_plan(path : Path, acceleration : float, deceleration : float, max_speed : float, current_speed : float) -> Trajectory:
#     """Generates a longitudinal trajectory for a path with a
#     trapezoidal velocity profile. 
    
#     1. accelerates from current speed toward max speed
#     2. travel along max speed
#     3. if at any point you can't brake before hitting the end of the path,
#        decelerate with accel = -deceleration until velocity goes to 0.
#     """
#     path_normalized = path.arc_length_parameterize()
#     #TODO: actually do something to points and times
#     points = [p for p in path_normalized.points]
#     times = [t for t in path_normalized.times]
#     trajectory = Trajectory(path.frame,points,times)
#     return trajectory


def longitudinal_plan(path: Path, acceleration: float, deceleration: float, max_speed: float, current_speed: float) -> Trajectory:
    """
    Computes a trajectory along a path using a trapezoidal velocity profile. This profile includes acceleration
    towards a maximum speed, maintaining this speed, and decelerating to a halt if necessary to avoid overshooting
    the path's endpoint.
    """
    path_parameterized = path.arc_length_parameterize()
    current_position = 0.0
    velocity = current_speed
    final_position = path_parameterized.times[-1]
    
    elapsed_time = 0.0
    time_increment = 0.05
    time_stamps = []
    position_points = []
    while current_position < final_position:
        time_stamps.append(elapsed_time)
        position_points.append(path_parameterized.eval(current_position))
        safe_stop_position = final_position - (velocity**2) / (2 * deceleration)
        if current_position < safe_stop_position and velocity < max_speed:
            velocity += acceleration * time_increment
        elif current_position >= safe_stop_position or velocity > max_speed:
            velocity -= deceleration * time_increment
            velocity = max(velocity, 0.0)
        current_position += velocity * time_increment
        elapsed_time += time_increment
        if velocity == 0.0:
            break
    if current_position > final_position:
        # Adjust for overshooting with braking
        adjustment_direction = path_parameterized.eval_derivative(path_parameterized.times[-1])
        time_stamps.append(elapsed_time)
        overshoot_distance = current_position - final_position
        position_points.append(vector_madd(path_parameterized.points[-1], adjustment_direction, overshoot_distance))
        while velocity > 0:
            velocity -= deceleration * time_increment
            velocity = max(0.0, velocity)
            overshoot_point = vector_madd(position_points[-1], adjustment_direction, velocity * time_increment)
            elapsed_time += time_increment
            time_stamps.append(elapsed_time)
            position_points.append(overshoot_point)
            if velocity == 0:
                break
    else:
        time_stamps.append(elapsed_time)
        position_points.append(path_parameterized.eval(current_position))
    return Trajectory(path.frame, position_points, time_stamps)





def longitudinal_brake(path : Path, deceleration : float, current_speed : float) -> Trajectory:
    """Generates a longitudinal trajectory for braking along a path."""
    path_normalized = path.arc_length_parameterize()
    #TODO
    current_position = 0.0
    velocity = current_speed
    final_position = path_normalized.times[-1]
    
    elapsed_time = 0.0
    time_increment = 0.05
    time_stamps = []
    position_points = []
    while current_position < final_position:
        time_stamps.append(elapsed_time)
        position_points.append(path_normalized.eval(current_position))
        velocity -= deceleration * time_increment
        velocity = max(0.0, velocity)
        current_position += velocity * time_increment
        elapsed_time += time_increment
        if velocity == 0.0:
            break
    if current_position > final_position:
        # Handle braking beyond endpoint
        adjustment_direction = path_normalized.eval_derivative(path_normalized.times[-1])
        time_stamps.append(elapsed_time)
        overshoot_distance = current_position - final_position
        position_points.append(vector_madd(path_normalized.points[-1], adjustment_direction, overshoot_distance))
        while velocity > 0:
            velocity -= deceleration * time_increment
            velocity = max(0.0, velocity)
            overshoot_point = vector_madd(position_points[-1], adjustment_direction, velocity * time_increment)
            elapsed_time += time_increment
            time_stamps.append(elapsed_time)
            position_points.append(overshoot_point)
            if velocity == 0:
                break
    else:
        time_stamps.append(elapsed_time)
        position_points.append(path_normalized.eval(current_position))
    return Trajectory(path.frame, position_points, time_stamps)


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

        if self.route_progress is None:
            self.route_progress = 0.0
        closest_dist,closest_parameter = state.route.closest_point_local((curr_x,curr_y),[self.route_progress-5.0,self.route_progress+5.0])
        route_with_lookahead = route.trim(closest_parameter,closest_parameter+10.0)
        self.route_progress = closest_parameter
        
        should_brake = False
        for r in state.relations:
            if r.type == EntityRelationEnum.YIELDING and r.obj1 == '':
                #yielding to something, brake
                should_brake = True
        should_accelerate = (not should_brake and curr_v < self.desired_speed)

        if should_accelerate:
            traj = longitudinal_plan(route_with_lookahead, self.acceleration, self.deceleration, self.desired_speed, curr_v)
        elif should_brake:
            traj = longitudinal_brake(route_with_lookahead, self.deceleration, curr_v)
        else:
            traj = longitudinal_plan(route_with_lookahead, 0.0, self.deceleration, self.desired_speed, curr_v)

        return traj 
