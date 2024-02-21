from typing import List
from ..component import Component
from ...state import AllState, VehicleState, EntityRelation, EntityRelationEnum, Path, Trajectory, Route, ObjectFrameEnum
from ...utils import serialization
from ...mathutils.transforms import vector_madd, vector_sub, vector_dist, normalize_vector
from ...utils import settings


def longitudinal_plan(path : Path, acceleration : float, deceleration : float, max_speed : float, current_speed : float) -> Trajectory:
    """Generates a longitudinal trajectory for a path with a
    trapezoidal velocity profile. 
    
    1. accelerates from current speed toward max speed
    2. travel along max speed
    3. if at any point you can't brake before hitting the end of the path,
       decelerate with accel = -deceleration until velocity goes to 0.
    """
    dt = settings.get("longitudinal_planning.time_resolution")

    points = []
    times = []

    # Add the first point and time
    points.append(path.points[0])
    times.append(0.0)

    # Calculate the full path length and distance to each intermediate point
    path_length = 0
    milestone_distances = [0]
    for i in range(len(path.points) - 1):
        path_length += vector_dist(path.points[i], path.points[i+1])
        milestone_distances.append(path_length)

    dist_traveled = 0
    next_milestone = 1
    while dist_traveled == 0 or current_speed > 0:
        
        # Calculate the distance to decelerate to 0 from current speed
        decel_distance = current_speed **2 / (2 * deceleration)

        # If the distance to the end of the path is less than the distance traveled during deceleration,
        # then we can't brake in time, so just decelerate to 0
        if (path_length - dist_traveled) <= decel_distance:
            current_acceleration = -1 * deceleration
        else:
            current_acceleration = acceleration

        # Update the current speed
        current_speed = current_speed + current_acceleration * dt
        if current_speed >= max_speed:
            current_speed = max_speed
            current_acceleration = 0

        # Update the distance traveled and check if we've reached the next milestone
        dist_traveled += current_speed * dt
        if dist_traveled >= milestone_distances[next_milestone] and next_milestone < len(milestone_distances) - 1:
            next_milestone += 1

        # Calculate the next point
        displacement = vector_sub(path.points[next_milestone], path.points[next_milestone - 1])
        distance_travelled_in_segment = dist_traveled - milestone_distances[next_milestone - 1]
        next_point = vector_madd(path.points[next_milestone - 1], normalize_vector(displacement), distance_travelled_in_segment)

        # Add the next point and time
        points.append(next_point)
        times.append(times[-1] + dt)

    trajectory = Trajectory(path.frame,points,times)
    return trajectory


def longitudinal_brake(path : Path, deceleration : float, current_speed : float) -> Trajectory:
    """Generates a longitudinal trajectory for braking along a path."""
    dt = settings.get("longitudinal_planning.time_resolution")
    
    points = []
    times = []

    # Add the first point and time
    points.append(path.points[0])
    times.append(0.0)

    # Calculate the full path length and distance to each intermediate point
    path_length = 0
    milestone_distances = [0]
    for i in range(len(path.points) - 1):
        path_length += vector_dist(path.points[i], path.points[i+1])
        milestone_distances.append(path_length)

    dist_traveled = 0
    next_milestone = 1
    while current_speed > 0:

        # Update the current speed
        current_speed = current_speed - deceleration * dt

        # Update the distance traveled and check if we've reached the next milestone
        dist_traveled += current_speed * dt + 0.5 * (-1 * deceleration) * dt**2
        if dist_traveled >= milestone_distances[next_milestone] and next_milestone < len(milestone_distances) - 1:
            next_milestone += 1

        # Calculate the next point
        displacement = vector_sub(path.points[next_milestone], path.points[next_milestone - 1])
        distance_travelled_in_segment = dist_traveled - milestone_distances[next_milestone - 1]
        next_point = vector_madd(path.points[next_milestone - 1], normalize_vector(displacement), distance_travelled_in_segment)

        # Add the next point and time
        points.append(next_point)
        times.append(times[-1] + dt)

    trajectory = Trajectory(path.frame,points,times)
    return trajectory


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
