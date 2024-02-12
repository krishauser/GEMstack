from typing import List
from ..component import Component
from ...state import AllState, VehicleState, EntityRelation, EntityRelationEnum, Path, Trajectory, Route, ObjectFrameEnum
from ...utils import serialization
from ...mathutils.transforms import vector_madd, vector_sub, vector_dist, normalize_vector

from numpy import arange

def longitudinal_plan(path : Path, acceleration : float, deceleration : float, max_speed : float, current_speed : float) -> Trajectory:
    """Generates a longitudinal trajectory for a path with a
    trapezoidal velocity profile. 
    
    1. accelerates from current speed toward max speed
    2. travel along max speed
    3. if at any point you can't brake before hitting the end of the path,
       decelerate with accel = -deceleration until velocity goes to 0.
    """
    # If the path follows the trapezoidal velocity profile, then they are all going to be in a line. 
    # So we can just take the first and last. 
    # path.points = [path.points[0], path.points[-1]]

    print("path {}".format(path.points))
    path_normalized = path.arc_length_parameterize()
    points = []
    times = []

    print("acceleration {}, deceleration {}, max_speed {}, current_speed {}".format(acceleration, deceleration, max_speed, current_speed))

    print("path_normalized")
    print(path_normalized.points)

    cur_point = path_normalized.points[0]
    next_point = path_normalized.points[-1]
    disp = normalize_vector(vector_sub(next_point, cur_point))

    # Add the first point and time
    points.append(path_normalized.points[0])
    times.append(0.0)

    # Resolution for Euler integration
    resolution = 0.05

    def euler_integration(t, resolution, current_speed, acceleration, current_point):
        int_speed = current_speed
        int_point = current_point
        for _ in arange(0, t, resolution):
            
            # Get the time of the intermediate point
            times.append(times[-1] + resolution)

            # Get the distance of the intermediate point
            distance_traveled = int_speed * resolution + 0.5 * acceleration * resolution**2

            # Add an intermediate point
            next_int_point = vector_madd(int_point, disp, distance_traveled)
            points.append(next_int_point)

            # Update the current point
            int_point = next_int_point
            int_speed = int_speed + acceleration * resolution

        return int_point, int_speed
    
    def quad_fmla(a, b, c):
        return (-1 * b + (b**2 - 4 * a * c)**0.5) / (2 * a) 

    # Set the maximum achievable speed
    if acceleration == 0:
        max_speed = current_speed
            
    # Calculate the distance to the next point
    distance = vector_dist(cur_point, next_point)

    # Calculate the time and dist to accelerate to max speed
    accel_time = (max_speed - current_speed) / acceleration if acceleration != 0 else 0.0
    accel_distance = current_speed * accel_time + 0.5 * acceleration * accel_time**2

    # Calculate the time and dist to decelerate to 0 from max speed
    decel_time = max_speed / deceleration if deceleration != 0 else 0.0
    decel_distance = max_speed * decel_time - 0.5 * deceleration * decel_time**2

    # If the distance to the next point is less than the distance traveled during acceleration 
    # and deceleration, then we can't brake in time
    triangle_case = distance < accel_distance + decel_distance

    if triangle_case:
        print("triangle")

        # Check if we can brake in time from the current speed
        if current_speed**2 / (2 * deceleration) > distance:
            print("doomed")
            accel_time = 0 # Do not accelerate
            decel_time = current_speed / deceleration # time to decelerate to 0

        else:
            # use quadratic fmla to calculate accel and decel time. 
            a = 0.5 * (acceleration + acceleration**2 / deceleration)
            b = current_speed + acceleration * current_speed / deceleration
            c = -1 * distance + current_speed**2 / (2 * deceleration)

            accel_time = quad_fmla(a, b, c) # time accelerating
            decel_time = (current_speed + acceleration * accel_time) / deceleration # time decellerating
    else:
        print("not triangle")
    
    # Euler integration to find points and times for accelerating
    cur_point, current_speed = euler_integration(accel_time, resolution, current_speed, acceleration, cur_point)
    print("finished accel", cur_point, current_speed)
    
    # Calculate the time to travel at max speed and add the points and times
    straight_time = 0 if triangle_case else (distance - accel_distance - decel_distance) / max_speed
    cur_point, current_speed = euler_integration(straight_time, resolution, current_speed, 0, cur_point)
    print("straights", cur_point, current_speed)

    # Euler integration to find points and times for decelerating
    cur_point, current_speed = euler_integration(decel_time, resolution, current_speed, -1 * deceleration, cur_point)
    print("finished decel", cur_point, current_speed)

    print("----------")
    trajectory = Trajectory(path.frame,points,times)
    return trajectory


def longitudinal_brake(path : Path, deceleration : float, current_speed : float) -> Trajectory:
    """Generates a longitudinal trajectory for braking along a path."""
    path_normalized = path.arc_length_parameterize()
    #TODO: actually do something to points and times
    points = []
    times = []

    # Add the first point and time
    points.append(path_normalized.points[0])
    times.append(0.0)

    if current_speed == 0:
        return Trajectory(path.frame, points, times)

    cur_point = path_normalized.points[0]
    next_point = path_normalized.points[-1]
    disp = normalize_vector(vector_sub(next_point, cur_point))

    # Resolution for Euler integration
    resolution = 0.05

    def euler_integration(t, resolution, current_speed, acceleration, current_point):
        int_speed = current_speed
        int_point = current_point
        for _ in arange(0, t, resolution):
            
            # Get the time of the intermediate point
            times.append(times[-1] + resolution)

            # Get the distance of the intermediate point
            distance_traveled = int_speed * resolution + 0.5 * acceleration * resolution**2

            # Add an intermediate point
            next_int_point = vector_madd(int_point, disp, distance_traveled)
            points.append(next_int_point)

            # Update the current point
            int_point = next_int_point
            int_speed = int_speed + acceleration * resolution

        return int_point, int_speed

    # Calculate the distance to the next point
    distance = vector_dist(cur_point, next_point)

    # Calculate the time and dist to decelerate to 0 from current speed
    decel_time = current_speed / deceleration

    # Euler integration to find points and times for decelerating
    cur_point, current_speed = euler_integration(decel_time, resolution, current_speed, -1 * deceleration, cur_point)

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
            if r.type == EntityRelationEnum.YIELD and r.obj1 == '':
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
