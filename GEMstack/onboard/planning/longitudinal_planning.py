from typing import List
from ..component import Component
from ...state import AllState, VehicleState, EntityRelation, EntityRelationEnum, Path, Trajectory, Route, ObjectFrameEnum
from ...utils import serialization
from ...mathutils.transforms import vector_madd, vector_sub, vector_dist

from scipy.optimize import fsolve
from numpy import arange

def longitudinal_plan(path : Path, acceleration : float, deceleration : float, max_speed : float, current_speed : float) -> Trajectory:
    """Generates a longitudinal trajectory for a path with a
    trapezoidal velocity profile. 
    
    1. accelerates from current speed toward max speed
    2. travel along max speed
    3. if at any point you can't brake before hitting the end of the path,
       decelerate with accel = -deceleration until velocity goes to 0.
    """
    print("path {}".format(path.points))
    path_normalized = path.arc_length_parameterize()
    #TODO: actually do something to points and times
    points = []
    times = []

    print("acceleration {}, deceleration {}, max_speed {}, current_speed {}".format(acceleration, deceleration, max_speed, current_speed))

    print("path_normalized")
    print(path_normalized.points)

    # Add the first point and time
    points.append(path_normalized.points[0])
    times.append(0.0)

    # For each segment, calculate a trapezoidal velocity profile
    cur_point = path_normalized.points[0]
    for next_point in path_normalized.points[1:]:
        print("cur_point")
        print(cur_point)

        print("next_point")
        print(next_point)

        # Calculate the distance to the next point
        distance = vector_dist(cur_point, next_point)

        # Calculate the time to accelerate to max speed
        accel_time = (max_speed - current_speed) / acceleration
        print("accel_time")
        print(accel_time)

        # Calculate the distance traveled during acceleration
        accel_distance = current_speed * accel_time + 0.5 * acceleration * accel_time**2
        print("accel_distance")
        print(accel_distance)

        # Calculate the time to decelerate to 0 from max speed
        decel_time = max_speed / deceleration
        print("decel_time")
        print(decel_time)

        # Calculate the distance traveled during deceleration
        decel_distance = max_speed * decel_time - 0.5 * deceleration * decel_time**2
        print("decel_distance")
        print(decel_distance)

        # If the distance to the next point is less than the distance traveled during acceleration and deceleration, then we can't brake in time    
        if distance < accel_distance + decel_distance:
            ndec = - 1.0 * deceleration
            resolution = 0.06
            print("triangle")
            a = 0.5 * (acceleration + acceleration**2 / deceleration)
            b = current_speed + acceleration * current_speed / deceleration
            c = -1 * distance + current_speed**2 / (2 * deceleration)

            t = (-1 * b + (b**2 - 4 * a * c)**0.5) / (2 * a)
            td = (current_speed + acceleration * t) / deceleration

            def f(t):
                return -1.0 * distance + current_speed * t + 0.5 * acceleration * t**2 + (0 - acceleration * t + current_speed) * (acceleration * t + current_speed) / (deceleration) + 0.5 * deceleration * ((0 - acceleration * t + current_speed) / deceleration)**2

            tstop = fsolve(f, 0.0)

            print("tstop")
            print(tstop)

            print("t")
            print(t)

            td = (current_speed + acceleration * t) / deceleration
            # Euler integration to find points and times at the resolution
            int_point = cur_point
            int_speed = current_speed
            for _ in arange(0, t, resolution):
                # Get the time of the intermediate point
                times.append(times[-1] + resolution)

                # Get the distance of the intermediate point
                distance_traveled = int_speed * resolution + 0.5 * acceleration * resolution**2

                int_distance = vector_dist(int_point, next_point)

                # Add an intermediate point
                next_int_point = vector_madd(int_point, vector_sub(next_point, int_point), distance_traveled / int_distance)
                points.append(next_int_point)

                # Update the current point
                int_point = next_int_point
                int_speed = int_speed + acceleration * resolution


            for _ in arange(0, td, resolution):
                # Get the time of the intermediate point
                times.append(times[-1] + resolution)

                # Get the distance of the intermediate point
                distance_traveled = int_speed * resolution + 0.5 * ndec * resolution**2

                int_distance = vector_dist(int_point, next_point)

                # Add an intermediate point
                next_int_point = vector_madd(int_point, vector_sub(next_point, int_point), distance_traveled / int_distance)
                points.append(next_int_point)

                # Update the current point
                int_point = next_int_point
                int_speed = int_speed + ndec * resolution

            print("int_point")
            print(int_point)
            print(int_speed)

            # Get the time of the intermediate point
            # times.append(times[-1] + t)

            # Get the distance of the intermediate point
            # distance_traveled = current_speed * t + 0.5 * acceleration * t**2
            
            # Add an intermediate point
            # points.append(vector_madd(cur_point, vector_sub(next_point, cur_point), distance_traveled / distance))

            # Add the next point
            # points.append(next_point)

            # Add the time to decelerate to 0
            # td = (current_speed + acceleration * t) / deceleration
            # times.append(times[-1] + td)

            # Update the current point
            cur_point = int_point
            current_speed = 0.0

            print("times[-1] {}".format(times[-1]))

        else:
            # Calculate the time to travel at max speed
            travel_time = (distance - accel_distance - decel_distance) / max_speed
            print("travel_time")
            print(travel_time)

            # Add the time to accelerate to max speed
            times.append(times[-1] + accel_time)
            
            # Add an intermediate point at max speed
            points.append(vector_madd(cur_point, vector_sub(next_point, cur_point), accel_distance / distance))

            # Add the time to travel at max speed
            times.append(times[-1] + travel_time)
            
            # Add an intermediate point at max speed
            points.append(vector_madd(cur_point, vector_sub(next_point, cur_point), (accel_distance + distance - decel_distance) / distance))

            # Add the time to decelerate to 0 from max speed
            times.append(times[-1] + decel_time)
            
            # Add the next point
            points.append(next_point)
            
            # Update the current point
            cur_point = next_point
            current_speed = 0.0

    print(points)
    print(times)

    trajectory = Trajectory(path.frame,points,times)
    return trajectory


def longitudinal_brake(path : Path, deceleration : float, current_speed : float) -> Trajectory:
    """Generates a longitudinal trajectory for braking along a path."""
    path_normalized = path.arc_length_parameterize()
    #TODO: actually do something to points and times
    points = [p for p in path_normalized.points]
    times = [t for t in path_normalized.times]
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
