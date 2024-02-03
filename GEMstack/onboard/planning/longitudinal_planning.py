from typing import List
from ..component import Component
from ...state import AllState, VehicleState, EntityRelation, EntityRelationEnum, Path, Trajectory, Route, ObjectFrameEnum
from ...utils import serialization
from ...mathutils.transforms import vector_madd
import math

def longitudinal_plan(path : Path, acceleration : float, deceleration : float, max_speed : float, current_speed : float) -> Trajectory:
    """Generates a longitudinal trajectory for a path with a
    trapezoidal velocity profile. 
    
    1. accelerates from current speed toward max speed
    2. travel along max speed
    3. if at any point you can't brake before hitting the end of the path,
       decelerate with accel = -deceleration until velocity goes to 0.
    """
    path_normalized = path.arc_length_parameterize()
    #TODO: actually do something to points and times
    points = [p for p in path_normalized.points]
    times = [t for t in path_normalized.times]
    
    # Function to calculate distance between two points
    def distance(p1, p2):
        return math.sqrt((p2[0] - p1[0]) ** 2 + (p2[1] - p1[1]) ** 2)
    
    path_length = sum(distance(points[i], points[i+1]) for i in range(len(points)-1))
    total_cover_d = 0
    total_t = 0
    
    if acceleration == 0.0:
        # Calculate the time to traverse each segment and accumulate
        current_time = 0
        v_c = max_speed
        total_time = path_length / v_c
        times = [i * total_time / (len(points) - 1) for i in range(len(points))]
   
        # Create the trajectory object with the calculated points and times
        trajectory = Trajectory(frame=path.frame, points=points, times=times)
    
    else:
        #calculate the time for acceleration
        t_a = (max_speed - current_speed) / acceleration
        #calculate the distance for acceleration
        d_a = current_speed * t_a + 0.5 * acceleration * (t_a ** 2)
        #calculate the distance for deceleration
        d_d = (max_speed ** 2)/(2 * -deceleration)

        d_total = d_a + d_d #total distance
        # path_length = path_normalized.calculate_path_length()


        #determine if there is enough distance to reach max_speed and then decelerate to 0:
        if d_total > path_length:
            d_max = path_length / 2
            peak_speed = math.sqrt(current_speed ** 2 + 2 * acceleration * d_max)
            t_a = (peak_speed - current_speed) / acceleration
            d_a = (current_speed + peak_speed) / 2 * t_a
        else:
            peak_speed = max_speed

        #calculate the distance at the max_speed
        d_at_max = path_length - d_total
        #calculate the time at the max_speed
        t_at_max = d_at_max / peak_speed

        #calculate the time to decelerate to 0
        t_d = peak_speed / -deceleration

        #interpolate points and construct time list:
        for i in range(len(points) - 1):
            segment_start = points[i]
            segment_end = points[i + 1]
            segment_length = distance(segment_start, segment_end)
            segment_time = 0

            #Determine the state (accelerating, cruising, decelerating) and calculate segment time:
            if total_cover_d < d_a:
                #Accelerating
                segment_time = (math.sqrt(current_speed**2 + 2 * acceleration * segment_length) - current_speed) / acceleration
                current_speed += acceleration * segment_time
            elif total_cover_d < d_a + d_at_max:
                #Cruising
                segment_time = segment_length / peak_speed
            else:
                segment_time = (current_speed - math.sqrt(current_speed**2 - 2 * deceleration * segment_length)) / deceleration
                current_speed = max(current_speed - deceleration * segment_time, 0)
            
            total_t += segment_time
            times[i] = total_t
            total_cover_d += segment_length

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
