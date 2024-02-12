from typing import List
from ..component import Component
from ...state import AllState, VehicleState, EntityRelation, EntityRelationEnum, Path, Trajectory, Route, ObjectFrameEnum
from ...utils import serialization, settings
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
    times = [t/(path_normalized.times[-1]) for t in path_normalized.times]

    total_distance = path_normalized.times[-1]

    # # If acceleration is zero, use triangular profile with non-jerk for smooth transition 
    if acceleration == 0:
        jerk = -10 #m/s^3
        # Calculate transition time
        transition_time = math.sqrt(2 * abs(current_speed) / abs(jerk))
        const_speed_time = abs(current_speed) / abs(jerk)
        total_time = transition_time + const_speed_time
        times = [total_time * t for t in times]
        return Trajectory(path.frame, points, times)

    if current_speed > max_speed:
        #decelerate
        total_time = current_speed/deceleration
        times = [total_time * t for t in times]
        return Trajectory(path.frame, points, times)
    
    decel_dist_curr_speed = current_speed**2/(2*deceleration)
    decel_dist_max_speed = max_speed**2/(2*deceleration)
    accel_dist_curr_to_max = (max_speed**2 - current_speed**2) / (2 * acceleration)


    #if we'll reach max speed and decelerate
    if total_distance-decel_dist_max_speed >= accel_dist_curr_to_max:
        accel_time = (max_speed-current_speed)/acceleration
        decel_time = math.sqrt(2*decel_dist_max_speed/deceleration)
        const_velocity_time = (total_distance-decel_dist_max_speed-accel_dist_curr_to_max)/max_speed
        total_time = accel_time+decel_time+const_velocity_time
        times = [total_time * t for t in times]
        return Trajectory(path.frame, points, times)
    

    #still have dist to accelerate, accel
    dt = 0.005 #seconds
    total_time = 0.0
    while total_distance-decel_dist_curr_speed > 0:
        #accelerate
        current_speed = current_speed + acceleration*dt
        total_time += dt
        decel_dist_curr_speed = current_speed**2/(2*deceleration)
    total_time += math.sqrt(2*decel_dist_curr_speed/deceleration)
    times = [total_time * t for t in times]
    return Trajectory(path.frame, points, times)

    return Trajectory(path.frame, points, times)


def longitudinal_brake(path : Path, deceleration : float, current_speed : float) -> Trajectory:
    """Generates a longitudinal trajectory for braking along a path."""
    path_normalized = path.arc_length_parameterize()
    #TODO: actually do something to points and times

    if path_normalized.times[-1] == 0:
        return Trajectory(path.frame,path_normalized.points,path_normalized.times)
    
    points = [p for p in path_normalized.points]
    times = [t/(path_normalized.times[-1]) for t in path_normalized.times]

    # Compute the time needed to decelerate to stop
    decel_time = current_speed / deceleration

    times = [t * decel_time for t in times] 
    return Trajectory(path.frame,points,times)


class YieldTrajectoryPlanner(Component):
    """Follows the given route.  Brakes if you have to yield or
    you are at the end of the route, otherwise accelerates to
    the desired speed.
    """
    def __init__(self):
        self.route_progress = None
        self.t_last = None
        self.acceleration = settings.get('control.longitudinal_planning_yielding.acceleration')
        self.desired_speed = settings.get('control.longitudinal_planning_yielding.desired_speed')
        self.deceleration = settings.get('control.longitudinal_planning_yielding.deceleration')

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
