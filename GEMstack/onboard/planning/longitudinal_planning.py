from typing import List
from ..component import Component
from ...state import AllState, VehicleState, EntityRelation, EntityRelationEnum, Path, Trajectory, Route, ObjectFrameEnum
from ...utils import serialization
from ...mathutils.transforms import vector_madd, normalize_vector, vector_sub
from scipy.optimize import fsolve
import math

DELTA_T = 0.05

# get's the position that we will stop given deceleration and speed
def get_p_stop(init_pos: float, deceleration : float, current_speed : float):
    if current_speed == 0:
        return init_pos
    if deceleration > 0 and current_speed > 0:
        return math.inf 
    p_stop = init_pos + current_speed**2 / (2*deceleration)
    return p_stop

def create_path(init_pos, init_time, init_vel, acceleration, stop_time):
    points = []
    times = []
    cur_time = init_time
    while (cur_time < stop_time):
        cur_time += DELTA_T
        time_diff = cur_time - init_time
        current_pos = init_pos + time_diff * init_vel + 0.5 * time_diff**2 * acceleration
        points.append(current_pos)
        times.append(cur_time)
    return points, times
    
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

    #################### Method 1 #####################
    #### Solve the equations and compute the path as what we did before ###

    # t_dec = current_speed / deceleration # time to stop from current speed
    # p_dec = 0.5 * current_speed * t_dec # distance to stop from current speed
    # p_path = times[-1] * 1 # total path length

    # # The path may contains three parts: Acceleration, Uniform speed, Deceleration

    # # Case 1: No Acceleration
    # # Acceleration is zero or we are already at speed faster than max_speed or too little time to stop
    # # No need to accelerate, only Uniform speed part and Deceleration part
    # # Time for deceleration part is just t_dec, compute time for uniform speed part
    # if acceleration == 0 or current_speed > max_speed or p_path < p_dec:
    #     p_acc = 0 # no acceleration part
    #     t_acc = 0 # no acceleration part
    #     p_uni = max(p_path - p_dec, 0) # If too little time to stop, deceleration only, so p_uni = 0
    #     t_uni = p_uni / current_speed
    # else: # Cases when we have acceleration part
    #     # Test if we can reach max_speed
    #     t_acc = (max_speed - current_speed) / acceleration # time to accelerate to max speed
    #     t_dec = max_speed / deceleration # time to stop from max speed
    #     p_acc = 0.5 * (max_speed + current_speed) * t_acc # distance to accelerate to max speed
    #     p_dec = 0.5 * max_speed * t_dec # distance to stop from max speed
    #     if p_acc + p_dec > p_path: # Case 2: can not reach max speed, PP Curve
    #         # Solution to the quadratic equation
    #         t_acc = (-(1 + acceleration / deceleration) * current_speed + math.sqrt(((1 + acceleration / deceleration) * current_speed) ** 2 - (acceleration + acceleration ** 2 / deceleration) * (current_speed ** 2 / deceleration - 2 * p_path))) / (acceleration + acceleration ** 2 / deceleration)
    #         max_reached_speed = (current_speed + t_acc * acceleration)
    #         t_dec = max_reached_speed / deceleration
    #         p_acc = 0.5 * (max_reached_speed + current_speed) * t_acc
    #         p_dec = 0.5 * max_reached_speed * t_dec
    #         p_uni = 0
    #         t_uni = 0
    #     else: # Case 3: can reach max speed, PLP Curve
    #         # t_acc and t_dec is what we just compute, no need to change
    #         p_uni = p_path - p_acc - p_dec # Distance to travel with uniform (max) speed
    #         t_uni = p_uni / max_speed

    # # We have computed t_acc, t_uni, t_dec. Compute the path.
    # # current position, time and velocity
    # points_acc, times_acc = create_path(0, 0, current_speed, acceleration, t_acc)
    # # Uniform speed part
    # points_uni, times_uni = create_path(p_acc, t_acc, current_speed + acceleration * t_acc, 0, t_acc + t_uni)
    # # Deceleration part
    # points_dec, times_dec = create_path(p_acc + p_uni, t_acc + t_uni, current_speed + acceleration * t_acc, -deceleration, t_acc + t_uni + t_dec)

    # points_ = points_acc + points_uni + points_dec + [p_acc + p_uni + p_dec] # distance here, need to change to 2D points
    # direction = normalize_vector(vector_sub(points[-1], points[0])) # heading of the vehicle
    # points = [vector_madd(points[0], direction, p) for p in points_]
    # times = times_acc + times_uni + times_dec + [t_acc + t_uni + t_dec]
    #################### Method 1 #####################
    

    #################### Method 2 #####################
    ### Simply travel along the path ### 

    direction = normalize_vector(vector_sub(points[-1], points[0])) # heading of the vehicle
    p_path = times[-1] * 1 # total path length
    point_start = points[0]
    p = 0
    t = 0
    v = current_speed

    points = []
    times = []

    dt = 0.05
    while p <= p_path or v > 0:
        points.append(p)
        times.append(t)

        # What should the acceleration be, accelerate or decelerate
        if 0.5 * v**2 / deceleration + p > p_path:
            a = -deceleration
        else:
            a = acceleration

        # update position, velocity, time
        p = p + v * dt + 0.5 * a * dt**2
        v = min(v + a * dt, max_speed)
        if v < 0:
            break
        t = t + dt

    points = [vector_madd(point_start, direction, p) for p in points] # distance here, need to change to 2D points
    #################### Method 2 #####################

    trajectory = Trajectory(path.frame,points,times)
    return trajectory

def longitudinal_brake(path : Path, deceleration : float, current_speed : float) -> Trajectory:
    """Generates a longitudinal trajectory for braking along a path."""
    #TODO: actually do something to points and times
    points = [p for p in path.points]
    # times = [t for t in path_normalized.times]
    times = []
    pointsr = []

    last_pos = points[-1][0]
    init_pos = points[0][0]
    current_pos = points[0][0]
    init_vel = current_speed
    t_stop = init_vel / deceleration
    p_stop = points[0][0] + init_vel**2 / (2*deceleration)

    time = 0
    times.append(time)
    pointsr.append(points[0])
    while time < t_stop:
        time += DELTA_T
        current_pos = init_pos + time * init_vel - 0.5 * time**2 * deceleration
        pointsr.append((current_pos,0))
        times.append(time)

    if p_stop < last_pos:
        time += DELTA_T
        times.append(time)
        pointsr.append((current_pos,0))

    trajectory = Trajectory(path.frame, pointsr, times)
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
