from typing import List
from ..component import Component
from ...state import AllState, VehicleState, EntityRelation, EntityRelationEnum, Path, Trajectory, Route, ObjectFrameEnum
from ...utils import serialization
from ...mathutils.transforms import vector_madd
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
    # brake down into 3 phases
    # accelerate until max speed
    # travel max speed
    # decelerate until stopping
    # how do we figure out when to do the deceleration phase?
    import numpy as np
    dt = 0.05
    if acceleration <= 0:
        max_speed = current_speed
        acceleration = -1
    p_end = points[-1][0] - points[0][0]
    t_dec = current_speed / deceleration
    p_start = points[0][0]
    if p_end < 0.5 * deceleration * t_dec ** 2:
        # t_end = (current_speed - np.sqrt(current_speed ** 2 - 2 * deceleration * p_end)) / deceleration
        t_end = t_dec
        t_path = np.linspace(0, t_end, int(t_end / dt))
        p_path = p_start + current_speed * t_path - 0.5 * deceleration * t_path ** 2
        p_path = p_path.tolist()
    else:
        t_dec_from_max = max_speed / deceleration
        t_acc_to_max = (max_speed - current_speed) / acceleration
        if p_end < 0.5 * (acceleration * t_acc_to_max ** 2 + deceleration * t_dec_from_max ** 2):
            if acceleration != -1:
                t_acc = (-(1 + acceleration / deceleration) * current_speed + np.sqrt(((1 + acceleration / deceleration) * current_speed) ** 2 - (acceleration + acceleration ** 2 / deceleration) * (current_speed ** 2 / deceleration - 2 * p_end))) / (acceleration + acceleration ** 2 / deceleration)
                t_acc_ = np.linspace(0, t_acc, int(t_acc / dt))
                p_acc = p_start + current_speed * t_acc_ + 0.5 * acceleration * t_acc_ ** 2
            t_dec = (t_acc * acceleration + current_speed) / deceleration
            t_end = t_dec + t_acc if acceleration != -1 else 0
            t_dec_ = np.linspace(0, t_dec, int(t_dec / dt))
            p_last = p_acc[-1] if acceleration >= 0 else p_start
            p_dec = p_last - 0.5 * deceleration * t_dec_ ** 2 + (t_acc * acceleration + current_speed) * t_dec_
            p_path = p_dec.tolist()
            t_path = (t_dec_+ (t_acc if acceleration != -1 else 0)).tolist()
            if acceleration != -1:
                p_path = p_acc.tolist() + p_path
                t_path = t_acc_.tolist() + t_path
        else:
            if acceleration != -1:
                t_acc = t_acc_to_max
                t_acc_ = np.linspace(0, t_acc, int(t_acc / dt))
                p_acc = p_start + current_speed * t_acc_ + 0.5 * acceleration * t_acc_ ** 2
            t_uni = (p_end - 0.5 * acceleration * t_acc_to_max ** 2 - 0.5 * deceleration * t_dec_from_max ** 2 - current_speed * t_acc_to_max) / max_speed
            t_uni_ = np.linspace(0, t_uni, int(t_uni / dt))
            p_last = p_acc[-1] if acceleration >= 0 else p_start
            p_uni = t_uni_ * max_speed + p_last
            t_dec_ = np.linspace(0, t_dec_from_max, int(t_dec_from_max / dt))
            p_dec = p_uni[-1] - 0.5 * deceleration * t_dec_ ** 2 + max_speed * t_dec_
            p_path = p_uni.tolist() + p_dec.tolist()
            t_path = (t_uni_ + (t_acc if acceleration != -1 else 0)).tolist() + (t_dec_+ (t_acc if acceleration != -1 else 0) + t_uni).tolist()
            if acceleration != -1:
                p_path = p_acc.tolist() + p_path
                t_path = t_acc_.tolist() + t_path
    points = [(p, 0) for p in p_path]
    times = [t for t in t_path]
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
