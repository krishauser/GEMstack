from typing import List
from ..component import Component
from ...state import AllState, VehicleState, EntityRelation, EntityRelationEnum, Path, Trajectory, Route, ObjectFrameEnum
from ...utils import serialization
from ...mathutils.transforms import vector_madd
from scipy.optimize import fsolve
import math

# get's the position that we will stop given deceleration and speed
def get_p_stop(init_pos: float, deceleration : float, current_speed : float):
    if current_speed == 0:
        return init_pos
    if deceleration > 0 and current_speed > 0:
        return math.inf 
    p_stop = init_pos + current_speed**2 / (2*deceleration)
    return p_stop

    
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
    path_end = points[-1][0]
    p0 = points[0][0]

    # we can use the formula for min time PP curve to solve this equation
    x1 = p0
    v1 = current_speed
    x2 = path_end
    v2 = 0 # since once we reach end position, we want to be at a complete stop
    a1 = acceleration
    a2 = -deceleration
    vl = max_speed
    # we will have 5 equations and 5 unknowns if we can reach max speed(ts1, ts2, xs1, xs2, T, Vs, xs)
    # Define the equations based on the image provided
    # we want the position when we reach maximum velocity
    delta_v = vl - v1
    ts1 = delta_v / a1
    xs1 = x1 + ts1*v1 + 0.5*(ts1**2)*a1
    # first we have to figure out if we can reach the max velocity (i.e) we can stop before xs at xs1 with speed vl
    p_stop = get_p_stop(xs1, deceleration, vl)
    if p_stop > x2:
        # use first equation to create path since we can't reach the max velocity without stopping

    else:
        z = (v2 - vl) / a2
        ts2 = ((x2 - xs1 - 0.5*z**2*a2) / vl) + ts1 - z
        T = ts2 + z
        points = []
        times = []
        t_step = 0.05
        cur_time = 0
        
    
    
    # brake down into 3 phases 
    # accelerate until max speed 

    # travel max speed

    # decelerate until stopping
    # how do we figure out when to do the deceleration phase?


    trajectory = Trajectory(path.frame,points,times)
    return trajectory


def longitudinal_brake(path : Path, deceleration : float, current_speed : float) -> Trajectory:
    """Generates a longitudinal trajectory for braking along a path."""
    #TODO: actually do something to points and times
    points = [p for p in path.points]
    # times = [t for t in path_normalized.times]
    times = [0]

    print("points before", [p[0] for p in points])
    print('times before', times)
    last_pos = points[-1][0]
   
    init_speed = current_speed
    t_stop = init_speed / deceleration
    p_stop = points[0][0] + init_speed**2 / (2*deceleration)
    print("t_stop", t_stop) 
    print("p_stop", p_stop)
    last_reach_point_idx = 0
    for i in range(1,len(points)):
        position = points[i][0]
        if position <= p_stop:
            displacement = position - points[0][0]
            #print(displacement)
            time_position = (-init_speed + (init_speed**2 - 2*-deceleration*-displacement)**0.5) / -deceleration
            times.append(time_position)
            last_reach_point_idx = i
        else:
            break
    points = points[:last_reach_point_idx+1]
    times = times[:last_reach_point_idx+1]
    if p_stop < last_pos:
        times.append(t_stop)
        points.append((p_stop, points[last_reach_point_idx][1]))
    
    print("points after", [p[0] for p in points])
    print('times after', times)
    
    trajectory = Trajectory(path.frame, points, times)
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
