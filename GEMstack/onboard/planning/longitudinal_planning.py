from typing import List
from ..component import Component
from ...state import AllState, VehicleState, EntityRelation, EntityRelationEnum, Path, Trajectory, Route, ObjectFrameEnum
from ...utils import serialization
from ...mathutils.transforms import vector_madd

def longitudinal_plan(path : Path, acceleration : float, deceleration : float, max_speed : float, current_speed : float) -> Trajectory:
    """Generates a longitudinal trajectory for a path with a
    trapezoidal velocity profile. 
    
    1. accelerates from current speed toward max speed
    2. travel along max speed
    3. if at any point you can't brake before hitting the end of the path,
       decelerate with accel = -deceleration until velocity goes to 0.
    """
    path_normalized = path.arc_length_parameterize()  
    points = [p for p in path_normalized.points]
    points = path_planning_2(sp=points[0][0], ep=points[-1][0], acc=acceleration, dec = deceleration, max_vel = max_speed, t = 0.1, cur_vel = 0)
    points = [(p, 0) for p in points]
    frag = (path_normalized.times[-1] - path_normalized.times[0])/len(points)
    times = []
    xt = 0
    while (xt < path_normalized.times[-1]) :
        times.append(xt)
        xt += frag
    print(points, times)

    trajectory = Trajectory(path.frame,points,times)

    return trajectory

def longitudinal_brake(path : Path, deceleration : float, current_speed : float) -> Trajectory:
    """Generates a longitudinal trajectory for braking along a path."""
    path_normalized = path.arc_length_parameterize()
    points = [p for p in path_normalized.points]
    print(points)
    px = path_planning_2(sp = points[0][0], ep = points[-1][0], dec = deceleration, cur_vel = current_speed, max_vel = current_speed)
    if (len(px) > 0) :
        points = [(p, 0) for p in px]
    frag = (path_normalized.times[-1] - path_normalized.times[0])/(len(points) - 1)
    times = [0]
    xt = 0
    while (xt < path_normalized.times[-1]) :
        times.append(xt)
        xt += frag
    print(points, times)
    trajectory = Trajectory(path.frame,points,times)
    return trajectory

def path_planning(sp : float, ep: float, dec: float, max_vel: float, t : float, acc: float = 0) :
    ps = []
    pe = []
    pm = []
    cur_vel = 0
    instant = 0
    distance_covered_start = 0
    while(cur_vel < max_vel and acc > 0):
        u = cur_vel
        cur_vel = cur_vel + acc*t
        instant += t
        distance_covered_start += u*t + 1/2 *acc * t**2
        ps.append(distance_covered_start + sp)
    
    (pe, distance_covered_end) = braking(dec = dec, ep = ep, cur_vel = max_vel, t = 0.1)

    remaining_distance = ep - distance_covered_start - distance_covered_end - sp
    while(remaining_distance > 0):
        instant += 1
        pm.insert(0, remaining_distance + ps[-1])
        remaining_distance = remaining_distance - max_vel * t
    print("ps:",ps,"pm:",pm,"pe:",pe)
    print(instant)
    return ps + pm + pe


def path_planning_2(sp : float, ep: float, max_vel: float, t : float = 0.1, cur_vel : float = 0, acc: float = 0, dec: float = 0) :
    ps = []
    pe = []
    pm = []
    instant = 0
    distance_covered_start = 0
    distance_covered_end = 0
    end_vel = 0
    while(distance_covered_start + distance_covered_end <= ep - sp and max_vel > 0.01):
        au = cur_vel
        cur_vel = min(cur_vel + acc*t, max_vel)
        instant += t
        distance_covered_start += t*au
        ps.append(distance_covered_start + sp)
        du = end_vel
        end_vel = min(end_vel + dec * t, max_vel)
        #instant += 1
        distance_covered_end += t*du
        pe.insert(0, ep-distance_covered_end)
        
    
    #(pe, distance_covered_end) = braking(dec = dec, ep = ep, cur_vel = max_vel, t = 0.1)

    # remaining_distance = ep - distance_covered_start - distance_covered_end - sp
    # while(remaining_distance > 0):
    #     instant += 1
    #     pm.insert(0, remaining_distance + ps[-1])
    #     remaining_distance = remaining_distance - max_vel * t
    print("ps:",ps,"pm:",pm,"pe:",pe)
    print(instant)
    return ps + pe


def braking(dec : float, ep : float, cur_vel: float, t : float):
    distance_covered_end = 0
    instant = 0
    pe = []
    while(cur_vel > 0 and dec < 0):
        u = cur_vel
        cur_vel = cur_vel + dec * t
        instant += 1
        distance_covered_end += u*t + 1/2 *dec * t**2
        pe.append(distance_covered_end)
    if (not pe) :
        return (None, None)
    print(cur_vel)
    dis = ep - pe[-1]
    pe = [i + dis for i in pe]
    return (pe, distance_covered_end)


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
