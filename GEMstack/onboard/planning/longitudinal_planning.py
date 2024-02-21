from typing import List
from ..component import Component
from ...state import AllState, VehicleState, EntityRelation, EntityRelationEnum, Path, Trajectory, Route, \
    ObjectFrameEnum
from ...utils import serialization
from ...mathutils.transforms import vector_madd
import numpy as np


def longitudinal_plan(path: Path, acceleration: float, deceleration: float, max_speed: float,
                      current_speed: float) -> Trajectory:
    """Generates a longitudinal trajectory for a path with a
    trapezoidal velocity profile.

    1. accelerates from current speed toward max speed
    2. travel along max speed
    3. if at any point you can't brake before hitting the end of the path,
       decelerate with accel = -deceleration until velocity goes to 0.
    """
    '''
    path_normalized = path.arc_length_parameterize()
    points = [p for p in path_normalized.points]

    points = path_planning_2(sp=points[0][0], ep=points[-1][0], acc=acceleration, dec=deceleration, max_vel=max_speed,
                             t=0.1, cur_vel=0)
    points = [(p, 0) for p in points]
    frag = (path_normalized.times[-1] - path_normalized.times[0]) / len(points)
    times = []
    xt = 0
    while (xt < path_normalized.times[-1]):
        times.append(xt)
        xt += frag
    print(points, times)
    '''

    path_normalized = path.arc_length_parameterize()
    points = [p for p in path_normalized.points]
    theta = np.arctan2(points[-1][1] - points[0][1], points[-1][0] - points[0][0])
    acceleration, deceleration, max_speed, current_speed = acceleration * np.cos(theta), deceleration * np.cos(
        theta), max_speed * np.cos(theta), current_speed * np.cos(theta)

    sp = points[0][0]
    ep = points[-1][0]
    length = ep - sp
    s_stop = current_speed ** 2 / (2 * deceleration)
    if s_stop > length:
        points, times = decelerate(sp, deceleration, current_speed)
        trajectory = Trajectory(path.frame, points, times)
        return trajectory

    if acceleration == 0:
        # t_const=current_speed/deceleration
        s_const = length - s_stop
        t_const = s_const / current_speed
        points_const, times_const = const_speed(sp, sp + s_const, current_speed, 0)
        points_dec, times_dec = decelerate(sp + s_const, deceleration, current_speed, t_const)
        trajectory = Trajectory(path.frame, points_const + points_dec, times_const + times_dec)
        return trajectory

    s_th = (max_speed ** 2 - current_speed ** 2) / (2 * acceleration) + max_speed ** 2 / (2 * deceleration)

    # s_acc: float = max_speed ** 2 / (2 * acceleration)
    # s_con = length - s_dec - s_acc

    if s_th > length:
        v_max = np.sqrt(
            (length + current_speed ** 2 / (2 * acceleration))
            /
            (1 / (2 * acceleration) + 1 / (2 * deceleration))
        )
        time_acc = (v_max - current_speed) / acceleration
        points_acc, times_acc = accelerate(sp, acceleration, current_speed, v_max)
        s_dec = v_max ** 2 / (2 * deceleration)
        points_dec, times_dec = decelerate(ep - s_dec, deceleration, v_max, time_acc)

        trajectory = Trajectory(path.frame, points_acc + points_dec, times_acc + times_dec)
        return trajectory

    s_acc = (max_speed ** 2 - current_speed ** 2) / (2 * acceleration)
    s_dec = max_speed ** 2 / (2 * deceleration)
    t_acc = (max_speed - current_speed) / acceleration
    t_const = (length - s_acc - s_dec) / max_speed
    t_dec = max_speed / deceleration
    points_acc, times_acc = accelerate(sp, acceleration, current_speed, max_speed)
    points_const, times_const = const_speed(sp + s_acc, ep - s_dec, max_speed, t_acc)
    points_dec, times_dec = decelerate(ep - s_dec, deceleration, max_speed, t_acc + t_const)
    trajectory = Trajectory(path.frame, points_acc + points_const + points_dec, times_acc + times_const + times_dec)
    return trajectory
    # raise NotImplementedError("Not implemented yet")

    # points_dacc, times_dacc = decelerate(max(ep - s_dacc, sp), deceleration, current_speed)
    # points_acc, times_acc = [], []
    # points_const, times_const = [], []
    # if ep > s_dacc:
    #    s_acc = max_speed ** 2 / (2 * acceleration)
    #    if s_acc > length - s_dacc:
    #        points, times = accelerate(ep - s_dacc, acceleration, current_speed, max_speed)

    # need to accelerate and deaccelerate
    # s_acc = max_speed ** 2 / (2 * acceleration)
    # if s_acc > length - s_dacc:
    # points, times = accelerate(ep - s_dacc, accceleration, current_speed, max_speed)
    # only need to accelerate


def const_speed(sp, ep, speed, t0=0.0) -> (List, List):
    n_points = 100
    time_stop = (ep - sp) / speed
    times = list(np.linspace(t0, time_stop + t0, n_points))
    points = [(sp + speed * (t - t0), 0) for t in times]
    return points, times


def accelerate(sp, acceleration: float, current_speed: float, max_speed, t0=0) -> (List, List):
    n_points = 100
    time_acc = (max_speed - current_speed) / acceleration
    times = list(np.linspace(t0, t0 + time_acc, n_points))  # [i * time_stop / n_points for i in range(n_points)]
    points = []
    tl = t0
    d = 0
    for t in times:
        current_speed = current_speed + acceleration * (t - tl)
        d += (t - tl) * current_speed
        points.append((sp + d, 0))
        tl = t
    #points = [(sp + current_speed * (t - t0) + 0.5 * acceleration * (t - t0) ** 2, 0) for t in times]

    return points, times


def decelerate(sp, deceleration: float, current_speed: float, t0=0.0) -> (List, List):
    if current_speed == 0:
        return [(sp, t0), (sp, t0)], [0, 0.1]

    n_points = 100
    time_stop = current_speed / deceleration
    # times = [i * time_stop / n_points + t0 for i in range(n_points)]
    # divide the time into n_points, end point included using numpy
    times = list(np.linspace(t0, time_stop + t0, n_points))

    points = []
    tl = t0
    d = 0
    for t in times:
        current_speed = current_speed - deceleration * (t - tl)
        d += (t - tl) * current_speed
        points.append((sp + d, 0))
        tl = t
    #points = [(sp + current_speed * (t - t0) - 0.5 * deceleration * (t - t0) ** 2, 0) for t in times]

    return points, times


def longitudinal_brake(path: Path, deceleration: float, current_speed: float, t0=0) -> Trajectory:
    """Generates a longitudinal trajectory for braking along a path."""
    '''
    path_normalized = path.arc_length_parameterize()
    points = [p for p in path_normalized.points]
    print(points)
    ep = current_speed ** 2 / (2 * deceleration)
    time = current_speed / deceleration
    px = path_planning_2(sp=points[0][0], ep=ep, acc=-deceleration, dec=deceleration, cur_vel=current_speed,
                         max_vel=current_speed)
    if (len(px) > 0):
        points = [(p, 0) for p in px]
        frag = (time - path_normalized.times[0]) / len(points)
    else:
        points = [(0, 0) for p in points]
        frag = (path_normalized.times[-1] - path_normalized.times[0]) / (len(points) - 1)  # time between each point

    times = []  # number of time instances
    xt = 0  #
    while (xt < path_normalized.times[-1]):
        times.append(xt)
        xt += frag
    times.append(path_normalized.times[-1])
    print(points, times)
    '''
    path_normalized = path.arc_length_parameterize()
    points = [p for p in path_normalized.points]
    theta = np.arctan2(points[-1][1] - points[0][1], points[-1][0] - points[0][0])
    deceleration, current_speed = deceleration * np.cos(theta), current_speed * np.cos(theta)
    sp = points[0][0]
    points, times = decelerate(sp, deceleration, current_speed)
    trajectory = Trajectory(path.frame, points, times)
    return trajectory


def path_planning(sp: float, ep: float, dec: float, max_vel: float, t: float, acc: float = 0):
    ps = []
    pe = []
    pm = []
    cur_vel = 0
    instant = 0
    distance_covered_start = 0
    while (cur_vel < max_vel and acc > 0):
        u = cur_vel
        cur_vel = cur_vel + acc * t
        instant += t
        distance_covered_start += u * t + 1 / 2 * acc * t ** 2
        ps.append(distance_covered_start + sp)

    (pe, distance_covered_end) = braking(dec=dec, ep=ep, cur_vel=max_vel, t=0.1)

    remaining_distance = ep - distance_covered_start - distance_covered_end - sp
    while (remaining_distance > 0):
        instant += 1
        pm.insert(0, remaining_distance + ps[-1])
        remaining_distance = remaining_distance - max_vel * t
    print("ps:", ps, "pm:", pm, "pe:", pe)
    print(instant)
    return ps + pm + pe


def path_planning_2(sp: float, ep: float, max_vel: float, t: float = 0.1, cur_vel: float = 0, acc: float = 0,
                    dec: float = 0):
    '''
    sp: start position
    ep: end position
    '''
    ps = []  # pos start
    pe = []  # pos end
    pm = []  # not used
    instant = 0  # not used
    distance_covered_start = 0  # s=ut+1/2at^2
    distance_covered_end = 0  # distance to the endpoint
    end_vel = 0  # velocity at the last point

    while (distance_covered_start + distance_covered_end <= ep - sp and max_vel > 0.01):
        au = cur_vel  # acceleration
        cur_vel = min(cur_vel + acc * t, max_vel)
        instant += t
        distance_covered_start += t * au
        ps.append(distance_covered_start + sp)
        du = end_vel
        end_vel = min(end_vel + dec * t, max_vel)
        # instant += 1
        distance_covered_end += t * du
        pe.insert(0, ep - distance_covered_end)

    # (pe, distance_covered_end) = braking(dec = dec, ep = ep, cur_vel = max_vel, t = 0.1)

    # remaining_distance = ep - distance_covered_start - distance_covered_end - sp
    # while(remaining_distance > 0):
    #     instant += 1
    #     pm.insert(0, remaining_distance + ps[-1])
    #     remaining_distance = remaining_distance - max_vel * t
    ctr = -1
    for i in range(len(pe)):
        if (ps[-1] >= pe[i]):
            ctr = i
        else:
            break

    print("ps:", ps, "pm:", pm, "pe:", pe)
    print(instant)
    return ps + pe


def braking(dec: float, ep: float, cur_vel: float, t: float):
    distance_covered_end = 0
    instant = 0
    pe = []
    while (cur_vel > 0 and dec < 0):
        u = cur_vel
        cur_vel = cur_vel + dec * t
        instant += 1
        distance_covered_end += u * t + 1 / 2 * dec * t ** 2
        pe.append(distance_covered_end)
    if (not pe):
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

    def update(self, state: AllState):
        vehicle = state.vehicle  # type: VehicleState
        route = state.route  # type: Route
        t = state.t

        if self.t_last is None:
            self.t_last = t
        dt = t - self.t_last

        curr_x = vehicle.pose.x
        curr_y = vehicle.pose.y
        curr_v = vehicle.v

        # figure out where we are on the route
        if self.route_progress is None:
            self.route_progress = 0.0
        closest_dist, closest_parameter = state.route.closest_point_local((curr_x, curr_y), [self.route_progress - 5.0,
                                                                                             self.route_progress + 5.0])
        self.route_progress = closest_parameter

        # extract out a 10m segment of the route
        route_with_lookahead = route.trim(closest_parameter, closest_parameter + 10.0)

        # parse the relations indicated
        should_brake = False
        for r in state.relations:
            if r.type == EntityRelationEnum.YIELDING and r.obj1 == '':
                # yielding to something, brake
                should_brake = True
        should_accelerate = (not should_brake and curr_v < self.desired_speed)

        # choose whether to accelerate, brake, or keep at current velocity
        if should_accelerate:
            traj = longitudinal_plan(route_with_lookahead, self.acceleration, self.deceleration, self.desired_speed,
                                     curr_v)
        elif should_brake:
            traj = longitudinal_brake(route_with_lookahead, self.deceleration, curr_v)
        else:
            traj = longitudinal_plan(route_with_lookahead, 0.0, self.deceleration, self.desired_speed, curr_v)

        return traj
