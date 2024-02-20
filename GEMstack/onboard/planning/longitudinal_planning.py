from typing import List
from ..component import Component
from ...state import AllState, VehicleState, EntityRelation, EntityRelationEnum, Path, Trajectory, Route, ObjectFrameEnum
from ...utils import serialization, settings
from ...mathutils.transforms import vector_madd, vector_dist, normalize_vector, vector_sub

'''
# Approach 1:
# 1. Add N equidistant points between each pair of points in the path
# 2. Compute time spent and distance travelled:
#    - under acceleration (t1, s1)
#    - at constant speed (t2, s2)
#    - under braking (t3, s3)
# 3. Compute time taken to reach each point

def add_points(points, n):
    points2 = [points[0]]
    for i in range(1, len(points)):
        for j in range(1, n+2):
            diff = tuple(map(lambda t1, t2: t1 - t2, points[i], points[i-1]))
            points2.append(tuple(map(lambda t1, t2: t1 + j * t2 / (n+1), points[i-1], diff)))
    return points2


def plan_too_fast(path, points, L, current_speed, deceleration):
    s_rem = current_speed**2 / (2 * deceleration) - L
    diff = tuple(map(lambda t1, t2: t1 - t2, points[-1], points[-2]))
    endpoint = tuple(map(lambda t1, t2: t1 + s_rem * t2 / vector_dist(points[-1], points[-2]), points[-1], diff))
    n = int(vector_dist(endpoint, points[-1]) / vector_dist(points[-1], points[-2]))
    points.extend(add_points([points[-1], endpoint], n-1)[1:])

    times = [0]
    for i in range(len(points) - 1):
        ds = vector_dist(points[i+1], points[0])
        dt = (current_speed - (current_speed**2 - 2 * deceleration * ds)**0.5) / deceleration
        times.append(dt)

    return Trajectory(path.frame, points, times)


def longitudinal_plan(path : Path, acceleration : float, deceleration : float, max_speed : float, current_speed : float) -> Trajectory:
    """Generates a longitudinal trajectory for a path with a trapezoidal velocity profile. 
    
    1. accelerates from current speed toward max speed
    2. travel along max speed
    3. if at any point you can't brake before hitting the end of the path,
       decelerate with accel = -deceleration until velocity goes to 0.
    """
    path_normalized = path.arc_length_parameterize()
    #TODO: actually do something to points and times
    points = [p for p in path_normalized.points]
    #times = [t for t in path_normalized.times]

    points = add_points(points, settings.get('planning.longitudinal_plan.n_points'))

    segment_lengths = [vector_dist(points[i], points[i+1]) for i in range(len(points) - 1)]
    L = sum(segment_lengths)

    if current_speed**2 / (2 * deceleration) > L: # not enough time to stop
        return plan_too_fast(path, points, L, current_speed, deceleration)

    if acceleration == 0:
        s1 = t1 = 0
        s3 = current_speed**2 / (2 * deceleration)
        t3 = current_speed / deceleration
        s2 = L - s3
        t2 = s2 / current_speed
    else:
        t1 = (max_speed - current_speed) / acceleration
        s1 = current_speed * t1 + 0.5 * acceleration * t1**2
        s3 = max_speed**2 / (2 * deceleration)
        t3 = max_speed / deceleration
        if s1 <= L - s3: # reach max speed and brake
            s2 = L - s1 - s3
            if s2 > 0:
                t2 = s2 / max_speed
            else:
                s2 = t2 = 0 
        else: # break before reaching max speed
            s2 = t2 = 0
            s1 = (deceleration * L - 0.5 * current_speed**2) / (acceleration + deceleration)
            s3 = (acceleration * L + 0.5 * current_speed**2) / (acceleration + deceleration)
            v_max_achieved = (2 * deceleration * s3)**0.5
            t1 = (v_max_achieved - current_speed) / acceleration
            t3 = v_max_achieved / deceleration

    times = [0]
    for i in range(len(points) - 1):
        if sum(segment_lengths[:i+1]) <= s1:
            du = current_speed + acceleration * times[-1]
            dt = ((du**2 + 2 * acceleration * segment_lengths[i])**0.5 - du) / acceleration
            times.append(times[-1] + dt)
            continue
        elif sum(segment_lengths[:i+1]) <= s1 + s2: 
            # not reachable if s2 = 0
            dt1 = t1 - times[-1]
            ds1 = s1 - sum(segment_lengths[:i])
            if dt1 < 0:
                dt1 = ds1 = 0
            ds2 = segment_lengths[i] - ds1 
            v_max_achieved = max_speed if acceleration != 0 else current_speed
            dt2 = ds2 / v_max_achieved
            times.append(times[-1] + dt1 + dt2)
            continue
        else:
            dt1 = t1 + t2 - times[-1]
            ds1 = s1 + s2 - sum(segment_lengths[:i])
            v_max_achieved = max_speed if acceleration != 0 and t2 != 0 else current_speed + acceleration * t1
            if dt1 < 0:
                dt1 = ds1 = 0
                du2 = v_max_achieved - deceleration * (times[-1] - t1 - t2)
            else: 
                du2 = v_max_achieved
            ds2 = segment_lengths[i] - ds1
            dt2 = (du2 - (du2**2 - 2 *  deceleration * ds2)**0.5) / deceleration
            times.append(times[-1] + dt1 + dt2)

    times[-1] = t1 + t2 + t3

    return Trajectory(path.frame, points, times)


def longitudinal_brake(path : Path, deceleration : float, current_speed : float) -> Trajectory:
    """Generates a longitudinal trajectory for braking along a path."""
    path_normalized = path.arc_length_parameterize()
    #TODO: actually do something to points and times
    points = [p for p in path_normalized.points]
    #times = [t for t in path_normalized.times]

    points = add_points(points, settings.get('planning.longitudinal_plan.n_points'))

    segment_lengths = [vector_dist(points[i], points[i+1]) for i in range(len(points) - 1)]
    L = sum(segment_lengths)

    if current_speed == 0: # stay still
        return Trajectory(path.frame, [points[0]] * 5, [i * 0.01 for i in range(0, 5)])

    if current_speed**2 / (2 * deceleration) > L: # not enough time to stop
        return plan_too_fast(path, points, L, current_speed, deceleration)

    times = [0]
    for i in range(len(points) - 1):
        if vector_dist(points[i+1], points[0]) <= current_speed**2 / (2 * deceleration):
            s = vector_dist(points[i+1], points[0])
            t = (current_speed - (current_speed**2 - 2 * deceleration * s)**0.5) / deceleration
            times.append(t)
            
    return Trajectory(path.frame, points[:len(times)], times)
'''

# Approach 2: Discretizing time

dt = settings.get('planning.longitudinal_plan.dt')

def get_unit_vector(points, s):
    l = [vector_dist(points[i], points[i+1]) for i in range(len(points) - 1)]
    cumulative_l = [sum(l[0:x:1]) for x in range(0, len(l) + 1)]
    i = next(x for x, val in enumerate(cumulative_l) if val > s) if s < sum(l) else len(points) - 1 
    return normalize_vector(vector_sub(points[i], points[i - 1]))


def longitudinal_plan(path : Path, acceleration : float, deceleration : float, max_speed : float, current_speed : float) -> Trajectory:
    """Generates a longitudinal trajectory for a path with a trapezoidal velocity profile. 
    
    1. accelerates from current speed toward max speed
    2. travel along max speed
    3. if at any point you can't brake before hitting the end of the path,
       decelerate with accel = -deceleration until velocity goes to 0.
    """
    path_normalized = path.arc_length_parameterize()
    #TODO: actually do something to points and times
    points = [p for p in path_normalized.points]
    #times = [t for t in path_normalized.times]

    l = [vector_dist(points[i], points[i+1]) for i in range(len(points) - 1)] # segment lengths

    points2 = [points[0]]
    times = [0]

    s = 0
    v = current_speed

    while v >= 0 or s < sum(l):
        a = acceleration if v**2 / (2 * deceleration) <= sum(l) - s else -deceleration
        v = min(v + a * dt, max_speed)
        s += v * dt

        points2.append(vector_madd(points2[-1], get_unit_vector(points, s), v * dt))
        times.append(times[-1] + dt)
        
    return Trajectory(path.frame, points2, times)


def longitudinal_brake(path : Path, deceleration : float, current_speed : float) -> Trajectory:
    """Generates a longitudinal trajectory for braking along a path."""
    path_normalized = path.arc_length_parameterize()
    #TODO: actually do something to points and times
    points = [p for p in path_normalized.points]
    #times = [t for t in path_normalized.times]

    if current_speed == 0: # stay still
        return Trajectory(path.frame, [points[0]] * 5, [i * 0.01 for i in range(0, 5)])

    points2 = [points[0]]
    times = [0]

    s = 0
    v = current_speed

    while v >= 0:
        v -= deceleration * dt
        s += v * dt

        points2.append(vector_madd(points2[-1], get_unit_vector(points, s), v * dt))
        times.append(times[-1] + dt)

    return Trajectory(path.frame, points2, times)


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
