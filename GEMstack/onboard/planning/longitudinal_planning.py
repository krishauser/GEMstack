from typing import List, Tuple
from ..component import Component
from ...state import AllState, VehicleState, EntityRelation, EntityRelationEnum, Path, Trajectory, Route, \
    ObjectFrameEnum
from ...utils import serialization, settings
from ...mathutils.transforms import vector_madd
import numpy as np

NO_OF_POINTS = settings.get('longitudinal_planning.no_of_samples')

def longitudinal_plan(path: Path, acceleration: float, deceleration: float, max_speed: float,
                      current_speed: float) -> Trajectory:
    """Generates a longitudinal trajectory for a path with a
    trapezoidal velocity profile.

    1. accelerates from current speed toward max speed
    2. travel along max speed
    3. if at any point you can't brake before hitting the end of the path,
       decelerate with accel = -deceleration until velocity goes to 0.
    """

    # s_acc = (max_speed ** 2 - current_speed ** 2) / (2 * acceleration)
    # s_dec = max_speed ** 2 / (2 * deceleration)
    # t_acc = (max_speed - current_speed) / acceleration
    # t_const = (length - s_acc - s_dec) / max_speed
    # points_acc, times_acc = accelerate(sp, acceleration, current_speed, max_speed)
    # points_const, times_const = const_speed(sp + s_acc, ep - s_dec, max_speed, t_acc)
    # points_dec, times_dec = decelerate(ep - s_dec, deceleration, max_speed, t_acc + t_const)
    # points, times = sample(points_acc + points_const + points_dec, times_acc + times_const + times_dec)
    # trajectory = Trajectory(path.frame, points, times)

    sp = path.points[0][0]
    length = 0

    points = [[sp, 0]]
    times = [0]

    expected_points_in_traj = [0]

    # vector distances for all points
    for i in range(len(path.points) - 1):
        length += path.points[i+1][0] - path.points[i][0]
        expected_points_in_traj.append(length)

    s = 0
    step_count = 1
    v = current_speed
    a = acceleration
    d = -1 * deceleration
    while s == 0 or v > 0:
        
        s_stop = -1 * v**2 / (2 * d)

        applied_a = d if s_stop  >= length - s else a

        v = v + applied_a * 1/NO_OF_POINTS
        if v >= max_speed:
            v = max_speed

        s += v/NO_OF_POINTS
        if s >= expected_points_in_traj[step_count] and step_count < len(expected_points_in_traj) - 1:
            step_count += 1

        distance_covered_before_next_step = s - expected_points_in_traj[step_count - 1]
        next_point = path.points[step_count - 1][0] + distance_covered_before_next_step

        points.append([next_point, 0])
        times.append(times[-1] + 1/NO_OF_POINTS)

    trajectory = Trajectory(path.frame,points,times)
    return trajectory

def sample(points: List, times: List):
    jump = len(points) // NO_OF_POINTS
    return ([points[i] for i in range(0, len(points), jump)], [times[i] for i in range(0, len(times), jump)])

def const_speed(sp, ep, speed, t0=0.0) -> Tuple[List, List]:
    n_points = 100
    time_stop = (ep - sp) / speed
    times = list(np.linspace(t0, time_stop + t0, n_points))
    points = [(sp + speed * (t - t0), 0) for t in times]
    return points[1:], times[1:]


def accelerate(sp, acceleration: float, current_speed: float, max_speed, t0=0) -> Tuple[List, List]:
    n_points = 100
    time_acc = (max_speed - current_speed) / acceleration
    times = list(np.linspace(t0, t0 + time_acc, n_points)) 
    points = []
    tl = t0
    d = 0
    for t in times:
        current_speed = current_speed + acceleration * (t - tl)
        d += (t - tl) * current_speed
        points.append((sp + d, 0))
        tl = t

    return points, times

def decelerate(sp, deceleration: float, current_speed: float, t0=0.0) -> Tuple[List, List]:
    if current_speed == 0:
        return [(sp, t0), (sp, t0)], [0, 0.1]

    n_points = 100
    time_stop = current_speed / deceleration
    times = list(np.linspace(t0, time_stop + t0, n_points))

    points = []
    tl = t0
    d = 0
    for t in times:
        current_speed = current_speed - deceleration * (t - tl)
        d += (t - tl) * current_speed
        points.append((sp + d, 0))
        tl = t

    return points, times

def longitudinal_brake(path: Path, deceleration: float, current_speed: float, t0=0) -> Trajectory:
    """Generates a longitudinal trajectory for braking along a path."""
    path_normalized = path.arc_length_parameterize()
    points = [p for p in path_normalized.points]
    theta = np.arctan2(points[-1][1] - points[0][1], points[-1][0] - points[0][0])
    deceleration, current_speed = deceleration * np.cos(theta), current_speed * np.cos(theta)
    sp = points[0][0]
    points, times = decelerate(sp, deceleration, current_speed)
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
        self.acceleration = settings.get('longitudinal_planning.vehicle_prop.acc')
        self.desired_speed = settings.get('longitudinal_planning.vehicle_prop.max_vel')
        self.deceleration = settings.get('longitudinal_planning.vehicle_prop.dec')

    def state_inputs(self):
        return ['all']

    def state_outputs(self) -> List[str]:
        return ['trajectory']

    def rate(self):
        return settings.get('longitudinal_planning.rate')

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