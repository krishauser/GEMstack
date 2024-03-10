from typing import List
from ..component import Component
from ...state import AllState, VehicleState, EntityRelation, EntityRelationEnum, Path, Trajectory, Route, \
    ObjectFrameEnum
from ...utils import serialization,settings
from ...mathutils.transforms import vector_dist, vector_madd


def longitudinal_plan(path: Path, acceleration: float, deceleration: float, max_speed: float,
                      current_speed: float) -> Trajectory:
    """Generates a longitudinal trajectory for a path with a
    trapezoidal velocity profile.

    1. accelerates from current speed toward max speed
    2. travel along max speed
    3. if at any point you can't brake before hitting the end of the path,
        decelerate with accel = -deceleration until velocity goes to 0.
    """
    path_normalized = path.arc_length_parameterize()

    position = settings.get('planning.start_position')
    current_v = current_speed
    end_position = path_normalized.points[-1][0]
    # print(f"time:{path_normalized.times}")
    # print(f"path points:{path_normalized.points}")
    time = settings.get('planning.start_time')
    time_step = settings.get('planning.time_step')

    times = []
    points = []
    stop_position = end_position - (current_v ** 2) / (2.0 * deceleration)
    while position < end_position:

        times.append(time)
        points.append(path_normalized.eval(position))

        if position < stop_position and current_v < max_speed - (0.5 * (time_step ** 2) * deceleration):
            current_v = min(current_v + acceleration * time_step, max_speed)
            position += current_v * time_step
        elif position >= stop_position or current_v > max_speed - (0.5 * (time_step ** 2) * acceleration):
            current_v = max(current_v - deceleration * time_step, 0)
            position += current_v * time_step
        else:
            current_v = max_speed
            position += current_v * time_step
        if current_v == 0.0:
            break
        time += time_step


    t = Trajectory(path.frame, points, times)
    return t


def longitudinal_brake(path: Path, deceleration: float, current_speed: float) -> Trajectory:
    """Generates a longitudinal trajectory for braking along a path."""
    path_normalized = path.arc_length_parameterize()
    ##TODO: actually do something to points and times
    position = 0.0
    current_v = current_speed
    current_p = path.points[-1][0]
    t = 0.0
    time_step = 0.05
    times = []
    points = []

    end_position = path_normalized.points[-1][0]
    while position < end_position:
        times.append(t)
        points.append(path_normalized.eval(position))
        current_speed = max(current_speed - deceleration * time_step, 0)
        if current_speed == 0:
            break
        position += current_speed * time_step
        t += time_step

    while current_v > 0:

        current_v = max(current_v - deceleration * time_step, 0)
        if current_v == 0:
            break
        position += current_v * time_step
        t += time_step

        times.append(t)

        points.append(path_normalized.eval(position))


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

        if self.route_progress is None:
            self.route_progress = 0.0
        closest_dist, closest_parameter = state.route.closest_point_local((curr_x, curr_y), [self.route_progress - 5.0,
                                                                                             self.route_progress + 5.0])
        route_with_lookahead = route.trim(closest_parameter, closest_parameter + 10.0)
        self.route_progress = closest_parameter

        should_brake = False
        for r in state.relations:
            if r.type == EntityRelationEnum.YIELDING and r.obj1 == '':
                # yielding to something, brake
                should_brake = True
        should_accelerate = (not should_brake and curr_v < self.desired_speed )
        print(f"speed desired :{self.desired_speed}")
        # print(f"should brake: {should_brake}")
        # print(f"cur_vel:{curr_v}")
        print(f"should_Acc:{should_accelerate}")
        if should_accelerate:
            traj = longitudinal_plan(route_with_lookahead, self.acceleration, self.deceleration, self.desired_speed,
                                     curr_v)
        elif should_brake:
            traj = longitudinal_brake(route_with_lookahead, self.deceleration, curr_v)
        else:
            traj = longitudinal_plan(route_with_lookahead, 0.0, self.deceleration, self.desired_speed, curr_v)

        return traj