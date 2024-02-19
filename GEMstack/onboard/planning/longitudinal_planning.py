from typing import List
from ..component import Component
from ...state import AllState, VehicleState, EntityRelation, EntityRelationEnum, Path, Trajectory, Route, ObjectFrameEnum
from ...utils import serialization
from ...mathutils.transforms import vector_madd, vector_dist
import math
from ...utils import settings

#time step
DELTA_TIME = settings.get('longitudinal_planning.delta_time')


def get_updated_status(current_point, current_speed, acceleration, max_speed=math.inf):
    current_speed += acceleration * DELTA_TIME            
    
    # make sure current_speed in the range(0, max_speed)
    current_speed = min(current_speed, max_speed)
    current_speed = max(current_speed, 0)
    
    current_point += DELTA_TIME * current_speed
    return current_point, current_speed


def longitudinal_plan(path: Path, acceleration: float, deceleration: float, max_speed: float, current_speed: float) -> Trajectory:
    """Generates a longitudinal trajectory for a path with a
    trapezoidal velocity profile.

    1. accelerates from current speed toward max speed
    2. travel along max speed
    3. if at any point you can't brake before hitting the end of the path,
       decelerate with accel = -deceleration until velocity goes to 0.
    """
    path_normalized = path.arc_length_parameterize()
    # TODO: actually do something to points and times
    points = [p for p in path_normalized.points]
    times = [t for t in path_normalized.times]

    print('(longitudinal_plan) input current_speed:', current_speed)
    print(f"(longitudinal_plan) before_points: {points[:5]} ... {points[-5:]}",)
    print(f"(longitudinal_plan) before_times: {times[:5]} ... {times[-5:]}")

    end_time = times[-1]
    current_point = points[0][0]
    current_time = times[0]
    end_point = points[-1][0]

    update_points = [points[0]]
    update_times = [current_time]
    while current_point <= end_point and current_time < end_time:
        current_time += DELTA_TIME

        # compute safety_dist
        distance_left = end_point - current_point
        safety_dist = (current_speed**2)/(2*deceleration)
        
        # About to reach path end, begin to deceleration
        if distance_left <= safety_dist:
            current_accel = -deceleration
        else:
            current_accel = acceleration
        
        # update status
        current_point, current_speed = \
            get_updated_status(current_point, current_speed, current_accel, max_speed)
                
        update_points.append((current_point,0))
        update_times.append(current_time)
    
    # safe decel
    while current_speed > 0:
        current_time += DELTA_TIME
        current_point, current_speed = \
            get_updated_status(current_point, current_speed, -deceleration, max_speed)
        update_points.append((current_point,0))
        update_times.append(current_time)

    traj = Trajectory(path.frame, update_points, update_times)

    print(f"(longitudinal_plan) update_points: {update_points[:5]} ... {update_points[-5:]}",)
    print(f"(longitudinal_plan) update_times: {update_times[:5]} ... {update_times[-5:]}")

    return traj


def longitudinal_brake(path: Path, deceleration: float, current_speed: float) -> Trajectory:
    """Generates a longitudinal trajectory for braking along a path."""
    path_normalized = path.arc_length_parameterize()
    # TODO: actually do something to points and times
    points = [p for p in path_normalized.points]
    times = [t for t in path_normalized.times]

    print('(longitudinal_brake) input current_speed:', current_speed)
    print(f"(longitudinal_brake) before_points: {points[:5]} ... {points[-5:]}",)
    print(f"(longitudinal_brake) before_times: {times[:5]} ... {times[-5:]}")

    end_time = times[-1]
    current_point = points[0][0]
    current_time = times[0]
    end_point = points[-1][0]

    update_points = [points[0]]
    update_times = [current_time]
    while current_point <= end_point and current_time < end_time:
        current_time += DELTA_TIME
        
        # update status
        current_point, current_speed = \
            get_updated_status(current_point, current_speed, -deceleration)
               
        update_points.append((current_point,0))
        update_times.append(current_time)

    # safe decel
    while current_speed > 0:
        current_time += DELTA_TIME
        current_point, current_speed = \
            get_updated_status(current_point, current_speed, -deceleration)
        update_points.append((current_point,0))
        update_times.append(current_time)

    print(f"(longitudinal_brake) update_points: {update_points[:5]} ... {update_points[-5:]}",)
    print(f"(longitudinal_brake) update_times: {update_times[:5]} ... {update_times[-5:]}")

    
    return Trajectory(path.frame, update_points, update_times)


class YieldTrajectoryPlanner(Component):
    """Follows the given route.  Brakes if you have to yield or
    you are at the end of the route, otherwise accelerates to
    the desired speed.
    """

    def __init__(self):
        self.route_progress = None
        self.t_last = None
        self.acceleration = settings.get('longitudinal_planning.vehicle_control.acceleration')
        self.desired_speed = settings.get('longitudinal_planning.vehicle_control.desired_speed')
        self.deceleration = settings.get('longitudinal_planning.vehicle_control.deceleration')


    def state_inputs(self):
        return ['all']

    def state_outputs(self) -> List[str]:
        return ['trajectory']

    def rate(self):
        return settings.get('longitudinal_planning.update_rate')


    def update(self, state: AllState):
        vehicle = state.vehicle  # type: VehicleState
        route = state.route   # type: Route
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
        closest_dist, closest_parameter = state.route.closest_point_local(
            (curr_x, curr_y), [self.route_progress-5.0, self.route_progress+5.0])
        self.route_progress = closest_parameter

        # extract out a 10m segment of the route
        route_with_lookahead = route.trim(
            closest_parameter, closest_parameter+10.0)

        # parse the relations indicated
        should_brake = False
        for r in state.relations:
            if r.type == EntityRelationEnum.YIELDING and r.obj1 == '':
                # yielding to something, brake
                should_brake = True
        should_accelerate = (not should_brake and curr_v < self.desired_speed)

        # choose whether to accelerate, brake, or keep at current velocity
        if should_accelerate:
            traj = longitudinal_plan(
                route_with_lookahead, self.acceleration, self.deceleration, self.desired_speed, curr_v)
        elif should_brake:
            traj = longitudinal_brake(
                route_with_lookahead, self.deceleration, curr_v)
        else:
            traj = longitudinal_plan(
                route_with_lookahead, 0.0, self.deceleration, self.desired_speed, curr_v)

        return traj
