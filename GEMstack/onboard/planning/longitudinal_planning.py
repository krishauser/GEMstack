from typing import List
from ..component import Component
from ...state import AllState, VehicleState, EntityRelation, EntityRelationEnum, Path, Trajectory, Route, ObjectFrameEnum
from ...utils import serialization, settings
from ...mathutils.transforms import vector_madd, vector_dist


DELTA_TIME = settings.get('longitudinal_planning.delta_time')
# DELTA_TIME = 0.05


def longitudinal_plan(path : Path, acceleration : float, deceleration : float, max_speed : float, current_speed : float) -> Trajectory:
    """Generates a longitudinal trajectory for a path with a
    trapezoidal velocity profile. 
    
    1. accelerates from current speed toward max speed
    2. travel along max speed
    3. if at any point you can't brake before hitting the end of the path,
       decelerate with accel = -deceleration until velocity goes to 0.
    """
    path_normalized = path.arc_length_parameterize()

    start_point = path_normalized.points[0]
    end_point = path_normalized.points[-1]
    points = [start_point]
    times = [0.0]

    current_point = points[0]
    current_time = times[0]

    # points = [p for p in path_normalized.points]
    # times = [t for t in path_normalized.times]

    brake = False

    while current_point[0] < end_point[0] and abs(end_point[0] - current_point[0]) > 0.0001:
        current_time += DELTA_TIME

        if not brake:
            stopping_distance = (current_speed ** 2) / (2*deceleration)

            distance_to_end_point = vector_dist(current_point, end_point)
            # distance_to_end_point = round(distance_to_end_point, 2)

            # print("distance_to_end_point:", distance_to_end_point)
            if distance_to_end_point <= stopping_distance:
                brake = True

            else:
                if abs(max_speed - current_speed < 0.0001):
                    # stay on max speed
                    current_speed = max_speed
                    current_point = (current_point[0] + current_speed * DELTA_TIME, 0)

                else:
                    # we need to accelerate to achieve max speed
                    current_x = current_point[0] + current_speed * DELTA_TIME + 1 / 2 * acceleration * (DELTA_TIME ** 2)
                    current_point = (current_x, 0)

                    current_speed += acceleration * DELTA_TIME

                points.append(current_point)
                times.append(current_time)

        else:
            # Case that we need to brake
            if current_speed > 0:
                current_x = current_point[0] + current_speed * DELTA_TIME - 0.5 * deceleration * (DELTA_TIME ** 2)
                current_point = (current_x, 0)
                current_speed -= deceleration * DELTA_TIME

                points.append(current_point)
                times.append(current_time)

            # Case that we finished our brake since current speed <= 0
            else:
                break

    trajectory = Trajectory(path.frame,points,times)
    return trajectory


def longitudinal_brake(path : Path, deceleration : float, current_speed: float) -> Trajectory:
    """Generates a longitudinal trajectory for braking along a path."""
    path_normalized = path.arc_length_parameterize()
    # TODO: actually do something to points and times
    times = [0]
    end_time = path_normalized.times[-1]
    points = [path_normalized.points[0]]
    current_position = points[0][0]
    remaining_distance = path_normalized.points[-1][0] - path_normalized.points[0][0]
    braking_distance = (current_speed ** 2) / (2 * deceleration)

    if current_speed == 0:
        while times[-1] <= end_time:
            times.append(times[-1] + 0.05)
            points.append((current_position, 0))
    
    while times[-1] <= 10 and current_speed > 0:
        current_speed -= deceleration * 0.05
        current_speed = max(0, current_speed)

        displacement = current_speed * 0.05
        current_position += displacement

        times.append(times[-1] + 0.05)
        points.append((currrent_position,0))
    
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
