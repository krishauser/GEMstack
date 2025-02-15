from typing import List, Tuple
from ..component import Component
from ...state import AllState, VehicleState, EntityRelation, EntityRelationEnum, Path, Trajectory, Route, AgentState

import time
import numpy as np
import math

PEDESTRIAN_LENGTH = 0.5
PEDESTRIAN_WIDTH = 0.5

VEHICLE_LENGTH = 3.5
VEHICLE_WIDTH = 2

VEHICLE_BUFFER_X = 3.0
VEHICLE_BUFFER_Y = 1.5

YIELD_BUFFER = 1.0

V_MAX = 5.0
ACCEL_MAX = 0.5


def detect_collision(curr_x: float, curr_y: float, curr_v: float, obj: AgentState, min_deceleration: float, max_deceleration: float) -> Tuple[bool, Union[float, List[float]]]:
    """Detects potential collision and computes required deceleration or necessary movement parameters."""
    obj_x, obj_y = obj.pose.x, obj.pose.y
    obj_v_x, obj_v_y = obj.velocity

    vehicle_front = curr_x + VEHICLE_LENGTH
    vehicle_left, vehicle_right = curr_y + VEHICLE_WIDTH / 2, curr_y - VEHICLE_WIDTH / 2
    pedestrian_front, pedestrian_back = obj_x + PEDESTRIAN_LENGTH / 2, obj_x - PEDESTRIAN_LENGTH / 2
    pedestrian_left, pedestrian_right = obj_y + PEDESTRIAN_WIDTH / 2, obj_y - PEDESTRIAN_WIDTH / 2

    # Object is behind or out of lateral bounds
    if vehicle_front > pedestrian_back and vehicle_front - VEHICLE_BUFFER_X < pedestrian_back or \
            vehicle_right - VEHICLE_BUFFER_Y > pedestrian_left or vehicle_left + VEHICLE_BUFFER_Y < pedestrian_right:
        return False, 0.0

    # Object is ahead and within collision zone
    if vehicle_front + VEHICLE_BUFFER_X >= pedestrian_back:
        return True, max_deceleration

    # Compute relative velocity
    relative_v = curr_v - obj_v_x
    if relative_v <= 0:
        return False, 0.0

    # Handling object moving laterally
    if obj_v_y != 0:
        if obj_v_y < 0:
            distance_to_pass = obj_y - (vehicle_right - VEHICLE_BUFFER_Y - YIELD_BUFFER) + PEDESTRIAN_WIDTH / 2
        else:
            distance_to_pass = (vehicle_left + VEHICLE_BUFFER_Y + YIELD_BUFFER) - obj_y + PEDESTRIAN_WIDTH / 2

        time_to_pass = distance_to_pass / abs(obj_v_y)
        distance_to_move = pedestrian_back - vehicle_front - VEHICLE_BUFFER_X + time_to_pass * obj_v_y

        time_to_max_v = (V_MAX - curr_v) / ACCEL_MAX

        if time_to_max_v > time_to_pass:
            if curr_v * time_to_pass + 0.5 * ACCEL_MAX * time_to_pass**2 > distance_to_move:
                return False, 0.0
        else:
            if (V_MAX**2 - curr_v**2) / (2 * ACCEL_MAX) + (time_to_pass - time_to_max_v) * V_MAX >= distance_to_move:
                return False, 0.0

        if curr_v**2 / (2 * distance_to_pass) >= 1.5:
            return True, curr_v**2 / (2 * distance_to_pass)

        return True, [distance_to_move, time_to_pass]

    # Compute deceleration for avoiding collision
    distance = pedestrian_back - vehicle_front - VEHICLE_BUFFER_X
    deceleration = relative_v ** 2 / (2 * distance)

    if deceleration > max_deceleration:
        return True, max_deceleration
    if deceleration < min_deceleration:
        return False, 0.0

    return True, deceleration


def longitudinal_plan(path: Path, acceleration: float, deceleration: float, max_speed: float, current_speed: float) -> Trajectory:
    """Generates a longitudinal trajectory for a path with a trapezoidal velocity profile."""

    path_normalized = path.arc_length_parameterize()
    points = path_normalized.points
    times = path_normalized.times

    path_length = path.length()
    if path_length < 0.05:
        return Trajectory(path.frame, points, times)

    cur_point, cur_time, cur_index = points[0], times[0], 0
    new_points, new_times, velocities = [cur_point], [cur_time], [current_speed]

    while current_speed > 0 or cur_index == 0:
        min_stop_time = current_speed / deceleration
        min_stop_dist = current_speed * min_stop_time - 0.5 * deceleration * min_stop_time ** 2

        if cur_point[0] + min_stop_dist >= points[-1][0]:  # Need to decelerate
            next_point = (cur_point[0] + min_stop_dist, 0) if cur_index == len(points) - 1 else points[cur_index + 1]
            delta_t = compute_time_to_x(cur_point[0], next_point[0], current_speed, -deceleration)
            cur_time += delta_t
            cur_point = next_point
            current_speed = max(0, current_speed - deceleration * delta_t)
            cur_index += 1

        elif current_speed < max_speed:  # Accelerating phase
            next_point = points[cur_index + 1]
            delta_t_max_speed = (max_speed - current_speed) / acceleration
            delta_x_max_speed = current_speed * delta_t_max_speed + 0.5 * acceleration * delta_t_max_speed ** 2
            stop_x_from_max_speed = max_speed * (max_speed / deceleration) - 0.5 * deceleration * (max_speed / deceleration) ** 2

            if next_point[0] + stop_x_from_max_speed < points[-1][0] and cur_point[0] + delta_x_max_speed >= next_point[0]:
                delta_t = compute_time_to_x(cur_point[0], next_point[0], current_speed, acceleration)
                cur_time += delta_t
                cur_point = [next_point[0], 0]
                current_speed += delta_t * acceleration
                cur_index += 1

            elif cur_point[0] + delta_x_max_speed + stop_x_from_max_speed >= points[-1][0]:
                delta_t = compute_time_triangle(cur_point[0], points[-1][0], current_speed, 0, acceleration, deceleration)
                cur_time += delta_t
                cur_point = [cur_point[0] + current_speed * delta_t + 0.5 * acceleration * delta_t ** 2, 0]
                current_speed += delta_t * acceleration

            else:
                cur_time += delta_t_max_speed
                cur_point = [cur_point[0] + delta_x_max_speed, 0]
                current_speed = max_speed

        elif current_speed == max_speed:  # Constant speed phase
            next_point = points[cur_index + 1]
            if next_point[0] + min_stop_dist >= points[-1][0]:
                cur_time += (points[-1][0] - min_stop_dist - cur_point[0]) / current_speed
                cur_point = [points[-1][0] - min_stop_dist, 0]
            else:
                cur_time += (next_point[0] - cur_point[0]) / current_speed
                cur_point = next_point
                cur_index += 1

        elif current_speed > max_speed:  # Decelerating phase
            next_point = points[cur_index + 1]
            delta_t_max_speed = (current_speed - max_speed) / deceleration
            delta_x_max_speed = current_speed * delta_t_max_speed - 0.5 * deceleration * delta_t_max_speed ** 2

            if cur_point[0] + delta_x_max_speed >= next_point[0]:
                delta_t = compute_time_to_x(cur_point[0], next_point[0], current_speed, -deceleration)
                cur_time += delta_t
                cur_point = [next_point[0], 0]
                current_speed -= delta_t * deceleration
                cur_index += 1
            else:
                cur_time += delta_t_max_speed
                cur_point = [cur_point[0] + delta_x_max_speed, 0]
                current_speed = max_speed

        else:
            raise ValueError("Unexpected condition in longitudinal planning")

        new_points.append(cur_point)
        new_times.append(cur_time)
        velocities.append(current_speed)

    return Trajectory(path.frame, new_points, new_times, velocities)


def compute_time_to_x(x0: float, x1: float, v: float, a: float) -> float:
    """Computes the time to go from x0 to x1 with initial velocity v0 and final velocity v1
    with constant acceleration a. I am assuming that we will always have a solution by setting
    discriminant equal to zero, i'm not sure if this is an issue."""
    return min(quad_root(a/2, v, x0 - x1))


def compute_time_triangle(x0 : float, xf: float,  v0: float, vf : float, acceleration : float, deceleration : float) -> float:
    """
    Compute the time to go from current point assuming we are accelerating to the point at which
    we would need to start breaking in order to reach the final point with velocity 0."""
    roots = quad_root(0.5*acceleration + acceleration**2/deceleration - 0.5*acceleration**2/deceleration,
                      v0+2*acceleration*v0/deceleration - acceleration*v0/deceleration,
                      x0 - xf + v0**2/deceleration - 0.5*v0**2/deceleration)
    t1 = max(roots)
    assert t1 > 0
    return t1


def quad_root(a: float, b: float, c: float) -> list[float]:
    discriminant = b**2 - 4 * a * c
    if discriminant < 0:
        return [0.0, 0.0]  # No real solution

    sqrt_d = math.sqrt(discriminant)
    t1, t2 = (-b + sqrt_d) / a, (-b - sqrt_d) / a

    return [t1, t2]


def longitudinal_brake(path : Path, deceleration : float, current_speed : float) -> Trajectory:
    """Generates a longitudinal trajectory for braking along a path."""
    path_normalized = path.arc_length_parameterize()
    points = [p for p in path_normalized.points]
    times = [t for t in path_normalized.times]

    print("=====LONGITUDINAL BRAKE=====")
    print("deceleration: ", deceleration)

    look_ahead_time = 10.0
    dt = 0.2

    print(times[0])

    trajectory_length = (look_ahead_time / dt) + 1
    times = np.linspace(0, look_ahead_time, int(trajectory_length)).tolist()

    stop_time = current_speed / deceleration
    stop_distance = current_speed * stop_time / 2

    new_points = [points[0]]
    for i in range(1, len(times)):
        if times[i] < stop_time:
            x = points[0][0] + current_speed * times[i] - 0.5 * deceleration * times[i]**2
        else:
            x = points[0][0] + stop_distance
        new_points.append([x, 0])
    points = new_points

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

        self.min_deceleration = 1.0
        self.max_deceleration = 8.0

    def state_inputs(self):
        return ['all']

    def state_outputs(self) -> List[str]:
        return ['trajectory']

    def rate(self):
        return 10.0

    def update(self, state : AllState):
        start_time = time.time()

        vehicle = state.vehicle
        route = state.route
        t = state.t

        if self.t_last is None:
            self.t_last = t
  
        # Position in vehicle frame (Start (0,0) to (15,0))
        curr_x = vehicle.pose.x
        curr_y = vehicle.pose.y
        curr_v = vehicle.v

        abs_x = curr_x + state.start_vehicle_pose.x
        abs_y = curr_y + state.start_vehicle_pose.y

        # figure out where we are on the route
        if self.route_progress is None:
            self.route_progress = 0.0
        closest_dist,closest_parameter = state.route.closest_point_local((curr_x,curr_y),[self.route_progress-5.0,self.route_progress+5.0])
        self.route_progress = closest_parameter

        lookahead_distance = max(10, curr_v**2 / (2 * self.deceleration))
        route_with_lookahead = route.trim(closest_parameter,closest_parameter + lookahead_distance)

        should_yield = False
        yield_deceleration = 0.0

        for r in state.relations:
            if r.type == EntityRelationEnum.YIELDING and r.obj1 == '':
                # get the object we are yielding to
                obj = state.agents[r.obj2]

                detected, deceleration = detect_collision(abs_x, abs_y, curr_v, obj, self.min_deceleration, self.max_deceleration)
                if isinstance(deceleration, list):
                    time_collision = deceleration[1]
                    distance_collision = deceleration[0]
                    b = 3*time_collision - 2*curr_v
                    c = curr_v**2 - 3*distance_collision
                    desired_speed = (-b + (b**2 - 4*c)**0.5)/2
                    deceleration = 1.5
                    route_with_lookahead = route.trim(closest_parameter,closest_parameter + distance_collision)
                    traj = longitudinal_plan(route_with_lookahead, self.acceleration, deceleration, desired_speed, curr_v)
                    return traj
                else:
                    if detected and deceleration > 0:
                        yield_deceleration = max(deceleration, yield_deceleration)
                        should_yield = True
                
                print("should yield: ", should_yield)

        if should_yield:
            traj = longitudinal_brake(route_with_lookahead, yield_deceleration, curr_v)
        else:
            traj = longitudinal_plan(route_with_lookahead, self.acceleration, self.deceleration, self.desired_speed, curr_v)
        return traj 
