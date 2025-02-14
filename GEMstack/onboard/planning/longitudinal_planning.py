from typing import List, Tuple
from ..component import Component
from ...state import AllState, VehicleState, EntityRelation, EntityRelationEnum, Path, Trajectory, Route, ObjectFrameEnum, AgentState
from ...utils import serialization
from ...mathutils.transforms import vector_madd

import time
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import math

def detect_collision(curr_x: float, curr_y: float, curr_v: float, obj: AgentState, min_deceleration: float, max_deceleration: float) -> Tuple[bool, float]:
    """Detects if a collision will occur with the given object and return deceleration to avoid it."""
    
    # Get the object's position and velocity
    obj_x = obj.pose.x
    obj_y = obj.pose.y
    obj_v_x = obj.velocity[0]
    obj_v_y = obj.velocity[1]

    pedestrian_length = 0.5
    pedestrian_width = 0.5

    vehicle_length = 3.5
    vehicle_width = 2

    vehicle_buffer_x = 3.0
    vehicle_buffer_y = 1.5

    yield_buffer_y = 1.0

    vehicle_front = curr_x + vehicle_length
    vehicle_back = curr_x
    vehicle_left = curr_y + vehicle_width / 2
    vehicle_right = curr_y - vehicle_width / 2

    pedestrian_front = obj_x + pedestrian_length / 2
    pedestrian_back = obj_x - pedestrian_length / 2
    pedestrian_left = obj_y + pedestrian_width / 2
    pedestrian_right = obj_y - pedestrian_width / 2

    # Check if the object is in front of the vehicle
    if vehicle_front > pedestrian_back:
        if vehicle_back > pedestrian_front:
            # The object is behind the vehicle
            print("Object is behind the vehicle")
            return False, 0.0
        if vehicle_right - vehicle_buffer_y > pedestrian_left or vehicle_left + vehicle_buffer_y < pedestrian_right:
            # The object is to the side of the vehicle
            print("Object is to the side of the vehicle")
            return False, 0.0
        # The object overlaps with the vehicle
        return True, max_deceleration
    
    if vehicle_right - vehicle_buffer_y > pedestrian_left and obj_v_y <= 0:
        # The object is to the right of the vehicle and moving away
        print("Object is to the right of the vehicle and moving away")
        return False, 0.0
    
    if vehicle_left + vehicle_buffer_y < pedestrian_right and obj_v_y >= 0:
        # The object is to the left of the vehicle and moving away
        print("Object is to the left of the vehicle and moving away")
        return False, 0.0
    
    if vehicle_front + vehicle_buffer_x >= pedestrian_back:
        # The object is in front of the vehicle and within the buffer
        print("Object is in front of the vehicle and within the buffer")
        return True, max_deceleration
    
    # Calculate the deceleration needed to avoid a collision
    print("Object is in front of the vehicle and outside the buffer")
    distance = pedestrian_back - vehicle_front - vehicle_buffer_x

    relative_v = curr_v - obj_v_x
    if relative_v <= 0:
        return False, 0.0
    
    print(relative_v, distance)

    if obj_v_y > 0 and ((obj_y - curr_y) / relative_v) < ((vehicle_right - vehicle_buffer_y - yield_buffer_y - pedestrian_left) / abs(obj_v_y)):
        # The object is to the right of the vehicle and moving towards it, but the vehicle will pass before the object reaches the vehicle
        print("The object is to the right of the vehicle and moving towards it, but the vehicle will pass before the object reaches the vehicle")
        return False, 0.0
    if obj_v_y < 0 and ((obj_y - curr_y) / relative_v) < ((pedestrian_right - vehicle_left - vehicle_buffer_y - yield_buffer_y) / abs(obj_v_y)):
        # The object is to the left of the vehicle and moving towards it, but the vehicle will pass before the object reaches the vehicle
        print("The object is to the left of the vehicle and moving towards it, but the vehicle will pass before the object reaches the vehicle")
        return False, 0.0
    
    if obj_v_y != 0:
        if obj_v_y < 0:
            # The object is moving toward the right side of the vehicle
            distance_to_pass = obj_y - (vehicle_right - vehicle_buffer_y - yield_buffer_y) + pedestrian_width / 2
        elif obj_v_y > 0:
            # The object is moving toward the left side of the vehicle
            distance_to_pass = (vehicle_left + vehicle_buffer_y + yield_buffer_y) - obj_y + pedestrian_width / 2

        time_to_pass = distance_to_pass / abs(obj_v_y)

        distance_to_move = pedestrian_back - vehicle_front - vehicle_buffer_x + time_to_pass * obj_v_y

        return True, [distance_to_move, time_to_pass]

    else:
        deceleration = relative_v ** 2 / (2 * distance)
        if deceleration > max_deceleration:
            return True, max_deceleration
        if deceleration < min_deceleration:
            return False, 0.0

        return True, deceleration

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

    #=============================================

    print("-----LONGITUDINAL PLAN-----")

    look_ahead_time = 10.0
    dt = 0.2
    stop_distance_buffer = 0.2

    trajectory_length = (look_ahead_time / dt) + 1
    times = np.linspace(0, look_ahead_time, int(trajectory_length)).tolist()

    new_points = [points[0]]

    for _ in range(1, len(times)):
        distance_to_stop = current_speed **2 / (2 * deceleration)
        if new_points[-1][0] >= points[-1][0]:
            new_points.append([new_points[-1][0], 0])
        elif (distance_to_stop + stop_distance_buffer) >= (points[-1][0] - new_points[-1][0]):
            # start decelerating
            new_points.append([new_points[-1][0] + current_speed * dt - 0.5 * deceleration * dt**2, 0])
            current_speed -= deceleration * dt
        elif current_speed < max_speed:
            # start accelerating
            new_points.append([new_points[-1][0] + current_speed * dt + 0.5 * acceleration * dt**2, 0])
            current_speed += acceleration * dt
        else:
            new_points.append([new_points[-1][0] + current_speed * dt, 0])
    
    points = new_points

    # for p, t in zip(points, times):
    #     print(f"Time: {t:.2f}, Point: {p}")

    trajectory = Trajectory(path.frame,points,times)
    return trajectory


def longitudinal_brake(path : Path, deceleration : float, current_speed : float) -> Trajectory:
    """Generates a longitudinal trajectory for braking along a path."""
    path_normalized = path.arc_length_parameterize()

    #TODO: actually do something to points and times
    points = [p for p in path_normalized.points]
    times = [t for t in path_normalized.times]

    #=============================================

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

    # for p, t in zip(points, times):
    #     print(f"Time: {t:.2f}, Point: {p}")

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

        vehicle = state.vehicle # type: VehicleState
        route = state.route   # type: Route
        t = state.t

        if self.t_last is None:
            self.t_last = t
        dt = t - self.t_last
  
        # Position in vehicle frame (Start (0,0) to (15,0))
        curr_x = vehicle.pose.x
        curr_y = vehicle.pose.y
        curr_v = vehicle.v

        abs_x = curr_x + state.start_vehicle_pose.x
        abs_y = curr_y + state.start_vehicle_pose.y

        #figure out where we are on the route
        if self.route_progress is None:
            self.route_progress = 0.0
        closest_dist,closest_parameter = state.route.closest_point_local((curr_x,curr_y),[self.route_progress-5.0,self.route_progress+5.0])
        self.route_progress = closest_parameter

        lookahead_distance = max(10, curr_v**2 / (2 * self.deceleration))
        route_with_lookahead = route.trim(closest_parameter,closest_parameter + lookahead_distance)
        print("Lookahead distance:", lookahead_distance)

        route_to_end = route.trim(closest_parameter, len(route.points) - 1)

        should_yield = False
        yield_deceleration = 0.0

        print("Current Speed: ", curr_v)

        for r in state.relations:
            if r.type == EntityRelationEnum.YIELDING and r.obj1 == '':
                #get the object we are yielding to
                obj = state.agents[r.obj2]

                detected, deceleration = detect_collision(abs_x, abs_y, curr_v, obj, self.min_deceleration, self.max_deceleration)
                if isinstance(deceleration, list):
                    time_collision = deceleration[1]
                    distance_collision = deceleration[0]
                    b = 3*time_collision - 2*curr_v
                    c = curr_v**2 - 3*distance_collision
                    desired_speed = (-b + (b**2 - 4*c)**0.5)/2
                    deceleration = 1.5
                    print("@@@@@ YIELDING", desired_speed)
                    traj = longitudinal_plan(route_to_end, self.acceleration, deceleration, desired_speed, curr_v)
                    return traj
                else:
                    if detected and deceleration > 0:
                        yield_deceleration = max(deceleration, yield_deceleration)
                        should_yield = True
                
                print("should yield: ", should_yield)

        #choose whether to accelerate, brake, or keep at current velocity
        if should_yield:
            traj = longitudinal_brake(route_with_lookahead, yield_deceleration, curr_v)
        else:
            traj = longitudinal_plan(route_to_end, self.acceleration, self.deceleration, self.desired_speed, curr_v)
        return traj 
