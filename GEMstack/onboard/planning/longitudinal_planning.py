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


        # if curr_v**2/2*distance_to_pass >= 1.5:
        #     return True, curr_v**2/2*distance_to_pass
        time_to_max_v = (5 - curr_v)/0.5

        if time_to_max_v > time_to_pass:
            if curr_v*time_to_pass + 0.25*time_to_pass**2 > distance_to_move:
                return False, 0.0
        else:
            if (25 - curr_v**2)*4 + (time_to_pass - time_to_max_v) * 5 >= distance_to_move:
                return False, 0.0


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
    times = [t for t in path_normalized.times]
    #=============================================

    print("-----LONGITUDINAL PLAN-----")
    print("path length: ", path.length())
    length = path.length()

    # If the path is too short, just return the path for preventing sudden halt of simulation
    if length < 0.05:
        return Trajectory(path.frame, points, times)

    # This assumes that the time denomination cannot be changed

    # Starting point
    x0 = points[0][0]
    cur_point = points[0]
    cur_time = times[0]
    cur_index = 0

    new_points = []
    new_times = []
    velocities = [] # for graphing and debugging purposes

    while current_speed > 0 or cur_index == 0:
        # we want to iterate through all the points and add them
        # to the new points. However, we also want to add "critical points"
        # where we reach top speed, begin decelerating, and stop
        new_points.append(cur_point)
        new_times.append(cur_time)
        velocities.append(current_speed)
        print("=====================================")
        # print("new points: ", new_points)
        # print("current index: ", cur_index)
        # print("current speed: ", current_speed)
        # print("current position: ", cur_point)

        # Information we will need: 
            # Calculate how much time it would take to stop
            # Calculate how much distance it would take to stop
        min_delta_t_stop = current_speed/deceleration
        min_delta_x_stop = current_speed*min_delta_t_stop - 0.5*deceleration*min_delta_t_stop**2
        # print(min_delta_x_stop)
        assert min_delta_x_stop >= 0


        # Check if we are done

        # If we cannot stop before or stop exactly at the final position requested
        if cur_point[0] + min_delta_x_stop >= points[-1][0]:
            print("In case one")
            # put on the breaks

            # Calculate the next point in a special manner because of too-little time to stop 
            if cur_index == len(points)-1:
                # the next point in this instance would be when we stop
                next_point = (cur_point[0] + min_delta_x_stop, 0)
            else:
                next_point = points[cur_index+1]

            # keep breaking until the next milestone in path
            if next_point[0] <= points[-1][0]:
                # print("continuing to next point")
                delta_t_to_next_x = compute_time_to_x(cur_point[0], next_point[0], current_speed, -deceleration)
                cur_time += delta_t_to_next_x
                cur_point = next_point
                current_speed -= deceleration*delta_t_to_next_x
                cur_index += 1
            else:
                # continue to the point in which we would stop (current_velocity = 0)
                # update to the next point 
                delta_t_to_next_x = compute_time_to_x(cur_point[0], next_point[0], current_speed, -deceleration)
                cur_point = next_point
                cur_time += delta_t_to_next_x
                # current_speed would not be exactly zero error would be less than 1e-4 but perfer to just set to zero
                #current_speed -= delta_t_to_next_x*deceleration
                current_speed = 0
                assert current_speed == 0

        # This is the case where we are accelerating to max speed
        # because the first if-statement covers for when we decelerating,
        # the only time current_speed < max_speed is when we are accelerating 
        elif current_speed < max_speed:
            # print("In case two")
            # next point 
            next_point = points[cur_index+1]
            # accelerate to max speed

            # calculate the time it would take to reach max speed
            delta_t_to_max_speed = (max_speed - current_speed)/acceleration
            # calculate the distance it would take to reach max speed
            delta_x_to_max_speed = current_speed*delta_t_to_max_speed + 0.5*acceleration*delta_t_to_max_speed**2
            
            delta_t_to_stop_from_max_speed = max_speed/deceleration
            delta_x_to_stop_from_max_speed = max_speed*delta_t_to_stop_from_max_speed - 0.5*deceleration*delta_t_to_stop_from_max_speed**2
            
            delta_t_to_next_point = compute_time_to_x(cur_point[0], next_point[0], current_speed, acceleration)
            velocity_at_next_point = current_speed + delta_t_to_next_point*acceleration
            time_to_stop_from_next_point = velocity_at_next_point/deceleration
            delta_x_to_stop_from_next_point = velocity_at_next_point*time_to_stop_from_next_point - 0.5*deceleration*time_to_stop_from_next_point**2
            # if we would reach max speed after the next point, 
            # just move to the next point and update the current speed and time
            if next_point[0] + delta_x_to_stop_from_next_point < points[-1][0] and \
                cur_point[0] + delta_x_to_max_speed >= next_point[0]:
                # ("go to next point")
                # accelerate to max speed
                delta_t_to_next_x = compute_time_to_x(cur_point[0], next_point[0], current_speed, acceleration)
                cur_time += delta_t_to_next_x
                cur_point = [next_point[0], 0]
                current_speed += delta_t_to_next_x*acceleration
                cur_index += 1
                
            # This is the case where we would need to start breaking before reaching 
            # top speed and before the next point (i.e. triangle shape velocity)
            elif cur_point[0] + delta_x_to_max_speed + delta_x_to_stop_from_max_speed >= points[-1][0]:
                # print(delta_x_to_max_speed)
                # print(delta_x_to_stop_from_max_speed)
                # Add a new point at the point where we should start breaking 
                # print("Adding new point to start breaking")
                delta_t_to_next_x = compute_time_triangle(cur_point[0], points[-1][0], current_speed, 0, acceleration, deceleration)
                # print(delta_t_to_next_x)
                #delta_t_to_next_x = compute_time_to_x(cur_point[0], points[-1][0] - min_delta_x_stop, current_speed, acceleration)
                next_x = cur_point[0] + current_speed*delta_t_to_next_x + 0.5*acceleration*delta_t_to_next_x**2
                cur_time += delta_t_to_next_x
                cur_point = [next_x, 0]
                current_speed += delta_t_to_next_x*acceleration
            
            # this is the case where we would reach max speed before the next point
            # we need to create a new point where we would reach max speed 
            else:
                # print("adding new point")
                # we would need to add a new point at max speed
                cur_time += delta_t_to_max_speed
                cur_point = [cur_point[0] + delta_x_to_max_speed, 0]
                current_speed = max_speed

        # This is the case where we are at max speed
        # special functionality is that this block must 
        # add a point where we would need to start declerating to reach
        # the final point
        elif current_speed == max_speed:
            next_point = points[cur_index+1]
            # continue on with max speed
            # print("In case three")
            
            # add point to start decelerating
            if next_point[0] + min_delta_x_stop >= points[-1][0]:
                # print("Adding new point to start decelerating")
                cur_time += (points[-1][0] - min_delta_x_stop - cur_point[0])/current_speed
                cur_point = [points[-1][0] - min_delta_x_stop,0]
                current_speed = max_speed
            else:
                # Continue on to next point
                # print("Continuing on to next point")
                cur_time += (next_point[0] - cur_point[0])/current_speed
                cur_point = next_point
                cur_index += 1
        
        # This is an edge case and should only be reach 
        # if the initial speed is greater than the max speed
        elif current_speed > max_speed:
            # We need to hit the breaks 

            next_point = points[cur_index+1]
            # print("In case four")
            # slow down to max speed 
            delta_t_to_max_speed = (current_speed - max_speed)/deceleration
            delta_x_to_max_speed = current_speed*delta_t_to_max_speed - 0.5*deceleration*delta_t_to_max_speed**2

            # If we would reach the next point before slowing down to max speed
            # keep going until we reach the next point
            if cur_point[0] + delta_x_to_max_speed >= next_point[0]:
                delta_t_to_next_x = compute_time_to_x(cur_point[0], next_point[0], current_speed, -deceleration)
                cur_time += delta_t_to_next_x
                cur_point = [next_point[0], 0]
                current_speed -= delta_t_to_next_x*deceleration
                cur_index += 1
            else:
            # We would reach max speed before the next point
            # we need to add a new point at the point where we
            # would reach max speed
                cur_time += delta_t_to_max_speed
                cur_point = [cur_point[0] + delta_x_to_max_speed, 0]
                current_speed = max_speed

        else:
            # not sure what falls here
            raise ValueError("LONGITUDINAL PLAN ERROR: Not sure how we ended up here")
        
    new_points.append(cur_point)
    new_times.append(cur_time)
    velocities.append(current_speed)
        
    points = new_points
    times = new_times
    print("[PLAN] Computed points:", points)
    print("[TIME] Computed time:", times)
    print("[Velocities] Computed velocities:", velocities)

    #=============================================

    trajectory = Trajectory(path.frame,points,times,velocities)
    return trajectory

def compute_time_to_x(x0 : float, x1 : float, v : float, a : float) -> float:
    """Computes the time to go from x0 to x1 with initial velocity v0 and final velocity v1
    with constant acceleration a. I am assuming that we will always have a solution by settings
    discriminant equal to zero, i'm not sure if this is an issue."""

    """Consider changing the system to use linear operators instead of explicitly calculating because of instances here"""
    # x1 = x0 + v0*t + 0.5*a*t^2
    # x1 - x0 = v0*t + 0.5*a*t^2
    # 0.5*a*t^2 + v0*t + x0 - x1 = 0
    # t = (-v0 + sqrt(v0^2 - 4*0.5*a*(x0-x1)))/(2*0.5*a)
    # t = (-v0 + sqrt(v0^2 + 2*a*(x1-x0)))/a
    # print("x0: ", x0)
    # print("x1: ", x1)
    # print("v: ", v)
    # print("a: ", a)

    t1 = (-v + max(0,(v**2 - 2*a*(x0-x1)))**0.5)/a
    t2 = (-v - max(0,(v**2 - 2*a*(x0-x1)))**0.5)/a

    # print("t1: ", t1)
    # print("t2: ", t2)

    if math.isnan(t1): t1 = 0
    if math.isnan(t2): t2 = 0

    # print("t1: ", t1)
    # print("t2: ", t2)

    valid_times = [n for n in [t1, t2] if n > 0]
    if valid_times:
        return min(valid_times)
    else:
        return 0.0

def compute_time_triangle(x0 : float, xf: float,  v0: float, vf : float, acceleration : float, deceleration : float) -> float:
    """
    Compute the time to go from current point assuming we are accelerating to the point at which
    we would need to start breaking in order to reach the final point with velocity 0."""
    # xf = v0*t1 + .5a*t1^2 + v1t2 -0.5d*t2^2
    # 0 = v1 - d*t2 = v0 + a*t1 - d * t2
    # t1 = (d*t2 - v0)/a
    # xf = v0*(d*t2 - v0)/a + 0.5*a*(d*t2 - v0)^2/a^2 + v1*t2 - 0.5*d*t2^2
    # xf = v0*d*t2/a - v0^2/a + 0.5*a*(d*t2^2 - 2*v0*d*t2 + v0^2)/a^2 + v1*t2 - 0.5*d*t2^2
    # 0 = t2^2(0.5*a*d/a^2 - 0.5*d) + t2(v0*d/a - v0*d*a/a^2 + v1) -v0^2/a +0.5*a*v0^2/a^2 -xf
    roots = quad_root(0.5*acceleration + acceleration**2/deceleration - 0.5*acceleration**2/deceleration,
                      v0+2*acceleration*v0/deceleration - acceleration*v0/deceleration,
                      x0 - xf + v0**2/deceleration - 0.5*v0**2/deceleration)
    print(roots)
    t1 = max(roots)
    assert t1 > 0
    return t1

def quad_root(a : float, b : float, c : float) -> float:
    x1 = (-b + max(0,(b**2 - 4*a*c))**0.5)/(2*a)
    x2 = (-b - max(0,(b**2 - 4*a*c))**0.5)/(2*a)

    # print("t1: ", t1)
    # print("t2: ", t2)

    if math.isnan(x1): x1 = 0
    if math.isnan(x2): x2 = 0

    # print("t1: ", t1)
    # print("t2: ", t2)

    valid_roots = [n for n in [x1, x2] if not type(n) is complex]
    # print(f"Valid roots {valid_roots}")
    return valid_roots

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
                    print("@@@@@ INPUT", deceleration)
                    time_collision = deceleration[1]
                    distance_collision = deceleration[0]
                    b = 3*time_collision - 2*curr_v
                    c = curr_v**2 - 3*distance_collision
                    desired_speed = (-b + (b**2 - 4*c)**0.5)/2
                    deceleration = 1.5
                    print("@@@@@ YIELDING", desired_speed)
                    route_with_lookahead = route.trim(closest_parameter,closest_parameter + distance_collision)
                    traj = longitudinal_plan(route_with_lookahead, self.acceleration, deceleration, desired_speed, curr_v)
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
            # traj = longitudinal_plan(route_to_end, self.acceleration, self.deceleration, self.desired_speed, curr_v)
            traj = longitudinal_plan(route_with_lookahead, self.acceleration, self.deceleration, self.desired_speed, curr_v)
        return traj 
