from typing import List, Tuple
from ..component import Component
from ...state import AllState, VehicleState, EntityRelation, EntityRelationEnum, Path, Trajectory, Route, AgentState

import time
import numpy as np
import math
import numpy as np

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
    vehicle_back = curr_x
    vehicle_left, vehicle_right = curr_y + VEHICLE_WIDTH / 2, curr_y - VEHICLE_WIDTH / 2
    pedestrian_front, pedestrian_back = obj_x + PEDESTRIAN_LENGTH / 2, obj_x - PEDESTRIAN_LENGTH / 2
    pedestrian_left, pedestrian_right = obj_y + PEDESTRIAN_WIDTH / 2, obj_y - PEDESTRIAN_WIDTH / 2

    # Check if the object is in front of the vehicle
    if vehicle_front > pedestrian_back:
        if vehicle_back > pedestrian_front:
            # The object is behind the vehicle
            print("Object is behind the vehicle")
            return False, 0.0
        if vehicle_right - VEHICLE_BUFFER_Y > pedestrian_left or vehicle_left + VEHICLE_BUFFER_Y < pedestrian_right:
            # The object is to the side of the vehicle
            print("Object is to the side of the vehicle")
            return False, 0.0
        # The object overlaps with the vehicle
        return True, max_deceleration

    if vehicle_right - VEHICLE_BUFFER_Y > pedestrian_left and obj_v_y <= 0:
        # The object is to the right of the vehicle and moving away
        print("Object is to the right of the vehicle and moving away")
        return False, 0.0

    if vehicle_left + VEHICLE_BUFFER_Y < pedestrian_right and obj_v_y >= 0:
        # The object is to the left of the vehicle and moving away
        print("Object is to the left of the vehicle and moving away")
        return False, 0.0

    if vehicle_front + VEHICLE_BUFFER_X >= pedestrian_back:
        # The object is in front of the vehicle and within the buffer
        print("Object is in front of the vehicle and within the buffer")
        return True, max_deceleration

    # Calculate the deceleration needed to avoid a collision
    print("Object is in front of the vehicle and outside the buffer")
    distance = pedestrian_back - vehicle_front - VEHICLE_BUFFER_X

    relative_v = curr_v - obj_v_x
    if relative_v <= 0:
        return False, 0.0

    print(relative_v, distance)

    if obj_v_y > 0 and ((obj_y - curr_y) / relative_v) < ((vehicle_right - VEHICLE_BUFFER_Y - YIELD_BUFFER - pedestrian_left) / abs(obj_v_y)):
        # The object is to the right of the vehicle and moving towards it, but the vehicle will pass before the object reaches the vehicle
        print("The object is to the right of the vehicle and moving towards it, but the vehicle will pass before the object reaches the vehicle")
        return False, 0.0
    if obj_v_y < 0 and ((obj_y - curr_y) / relative_v) < ((pedestrian_right - vehicle_left - VEHICLE_BUFFER_Y - YIELD_BUFFER) / abs(obj_v_y)):
        # The object is to the left of the vehicle and moving towards it, but the vehicle will pass before the object reaches the vehicle
        print("The object is to the left of the vehicle and moving towards it, but the vehicle will pass before the object reaches the vehicle")
        return False, 0.0

    if obj_v_y != 0:
        if obj_v_y < 0:
            # The object is moving toward the right side of the vehicle
            distance_to_pass = obj_y - (vehicle_right - VEHICLE_BUFFER_Y - YIELD_BUFFER) + PEDESTRIAN_WIDTH / 2
        else:
            # The object is moving toward the left side of the vehicle
            distance_to_pass = (vehicle_left + VEHICLE_BUFFER_Y + YIELD_BUFFER) - obj_y + PEDESTRIAN_WIDTH / 2

        time_to_pass = distance_to_pass / abs(obj_v_y)

        distance_to_move = pedestrian_back - vehicle_front - VEHICLE_BUFFER_X + time_to_pass * obj_v_y


        # if curr_v**2/2*distance_to_pass >= 1.5:
        #     return True, curr_v**2/2*distance_to_pass
        time_to_max_v = (V_MAX - curr_v)/0.5

        if time_to_max_v > time_to_pass:
            if curr_v*time_to_pass + 0.5*ACCEL_MAX*time_to_pass**2 > distance_to_move:
                return False, 0.0
        else:
            if (V_MAX**2 - curr_v**2)/(2*ACCEL_MAX) + (time_to_pass - time_to_max_v) * V_MAX >= distance_to_move:
                return False, 0.0

        return True, [distance_to_move, time_to_pass]

    else:
        deceleration = relative_v ** 2 / (2 * distance)
        if deceleration > max_deceleration:
            return True, max_deceleration
        if deceleration < min_deceleration:
            return False, 0.0

        return True, deceleration


def longitudinal_plan(path : Path, acceleration : float, deceleration : float, max_speed : float, current_speed : float,
                      method : str) -> Trajectory:
    """Generates a longitudinal trajectory for a path with a
    trapezoidal velocity profile.

    1. accelerates from current speed toward max speed
    2. travel along max speed
    3. if at any point you can't brake before hitting the end of the path,
       decelerate with accel = -deceleration until velocity goes to 0.
    """

    if method == "milestone":
        return longitudinal_plan_milestone(path, acceleration, deceleration, max_speed, current_speed)
    elif method == "dt":
        return longitudinal_plan_dt(path, acceleration, deceleration, max_speed, current_speed)
    elif method == "dx":
        return longitudinal_plan_dx(path, acceleration, deceleration, max_speed, current_speed)
    else:
        raise NotImplementedError("Invalid method, only milestone, dt, adn dx are implemented.")


def longitudinal_plan_milestone(path : Path, acceleration : float, deceleration : float, max_speed : float, current_speed : float) -> Trajectory:
    """Generates a longitudinal trajectory for a path with a
    trapezoidal velocity profile.

    1. accelerates from current speed toward max speed
    2. travel along max speed
    3. if at any point you can't brake before hitting the end of the path,
       decelerate with accel = -deceleration until velocity goes to 0.
    """
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


def solve_for_v_peak(v0: float, acceleration: float, deceleration: float, total_length: float) -> float:

    if acceleration <= 0 or deceleration <= 0:
        raise ValueError("Acceleration and deceleration cant be negative")

    #Formuala: (v_peak^2 - v0^2)/(2*a) + v_peak^2/(2*d) = total_length
    numerator = deceleration * v0**2 + 2 * acceleration * deceleration * total_length
    denominator = acceleration + deceleration
    v_peak_sq = numerator / denominator

    if v_peak_sq < 0:
        return 0.0

    return math.sqrt(v_peak_sq)

def compute_dynamic_dt(acceleration, speed, k=0.005, a_min=0.5):
    position_step = k * max(speed, 1.0)  # Ensures position step is speed-dependent
    return np.sqrt(2 * position_step / max(acceleration, a_min))

def longitudinal_plan_dt(path, acceleration: float, deceleration: float, max_speed: float, current_speed: float):


    # 1 parametrizatiom.
    path_norm = path.arc_length_parameterize(speed=1.0)
    total_length = path.length()

    # -------------------
    # If the path is too short, just return the path for preventing sudden halt of simulation
    if total_length < 0.05:
        points = [p for p in path_norm.points]
        times = [t for t in path_norm.times]
        return Trajectory(path.frame, points, times)
    # -------------------

    # 2. Compute distances for d_accel,d_decel
    if max_speed > current_speed:
        d_accel = (max_speed**2 - current_speed**2) / (2 * acceleration)
    else:
        d_accel = 0.0  # Already at or above max_speed

    d_decel = (max_speed**2) / (2 * deceleration)

    # 3. trapezoidal or triangle?
    if d_accel + d_decel <= total_length:
        t_accel = (max_speed - current_speed) / acceleration if max_speed > current_speed else 0.0
        t_decel = max_speed / deceleration
        d_cruise = total_length - d_accel - d_decel
        t_cruise = d_cruise / max_speed if max_speed != 0 else 0.0
        t_final = t_accel + t_cruise + t_decel
        profile_type = "trapezoidal"
    else:
        # Triangular profile: not enough distance to reach max_speed so we will calculate peak speed.
        peak_speed = solve_for_v_peak(current_speed, acceleration, deceleration, total_length)
        # choose the min just in case
        peak_speed = min(peak_speed, max_speed)
        t_accel = (peak_speed - current_speed) / acceleration if peak_speed > current_speed else 0.0
        t_decel = peak_speed / deceleration
        t_final = t_accel + t_decel
        profile_type = "triangular"

    t = 0
    times = []
    s_vals = []
    num_time_steps = 0
    while t < t_final:
        times.append(t)
        if profile_type == "trapezoidal":
            if t < t_accel:
                # Acceleration phase.
                s = current_speed * t + 0.5 * acceleration * t**2
            elif t < t_accel + t_cruise:
                # Cruise phase.
                s = d_accel + max_speed * (t - t_accel)
            else:
                # Deceleration phase.
                t_decel_phase = t - (t_accel + t_cruise)
                s = total_length - 0.5 * deceleration * (t_decel - t_decel_phase)**2
        else:  # Triangular profile.
            if t < t_accel:
                # Acceleration phase.
                s = current_speed * t + 0.5 * acceleration * t**2
            else:
                t_decel_phase = t - t_accel
                s_accel = current_speed * t_accel + 0.5 * acceleration * t_accel**2
                s = s_accel + peak_speed * t_decel_phase - 0.5 * deceleration * t_decel_phase**2

        s_vals.append(min(s, total_length))
        if s >= total_length:
            break

        dt = compute_dynamic_dt(acceleration if t < t_accel else deceleration,current_speed)
        t = t + dt

        num_time_steps +=1

    # Compute trajectory points
    points = [path_norm.eval(s) for s in s_vals]
    print("Number of time steps is --------------------", num_time_steps)

    # return Trajectory(path_norm.frame, points, times)


    # # Plot: update a single window
    # import matplotlib.pyplot as plt
    # plt.figure("Distance vs Time")
    # plt.clf()  # Clear the current figure
    # plt.plot(times, s_vals)
    # plt.xlabel("Time (s)")
    # plt.ylabel("Distance (m)")
    # plt.title("Distance vs Time")
    # plt.draw()
    # plt.pause(0.001)



    # 4. Create a time grid.
    # dt = 0.1  # adjust based on computation
    # times = np.arange(0, t_final + dt, dt)
    # num_time_steps = 0

    # # 5. Compute the distance s(t) for each time step.
    # s_vals = []
    # for t in times:
    #     if profile_type == "trapezoidal":
    #         if t < t_accel:
    #             # Acceleration phase.
    #             s = current_speed * t + 0.5 * acceleration * t**2
    #         elif t < t_accel + t_cruise:
    #             # Cruise phase.
    #             s = d_accel + max_speed * (t - t_accel)
    #         else:
    #             # Deceleration phase.
    #             t_decel_phase = t - (t_accel + t_cruise)
    #             # Compute the remaining distance using the deceleration equation.
    #             s = total_length - 0.5 * deceleration * (t_decel - t_decel_phase)**2
    #     else:  # Triangular profile.
    #         if t < t_accel:
    #             # Acceleration phase.
    #             s = current_speed * t + 0.5 * acceleration * t**2
    #         else:
    #             t_decel_phase = t - t_accel
    #             s_accel = current_speed * t_accel + 0.5 * acceleration * t_accel**2
    #             s = s_accel + peak_speed * t_decel_phase - 0.5 * deceleration * t_decel_phase**2
    #     num_time_steps +=1

    #     # should not exceed total path length
    #     s_vals.append(min(s, total_length))
    # print("NUmber of time steps -----------",num_time_steps)
    # print("T FInal ----------------------------", t_final)
    # points = [path_norm.eval(s) for s in s_vals]



    trajectory = Trajectory(path_norm.frame, points, list(times))
    return trajectory

def longitudinal_plan_dx(path : Path, acceleration : float, deceleration : float, max_speed : float, current_speed : float) -> Trajectory:
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
    # Adjust these two numbers to choose between computation speed or smoothness
    rq = 0.1  # Smaller, smoother
    multi = 5 # Larger, smoother
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
    acc = 0

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
        print("new points: ", new_points)
        print("current index: ", cur_index)
        print("current speed: ", current_speed)

        # Information we will need:
            # Calculate how much time it would take to stop
            # Calculate how much distance it would take to stop
        min_delta_t_stop = current_speed/deceleration
        min_delta_x_stop = current_speed*min_delta_t_stop - 0.5*deceleration*min_delta_t_stop**2
        assert min_delta_x_stop >= 0


        # Check if we are done

        # If we cannot stop before or stop exactly at the final position requested
        if cur_point[0] + min_delta_x_stop >= points[-1][0] - 0.0001:
            acc = deceleration
            flag = 1
            print("In case one")
            # put on the breaks
            # Calculate the next point in a special manner because of too-little time to stop
            if cur_index >= len(points)-1:
                # the next point in this instance would be when we stop
                print(1)
                if min_delta_x_stop < rq * acc:
                    next_point = (cur_point[0] + min_delta_x_stop, 0)
                else:
                    next_point = (cur_point[0] + (min_delta_x_stop / (acc * multi)), 0)
                    flag = 0
            else:
                print(2)
                next_point = points[cur_index+1]
                if next_point[0] - cur_point[0] > rq * acc:
                    tmp = cur_point[0] + (next_point[0] - cur_point[0]) / (acc * multi)
                    flag = 0
                    next_point = [tmp, next_point[1]]

            # keep breaking until the next milestone in path
            print("continuing to next point")
            delta_t_to_next_x = compute_time_to_x(cur_point[0], next_point[0], current_speed, -deceleration)
            cur_time += delta_t_to_next_x
            cur_point = next_point
            current_speed -= deceleration*delta_t_to_next_x
            if flag:
                cur_index += 1

        # This is the case where we are accelerating to max speed
        # because the first if-statement covers for when we decelerating,
        # the only time current_speed < max_speed is when we are accelerating
        elif current_speed < max_speed:
            print("In case two")
            print(current_speed)
            acc = acceleration
            flag = 1
            # next point
            next_point = points[cur_index+1]
            if next_point[0] - cur_point[0] > rq * acc:
                tmp = cur_point[0] + (next_point[0] - cur_point[0]) / (acc * multi)
                flag = 0
                next_point = [tmp, next_point[1]]
            # accelerate to max speed

            # calculate the time it would take to reach max speed
            delta_t_to_max_speed = (max_speed - current_speed)/acceleration
            # calculate the distance it would take to reach max speed
            delta_x_to_max_speed = current_speed*delta_t_to_max_speed + 0.5*acceleration*delta_t_to_max_speed**2

            # if we would reach max speed after the next point,
            # just move to the next point and update the current speed and time
            if cur_point[0] + delta_x_to_max_speed >= next_point[0]:
                print("go to next point")
                # accelerate to max speed
                delta_t_to_next_x = compute_time_to_x(cur_point[0], next_point[0], current_speed, acceleration)
                cur_time += delta_t_to_next_x
                cur_point = [next_point[0], 0]
                current_speed += delta_t_to_next_x*acceleration
                if flag:
                    cur_index += 1

            # this is the case where we would reach max speed before the next point
            # we need to create a new point where we would reach max speed
            else:
                print("adding new point")
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
            print("In case three")

            # add point to start decelerating
            if next_point[0] + min_delta_x_stop >= points[-1][0]:
                print("Adding new point to start decelerating")
                cur_time += (points[-1][0] - min_delta_x_stop - cur_point[0])/current_speed
                cur_point = [points[-1][0] - min_delta_x_stop,0]
                current_speed = max_speed
            else:
                # Continue on to next point
                print("Continuing on to next point")
                cur_time += (next_point[0] - cur_point[0])/current_speed
                cur_point = next_point
                cur_index += 1

        # This is an edge case and should only be reach
        # if the initial speed is greater than the max speed
        elif current_speed > max_speed:
            # We need to hit the breaks
            acc = deceleration
            flag = 1
            # next point
            next_point = points[cur_index+1]
            if next_point[0] - cur_point[0] > rq * acc:
                tmp = cur_point[0] + (next_point[0] - cur_point[0]) / (acc * multi)
                flag = 0
                next_point = [tmp, next_point[1]]
            print("In case four")
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

    trajectory = Trajectory(path.frame,points,times)
    return trajectory

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
                    traj = longitudinal_plan(route_with_lookahead, self.acceleration, deceleration, desired_speed, curr_v, "milestone")
                    return traj
                else:
                    if detected and deceleration > 0:
                        yield_deceleration = max(deceleration, yield_deceleration)
                        should_yield = True
                
                print("should yield: ", should_yield)

        if should_yield:
            traj = longitudinal_brake(route_to_end, yield_deceleration, curr_v)
        else:
            traj = longitudinal_plan(route_to_end, self.acceleration, self.deceleration, self.desired_speed, curr_v)
        return traj 
