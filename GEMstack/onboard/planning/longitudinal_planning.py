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

class CollisionDetector:
    """
    Simulation class to update positions of two rectangles (vehicle and pedestrian)
    with velocities v1 and v2, performing collision detection at each time step.
    All functions remain within the class, and variables defined in __init__ remain unchanged;
    local copies are used during simulation.
    """
    def __init__(self, x1, y1, t1, x2, y2, t2, v1, v2, total_time=10.0, desired_speed=2.0, acceleration=0.5):

        self.vehicle_x = x1
        self.vehicle_y = y1
        self.pedestrian_x = x2
        self.pedestrian_y = y2

        # Vehicle parameters with buffer adjustments
        self.vehicle_size_x = 3.2
        self.vehicle_size_y = 1.7
        self.vehicle_buffer_x = 3.0 + 1.0
        self.vehicle_buffer_y = 2.0

        # Vehicle rectangle
        self.x1 = self.vehicle_x + (self.vehicle_size_x + self.vehicle_buffer_x)*0.5 # Offset for buffer (remains constant)
        self.y1 = self.vehicle_y
        self.w1 = self.vehicle_size_x + self.vehicle_buffer_x  # Increase width with buffer
        self.h1 = self.vehicle_size_y + self.vehicle_buffer_y  # Increase height with buffer
        self.t1 = t1

        # Pedestrian rectangle
        self.x2 = x2
        self.y2 = y2
        self.w2 = 0.5
        self.h2 = 0.5
        self.t2 = t2

        # Velocities are expected as [vx, vy]
        self.v1 = v1
        self.v2 = v2

        self.dt = 0.1  # seconds
        self.total_time = total_time  # seconds

        self.desired_speed = desired_speed
        self.acceleration = acceleration

    def set_params(self, speed, acceleration):
        self.desired_speed = speed
        self.acceleration = acceleration

    def get_corners(self, x, y, w, h, theta):
        """
        Returns the 4 corners of a rotated rectangle.
        (x, y): center of rectangle
        w, h: width and height of rectangle
        theta: rotation angle in radians
        """
        cos_t = math.cos(theta)
        sin_t = math.sin(theta)
        hw, hh = w / 2.0, h / 2.0

        corners = np.array([
            [ hw * cos_t - hh * sin_t,  hw * sin_t + hh * cos_t],
            [-hw * cos_t - hh * sin_t, -hw * sin_t + hh * cos_t],
            [-hw * cos_t + hh * sin_t, -hw * sin_t - hh * cos_t],
            [ hw * cos_t + hh * sin_t,  hw * sin_t - hh * cos_t]
        ])

        corners[:, 0] += x
        corners[:, 1] += y

        return corners

    def get_axes(self, rect):
        axes = []
        for i in range(len(rect)):
            p1 = rect[i]
            p2 = rect[(i + 1) % len(rect)]
            edge = p2 - p1
            normal = np.array([-edge[1], edge[0]])
            norm = np.linalg.norm(normal)
            if norm:
                normal /= norm
            axes.append(normal)
        return axes

    def project(self, rect, axis):
        dots = np.dot(rect, axis)
        return np.min(dots), np.max(dots)

    def sat_collision(self, rect1, rect2):
        """
        Determines if two convex polygons (rectangles) collide using the
        Separating Axis Theorem (SAT).
        rect1 and rect2 are numpy arrays of shape (4,2) representing their corners.
        """
        axes1 = self.get_axes(rect1)
        axes2 = self.get_axes(rect2)

        for axis in axes1 + axes2:
            min1, max1 = self.project(rect1, axis)
            min2, max2 = self.project(rect2, axis)
            if max1 < min2 or max2 < min1:
                return False
        return True

    def plot_rectangles(self, rect1, rect2, collision, ax):
        """
        Plot two rectangles on a provided axis and indicate collision by color.
        """
        color = 'red' if collision else 'blue'
        for rect in [rect1, rect2]:
            polygon = patches.Polygon(rect, closed=True, edgecolor=color, fill=False, linewidth=2)
            ax.add_patch(polygon)
            ax.scatter(rect[:, 0], rect[:, 1], color=color, zorder=3)
        ax.set_title(f"Collision: {'Yes' if collision else 'No'}")

    def run(self, is_displayed=False):
        collision_distance = -1
        steps = int(self.total_time / self.dt) + 1

        # Create local variables for positions; these will be updated 
        # without modifying the __init__ attributes.
        current_x1 = self.x1
        current_y1 = self.y1
        current_x2 = self.x2
        current_y2 = self.y2
        current_v1 = self.v1[0]

        if is_displayed:
            plt.ion()  # Turn on interactive mode
            fig, ax = plt.subplots(figsize=(10,5))

        for i in range(steps):
            t_sim = i * self.dt

            # Compute rectangle corners using the local positional variables.
            rect1 = self.get_corners(current_x1, current_y1, self.w1, self.h1, self.t1)
            rect2 = self.get_corners(current_x2, current_y2, self.w2, self.h2, self.t2)

            # Perform collision detection.
            collision = self.sat_collision(rect1, rect2)

            if is_displayed:
                # Plot the current step.
                ax.clear()
                self.plot_rectangles(rect1, rect2, collision, ax)
                ax.set_aspect('equal')
                ax.grid(True, linestyle='--', alpha=0.5)
                ax.set_xlim(self.vehicle_x - 5, self.vehicle_x + 20)
                ax.set_ylim(self.vehicle_y -5, self.vehicle_y +5)

                # Draw an additional rectangle (vehicle body) at the vehicle's center.
                rect_vehiclebody = patches.Rectangle(
                    (current_x1 - (self.vehicle_size_x + self.vehicle_buffer_x)*0.5, current_y1 - self.vehicle_size_y * 0.5),
                    self.w1 - self.vehicle_buffer_x,
                    self.h1 - self.vehicle_buffer_y,
                    edgecolor='green',
                    fill=False,
                    linewidth=2,
                    linestyle='--'
                )
                ax.add_patch(rect_vehiclebody)

                ax.text(0.01, 0.99, f"t = {t_sim:.1f}s", fontsize=12, transform=ax.transAxes, verticalalignment='top')
                plt.draw()

                # Pause briefly to simulate real-time updating.
                plt.pause(self.dt * 0.05)

            # Stop simulation if collision is detected.
            if collision:
                current_vehicle_x  = current_x1 - (self.vehicle_size_x + self.vehicle_buffer_x) * 0.5
                current_vehicle_y  = current_y1
                collision_distance = current_vehicle_x - self.vehicle_x
                break

            # Update the vehicle's speed if it is not at the desired speed.
            next_v = current_v1 + self.acceleration * self.dt
            # Accelerating: Check if the vehicle is about to exceed the desired speed.
            if next_v > self.desired_speed and current_v1 <= self.desired_speed:
                current_v1 = self.desired_speed
            # Decelerating: Check if the vehicle is about to fall below the desired speed.
            elif next_v < self.desired_speed and current_v1 >= self.desired_speed:
                current_v1 = self.desired_speed
            else:
                current_v1 = next_v

            current_v1 = 0.0 if current_v1 < 0.0 else current_v1

            # Update local positions based on velocities.
            current_x1 += current_v1 * self.dt
            current_y1 += self.v1[1] * self.dt
            current_x2 += self.v2[0] * self.dt
            current_y2 += self.v2[1] * self.dt

        if is_displayed:
            plt.ioff()
            plt.show(block=True)

        print("Collision distance:", collision_distance)

        return collision_distance


def detect_collision(curr_x: float, curr_y: float, curr_v: float, obj: AgentState, min_deceleration: float, max_deceleration: float, acceleration: float, max_speed: float) -> Tuple[bool, float]:
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
        # The object overlaps with the vehicle's buffer
        return True, max_deceleration
    
    if vehicle_right - vehicle_buffer_y > pedestrian_left and obj_v_y <= 0:
        # The object is to the right of the vehicle and moving away
        print("Object is to the right of the vehicle and moving away")
        return False, 0.0
    
    if vehicle_left + vehicle_buffer_y < pedestrian_right and obj_v_y >= 0:
        # The object is to the left of the vehicle and moving away
        print("Object is to the left of the vehicle and moving away")
        return False, 0.0
    
    if vehicle_front + vehicle_buffer_x >= pedestrian_back and (vehicle_right - vehicle_buffer_y <= pedestrian_left and vehicle_left + vehicle_buffer_y >= pedestrian_right):
        # The object is in front of the vehicle and within the buffer
        print("Object is in front of the vehicle and within the buffer")
        return True, max_deceleration
    
    # Calculate the deceleration needed to avoid a collision
    print("Object is in front of the vehicle and outside the buffer")
    distance = pedestrian_back - vehicle_front
    distance_with_buffer = pedestrian_back - vehicle_front - vehicle_buffer_x

    relative_v = curr_v - obj_v_x
    if relative_v <= 0:
        return False, 0.0

    if obj_v_y == 0:
        # The object is in front of the vehicle blocking it
        deceleration = relative_v ** 2 / (2 * distance_with_buffer)
        if deceleration > max_deceleration:
            return True, max_deceleration
        if deceleration < min_deceleration:
            return False, 0.0

        return True, deceleration
    
    print(relative_v, distance_with_buffer)

    if obj_v_y > 0:
        # The object is to the right of the vehicle and moving towards it
        time_to_get_close = (vehicle_right - vehicle_buffer_y - yield_buffer_y - pedestrian_left) / abs(obj_v_y)
        time_to_pass = (vehicle_left + vehicle_buffer_y + yield_buffer_y - pedestrian_right) / abs(obj_v_y)
    else:
        # The object is to the left of the vehicle and moving towards it
        time_to_get_close = (pedestrian_right - vehicle_left - vehicle_buffer_y - yield_buffer_y) / abs(obj_v_y)
        time_to_pass = (pedestrian_left - vehicle_right + vehicle_buffer_y + yield_buffer_y) / abs(obj_v_y)

    time_to_accel_to_max_speed = (max_speed - curr_v) / acceleration
    distance_to_accel_to_max_speed = (max_speed + curr_v - 2 * obj_v_x) * time_to_accel_to_max_speed / 2 #area of trapezoid
    
    if distance_to_accel_to_max_speed > distance_with_buffer:
        # The object will reach the buffer before reaching max speed
        time_to_buffer_when_accel = (-relative_v + (relative_v * relative_v + 2 * distance_with_buffer * acceleration) ** 0.5) / acceleration
    else:
        # The object will reach the buffer after reaching max speed
        time_to_buffer_when_accel = time_to_accel_to_max_speed + (distance_with_buffer - distance_to_accel_to_max_speed) / (max_speed - obj_v_x)

    if distance_to_accel_to_max_speed > distance:
        # We will collide before reaching max speed
        time_to_collide_when_accel = (-relative_v + (relative_v * relative_v + 2 * distance * acceleration) ** 0.5) / acceleration
    else:
        # We will collide after reaching max speed
        time_to_collide_when_accel = time_to_accel_to_max_speed + (distance - distance_to_accel_to_max_speed) / (max_speed - obj_v_x)

    if time_to_get_close > time_to_collide_when_accel:
        # We can do normal driving and will pass the object before it gets in our way
        print("We can do normal driving and will pass the object before it gets in our way")
        return False, 0.0

    if vehicle_front + vehicle_buffer_x >= pedestrian_back:
        # We cannot move pass the pedestrian before it reaches the buffer from side
        return True, max_deceleration

    if time_to_pass < time_to_buffer_when_accel:
        # The object will pass through our front before we drive normally and reach it
        print("The object will pass through our front before we drive normally and reach it")
        return False, 0.0

    distance_to_move = distance_with_buffer + time_to_pass * obj_v_y

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

    #TODO: actually do something to points and times
    points = [p for p in path_normalized.points]
    times = [t for t in path_normalized.times]

    #=============================================

    print("=====LONGITUDINAL BRAKE=====")
    print("path length: ", path.length())
    length = path.length()

    x0 = points[0][0]
    t_stop = current_speed / deceleration
    x_stop = x0 + current_speed * t_stop - 0.5 * deceleration * t_stop**2

    new_points = []
    velocities = []

    for t in times:
        if t <= t_stop:
            x = x0 + current_speed * t - 0.5 * deceleration * t**2
        else:
            x = x_stop
        new_points.append([x, 0])
        velocities.append(current_speed - deceleration * t)
    points = new_points
    print("[BRAKE] Computed points:", points)

    #=============================================

    trajectory = Trajectory(path.frame,points,times,velocities)
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

                detected, deceleration = detect_collision(abs_x, abs_y, curr_v, obj, self.min_deceleration, self.max_deceleration, self.acceleration, self.desired_speed)
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

        should_accelerate = (not should_yield and curr_v < self.desired_speed)

        #choose whether to accelerate, brake, or keep at current velocity
        if should_accelerate:
            traj = longitudinal_plan(route_with_lookahead, self.acceleration, self.deceleration, self.desired_speed, curr_v, "milestone")
        elif should_yield:
            traj = longitudinal_brake(route_with_lookahead, yield_deceleration, curr_v)
        else:
            traj = longitudinal_plan(route_with_lookahead, 0.0, self.deceleration, self.desired_speed, curr_v, "dt")

        return traj 
    

# class YieldTrajectoryPlanner(Component):
#     """Follows the given route.  Brakes if you have to yield or
#     you are at the end of the route, otherwise accelerates to
#     the desired speed.
#     """

#     def __init__(self):
#         self.route_progress = None
#         self.t_last = None
#         self.acceleration = 0.5
#         self.desired_speed = 1.0 # default 1.0

#         # Yielding parameters
#         # Yielding speed [..., 1.0, 0.8, ..., 0.2]
#         self.yield_speed        = [v for v in np.arange(self.desired_speed, 0.1, -0.25)]
#         self.yield_deceleration = 0.5
#         self.deceleration       = 2.0
#         self.max_deceleration   = 8.0

#     def state_inputs(self):
#         return ['all']

#     def state_outputs(self) -> List[str]:
#         return ['trajectory']

#     def rate(self):
#         return 10.0

#     def update(self, state : AllState):
#         start_time = time.time()

#         vehicle = state.vehicle # type: VehicleState
#         route = state.route   # type: Route
#         t = state.t

#         if self.t_last is None:
#             self.t_last = t
#         dt = t - self.t_last
  
#         # Position in vehicle frame (Start (0,0) to (15,0))
#         curr_x = vehicle.pose.x
#         curr_y = vehicle.pose.y
#         curr_v = vehicle.v

#         #figure out where we are on the route
#         if self.route_progress is None:
#             self.route_progress = 0.0
#         closest_dist,closest_parameter = state.route.closest_point_local((curr_x,curr_y),[self.route_progress-5.0,self.route_progress+5.0])
#         self.route_progress = closest_parameter

#         lookahead_distance = max(10, curr_v**2 / (2 * self.yield_deceleration))
#         route_with_lookahead = route.trim(closest_parameter,closest_parameter + lookahead_distance)
#         print("Lookahead distance:", lookahead_distance)
#         #extract out a 10m segment of the route
#         # route_with_lookahead = route.trim(closest_parameter,closest_parameter+10.0)


#         # Default values
#         should_brake = False
#         desired_speed = self.desired_speed
#         accel = self.acceleration
#         decel = self.deceleration

#         for r in state.relations:
#             if r.type == EntityRelationEnum.YIELDING and r.obj1 == '':
#                 #yielding to something, brake

#                 #=========================
#                 """
#                 Collision detection:
#                     - Compute the lookahead distance required to avoid collision using:
#                         d = v^2/(2*a)
#                     - For many steps along the route (using a resolution that adapts if the
#                     planner runs too slowly), simulate the vehicle's future positions.
#                     - If a pedestrian is detected within 3m longitudinal and 1m lateral buffer,
#                     determine the distance-to-collision. Then compute the required deceleration:
#                         a = -(v^2)/(2*d_collision)
#                     - For distant crossing pedestrians, apply a gentle deceleration based on the
#                     perception-estimated pedestrian velocity.
#                 """

#                 print("#### YIELDING PLANNING ####")

#                 # Vehicle parameters.


#                 """
#                 TODO: Change the pose of vehicle and pedestrian according to the frames.
#                     Simulation => World frame?
#                     Vehicle => Relative frame?
#                 """


#                 x1, y1 = vehicle.pose.x + state.start_vehicle_pose.x, vehicle.pose.y + state.start_vehicle_pose.y
#                 v1 = [curr_v, 0]     # Vehicle speed vector

#                 for n,a in state.agents.items():

#                     # Pedestrian parameters.
#                     x2, y2 = a.pose.x, a.pose.y
#                     v2 = [a.velocity[0], a.velocity[1]]     # Pedestrian speed vector

#                     # Total simulation time
#                     if v1[0] > 0.1:
#                         total_time = min(10, lookahead_distance / v1[0])
#                     else:
#                         total_time = 10
#                     print(f"Total time: {total_time:.2f} seconds")

#                     # Create and run the simulation.
#                     print(f"Vehicle: ({x1:.1f}, {y1:.1f}, ({v1[0]:.1f}, {v1[1]:.1f}))")
#                     print(f"Pedestrian: ({x2:.1f}, {y2:.1f}, ({v2[0]:.1f}, {v2[1]:.1f}))")

#                     # Simulate if a collision will occur when the vehicle accelerate to desired speed.
#                     sim = CollisionDetector(x1, y1, 0, x2, y2, 0, v1, v2, total_time, desired_speed, accel)
#                     collision_distance = sim.run()
                    
#                     # No collision detected with acceleration to desired speed.
#                     if collision_distance < 0:
#                         print("No collision detected.")

#                     # Collision detected with acceleration to desired speed.
#                     # => Check if the vehicle can yield to the pedestrian with deceleration.
#                     else:

#                         ###############################################
#                         # # UNCOMMENT NOT TO YIELD: JUST STOP FOR PART1
#                         # print("The vehicle is Stopping.")
#                         # # Decide the deceleration based on the collision distance.
#                         # brake_deceleration = max(self.deceleration, v1[0]**2 / (2 * (collision_distance)))
#                         # if brake_deceleration > self.deceleration:
#                         #     if brake_deceleration > self.max_deceleration:
#                         #         brake_deceleration = self.max_deceleration
#                         #     decel = brake_deceleration if brake_deceleration > decel else decel
#                         #     should_brake = True
#                         #     break
#                         # break
#                         ###############################################

#                         print("Collision detected. Try to find yielding speed.")
#                         # Update lookahead distance to pedestrian.
#                         route_with_lookahead = route.trim(closest_parameter, closest_parameter + collision_distance)

#                         collision_distance_after_yield = -1

#                         # Simulate with different yield speeds.
#                         # Try to maximize the yield speed to avoid collision.
#                         for v in self.yield_speed:
#                             # Simulate if the vehicle can yield to the pedestrian with acceleration to yielding speed.
#                             if v > v1[0]:
#                                 sim.set_params(v, accel)
#                             # Simulate if the vehicle can yield to the pedestrian with deceleration to yielding speed.
#                             else:
#                                 sim.set_params(v, self.yield_deceleration * -1.0)
#                             collision_distance_after_yield = sim.run()
#                             if collision_distance_after_yield < 0:
#                                 print(f"Yielding at speed: {v}")
#                                 desired_speed = v if v < desired_speed else desired_speed
#                                 decel = self.yield_deceleration if self.yield_deceleration > decel else decel
#                                 break
                        
#                         # Collision detected with any yielding speed.
#                         # => Brake to avoid collision.
#                         if collision_distance_after_yield > 0:
#                             print("The vehicle is Stopping.")
#                             # Decide the deceleration based on the collision distance.
#                             brake_deceleration = max(self.deceleration, v1[0]**2 / (2 * (collision_distance)))
#                             if brake_deceleration > self.max_deceleration:
#                                 brake_deceleration = self.max_deceleration
#                             decel = brake_deceleration if brake_deceleration > decel else decel
#                             should_brake = True

#                     break
            
#                 # # UNCOMMENT TO BRAKE FOR ALL PEDESTRIANS
#                 # should_brake = True
#                 # desired_speed = 0.0
#                 # decel = self.deceleration

#                 # # UNCOMMENT NOT TO BRAKE
#                 # should_brake = False
#                 # desired_speed = self.desired_speed
#                 # decel = self.deceleration

#                 #=========================

#         print(f"Desired speed: {desired_speed:.2f} m/s")
#         print(f"Deceleration: {decel:.2f} m/s^2")

#         should_accelerate = (not should_brake and curr_v < self.desired_speed)

#         #choose whether to accelerate, brake, or keep at current velocity
#         if should_accelerate:
#             traj = longitudinal_plan(route_with_lookahead, accel, decel, desired_speed, curr_v)
#         elif should_brake:
#             traj = longitudinal_brake(route_with_lookahead, decel, curr_v)
#         else:
#             traj = longitudinal_plan(route_with_lookahead, 0.0, decel, desired_speed, curr_v)

#         print(f"Simulation took {time.time() - start_time:.3f} seconds.")

#         return traj 
