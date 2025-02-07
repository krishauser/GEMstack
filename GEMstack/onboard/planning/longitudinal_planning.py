from typing import List
from ..component import Component
from ...state import AllState, VehicleState, EntityRelation, EntityRelationEnum, Path, Trajectory, Route, ObjectFrameEnum
from ...utils import serialization
from ...mathutils.transforms import vector_madd

import time
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import math

class CollosionDetector:
    """
    Simulation class to update positions of two rectangles (vehicle and pedestrian)
    with velocities v1 and v2, performing collision detection at each time step.
    All functions remain within the class, and variables defined in __init__ remain unchanged;
    local copies are used during simulation.
    """
    def __init__(self, x1, y1, t1, x2, y2, t2, v1, v2, total_time=10.0, basic_deceleration=2.0, max_deceleration=8.0):

        self.vehicle_x = x1
        self.vehicle_y = y1
        self.pedestrian_x = x2
        self.pedestrian_y = y2

        # Vehicle parameters with buffer adjustments
        self.vehicle_size_x = 3.2
        self.vehicle_size_y = 1.7
        self.vehicle_buffer_x = 3.0
        self.vehicle_buffer_y = 1.0

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
        self.basic_deceleration = basic_deceleration
        self.max_deceleration = max_deceleration

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
        output_deceleration = 0.0
        relation = "None"
        # None: No relation 
        # Yielding: Vehicle is yielding to pedestrian
        # Stopping: Vehicle is stopping for pedestrian

        steps = int(self.total_time / self.dt) + 1

        # Create local variables for positions; these will be updated 
        # without modifying the __init__ attributes.
        current_x1 = self.x1
        current_y1 = self.y1
        current_x2 = self.x2
        current_y2 = self.y2

        if is_displayed:
            plt.ion()  # Turn on interactive mode
            fig, ax = plt.subplots(figsize=(10,5))
            ax.set_xlim(-5, 25)
            ax.set_ylim(-5, 5)
            ax.grid(True, linestyle='--', alpha=0.5)
            ax.set_aspect('equal')

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
                ax.set_xlim(self.vehicle_x - 5, self.vehicle_x + 20)
                ax.set_ylim(-5, 5)
                ax.grid(True, linestyle='--', alpha=0.5)
                self.plot_rectangles(rect1, rect2, collision, ax)

                # Draw an additional rectangle (vehicle body) at the vehicle's center.
                rect_vehiclebody = patches.Rectangle(
                    (current_x1 - 3.1, current_y1 - 0.85),
                    self.w1 - 3.0,
                    self.h1 - 1.0,
                    edgecolor='green',
                    fill=False,
                    linewidth=2,
                    linestyle='--'
                )
                ax.add_patch(rect_vehiclebody)

                ax.text(0, 5.5, f"t = {t_sim:.1f}s", fontsize=12)
                plt.draw()

                # Pause briefly to simulate real-time updating.
                plt.pause(self.dt * 0.05)

            # Stop simulation if collision is detected.
            if collision:
                # Check if the vehicle will hit the pedestrian or can stop before hitting.
                # Dmin = v^2 / (2 * a) => a = -v^2 / (2 * D)
                # Minimum distance required to stop before hitting with basic_deceleration
                minimum_distance         = self.v1[0]**2 / (2 * self.basic_deceleration)
                current_vehicle_x        = current_x1 - (self.vehicle_size_x + self.vehicle_buffer_x) * 0.5
                current_vehicle_y        = current_y1

                print("Collision detected. Stopping simulation.")
                print(f"Collision coordinates: ({current_vehicle_x:.1f}, {current_vehicle_y:.1f})")
                print(f"Vehicle speed: {self.v1[0]:.1f}")
                print(f"Minimum distance required to avoid collision: {minimum_distance:.1f}")

                if minimum_distance > current_vehicle_x - self.vehicle_x:
                    print("Vehicle will hit the pedestrian!!!")
                    relation = "Stopping"
                    # Deceleration to stop at the current position > basic_deceleration
                    output_deceleration = min(self.max_deceleration, self.v1[0]**2 / (2 * (current_vehicle_x - self.vehicle_x)))
                else:
                    print("Vehicle can yield. Speed down with:")
                    # Deceleration to stop at the current position < basic_deceleration
                    output_deceleration = self.v1[0]**2 / (2 * (current_vehicle_x - self.vehicle_x))
                    print(f"Appropriate deceleration: {output_deceleration:.2f}")
                    relation = "Yielding"
                break

            # Update local positions based on velocities.
            current_x1 += self.v1[0] * self.dt
            current_y1 += self.v1[1] * self.dt
            current_x2 += self.v2[0] * self.dt
            current_y2 += self.v2[1] * self.dt

        if is_displayed:
            plt.ioff()
            plt.show(block=True)

        return relation, output_deceleration


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

    # Starting point
    x0 = points[0][0]
    # Time to stop
    t_stop = current_speed / deceleration
    # Position to stop
    x_stop = x0 + current_speed * t_stop - 0.5 * deceleration * t_stop**2

    # GEM will decelerate to 0 before the end of the path
    if length < 10.0 and x_stop > points[-1][0]:
        new_points = []
        for t in times:
            if t <= t_stop:
                x = x0 + current_speed * t - 0.5 * deceleration * t**2
            # Keep the position after reaching 0 velocity
            else:
                x = x_stop
            new_points.append([x, 0])
        points = new_points
        print("[BRAKE] Computed points:", points)

    # GEM will accelerate to max speed before the end of the path
    else:
        new_points = []
        for t in times:
            # Accelerate to max speed
            if max_speed > current_speed:
                x = x0 + current_speed * t + 0.5 * acceleration * t**2
            # Keep the velocity after reaching max speed
            else:
                x = x0 + current_speed * t
            new_points.append([x, 0])
        points = new_points
        print("[ACCEL] Computed points:", points)

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
    for t in times:
        if t <= t_stop:
            x = x0 + current_speed * t - 0.5 * deceleration * t**2
        else:
            x = x_stop
        new_points.append([x, 0])
    points = new_points
    print("[BRAKE] Computed points:", points)

    #=============================================

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
        self.desired_speed = 2.0 # default 1.0
        self.deceleration = 2.0

        self.max_deceleration = 8.0
        self.relation = "None"

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

        #figure out where we are on the route
        if self.route_progress is None:
            self.route_progress = 0.0
        closest_dist,closest_parameter = state.route.closest_point_local((curr_x,curr_y),[self.route_progress-5.0,self.route_progress+5.0])
        self.route_progress = closest_parameter

        lookahead_distance = max(5, curr_v**2 / (2 * self.deceleration))
        route_with_lookahead = route.trim(closest_parameter,closest_parameter + lookahead_distance)
        print("Lookahead distance:", lookahead_distance)
        #extract out a 10m segment of the route
        # route_with_lookahead = route.trim(closest_parameter,closest_parameter+10.0)

        decel = self.deceleration
        prev_relation = self.relation
        should_yield = False

        #parse the relations indicated
        should_brake = False

        for r in state.relations:
            if r.type == EntityRelationEnum.YIELDING and r.obj1 == '':
                #yielding to something, brake

                #=========================
                """
                Collision detection:
                    - Compute the lookahead distance required to avoid collision using:
                        d = v^2/(2*a)
                    - For many steps along the route (using a resolution that adapts if the
                    planner runs too slowly), simulate the vehicle's future positions.
                    - If a pedestrian is detected within 3m longitudinal and 1m lateral buffer,
                    determine the distance-to-collision. Then compute the required deceleration:
                        a = -(v^2)/(2*d_collision)
                    - For distant crossing pedestrians, apply a gentle deceleration based on the
                    perception-estimated pedestrian velocity.
                """

                print("#### YIELDING PLANNING ####")

                # Vehicle parameters.
                x1, y1 = vehicle.pose.x + state.start_vehicle_pose.x, vehicle.pose.y + state.start_vehicle_pose.y
                v1 = [curr_v, 0]     # Vehicle speed vector

                for n,a in state.agents.items():

                    # Pedestrian parameters.
                    x2, y2 = a.pose.x, a.pose.y
                    v2 = [a.velocity[0], a.velocity[1]]     # Pedestrian speed vector
                    # Total simulation time
                    if curr_v > 0.1:
                        total_time = min(10, lookahead_distance / curr_v)
                    else:
                        total_time = 10
                    print(f"Total time: {total_time:.2f} seconds")

                    # Create and run the simulation.
                    print(f"Vehicle: ({x1:.1f}, {y1:.1f}, ({v1[0]:.1f}, {v1[1]:.1f}))")
                    print(f"Pedestrian: ({x2:.1f}, {y2:.1f}, ({v2[0]:.1f}, {v2[1]:.1f}))")
                    sim = CollosionDetector(x1, y1, 0, x2, y2, 0, v1, v2, total_time, self.deceleration, self.max_deceleration)

                    self.relation, decel = sim.run()
                    print(f"Relation: {self.relation}")
                    print(f"Deceleration: {decel:.2f} m/s^2")

                    # relation: None, Yielding, Stopping
                    # Stopping => None
                    if prev_relation == "Stopping" and self.relation == "Yielding":
                        self.relation = "Stopping"

                    break

                if self.relation == "Yielding":
                    should_brake = True
                    should_yield = True
                elif self.relation == "Stopping":
                    should_brake = True
                    should_yield = False

                # # UNCOMMENT TO BRAKE FOR ALL PEDESTRIANS
                # should_brake = True

                # # UNCOMMENT NOT TO BRAKE
                # should_brake = False

                #=========================

        should_accelerate = (not should_brake and curr_v < self.desired_speed)

        #choose whether to accelerate, brake, or keep at current velocity
        if should_accelerate:
            traj = longitudinal_plan(route_with_lookahead, self.acceleration, self.deceleration, self.desired_speed, curr_v)
        elif should_brake and not should_yield:
            traj = longitudinal_brake(route_with_lookahead, self.deceleration, curr_v)
        elif should_brake and should_yield:
            print("Yielding with", decel, "m/s^2")
            traj = longitudinal_brake(route_with_lookahead, decel, curr_v)
        else:
            traj = longitudinal_plan(route_with_lookahead, 0.0, self.deceleration, self.desired_speed, curr_v)

        print(f"Simulation took {time.time() - start_time:.3f} seconds.")

        return traj 
