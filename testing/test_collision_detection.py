import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import math
import time

class Simulation:
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


if __name__ == "__main__":
    # Vehicle parameters. x, y, theta (angle in radians)
    x1, y1, t1 = 7.6, 5.0, 0
    # Pedestrian parameters. x, y, theta (angle in radians)
    x2, y2, t2 = 15.0, 5.8, 0
    # Velocity vectors: [vx, vy]
    v1 = [0.6, 0]     # Vehicle speed vector
    v2 = [0, 0.5]     # Pedestrian speed vector
    # Total simulation time
    total_time = 10.0

    # Basic deceleration for the vehicle
    # Output is decided based on this deceleration if the vehicle is about to hit
    basic_deceleration = 2.0

    # Max deceleration for the vehicle
    # Return deceleration bigger than basic_deceleration when the vehicle will hit
    max_deceleration = 8.0

    # Create and run the simulation.
    start_time = time.time()
    # Simulate with the above parameters: Whether to hit without decelerating
    sim = Simulation(x1, y1, t1, x2, y2, t2, v1, v2, total_time, basic_deceleration, max_deceleration)

    # relation, decel = sim.run(is_displayed=True)
    relation, decel = sim.run(is_displayed=False)

    print(f"Relation: {relation}")
    print(f"Deceleration: {decel:.2f} m/s^2")
    # print(f"Simulation took {time.time() - start_time:.3f} seconds.")