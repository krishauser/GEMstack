from ...state import AllState,VehicleState,AgentState,AgentEnum,EntityRelation,EntityRelationEnum,ObjectFrameEnum
from ..component import Component
from typing import List,Dict,Union,Tuple
from scipy.optimize import minimize
import numpy as np
import math
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from ...utils import settings

################################################################################
########## Collisiong Detection ################################################
################################################################################

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

        # print("Collision distance:", collision_distance)

        return collision_distance



def detect_collision(curr_x: float, curr_y: float, curr_v: float, obj: AgentState, min_deceleration: float, max_deceleration: float, acceleration: float, max_speed: float) -> Tuple[bool, float]:
    """Detects if a collision will occur with the given object and return deceleration to avoid it."""
    
    # Get the object's position and velocity
    obj_x = obj.pose.x
    # obj_x = 0.0
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



def detect_collision_analytical(r_pedestrain_x: float, r_pedestrain_y: float, p_vehicle_left_y_after_t: float, p_vehicle_right_y_after_t: float, lateral_buffer: float) -> Union[bool, str]:
    """Detects if a collision will occur with the given object and return deceleration to avoid it. Analytical"""
    if r_pedestrain_x < 0 and abs(r_pedestrain_y) > lateral_buffer:
        return False
    elif r_pedestrain_x < 0:
        return 'max'
    if r_pedestrain_y >= p_vehicle_left_y_after_t and r_pedestrain_y <= p_vehicle_right_y_after_t:
        return True
    
    return False


def get_minimum_deceleration_for_collision_avoidance(curr_x: float, curr_y: float, curr_v: float, obj: AgentState, min_deceleration: float, max_deceleration: float) -> Tuple[bool, float]:
    """Detects if a collision will occur with the given object and return deceleration to avoid it. Via Optimization"""

    # Get the object's position and velocity
    obj_x = obj.pose.x
    obj_y = obj.pose.y
    obj_v_x = obj.velocity[0]
    obj_v_y = obj.velocity[1]


    vehicle_length = 3
    vehicle_width = 2

    vehicle_buffer_x = 3.0
    vehicle_buffer_y = 1.0

    obj_x = obj_x - curr_x
    obj_y = obj_y - curr_y

    curr_x = curr_x - curr_x
    curr_y = curr_y - curr_y

    vehicle_front = curr_x + vehicle_length + vehicle_buffer_x
    vehicle_back = curr_x
    vehicle_left = curr_y - vehicle_width / 2
    vehicle_right = curr_y + vehicle_width / 2

    r_vehicle_front = vehicle_front - vehicle_front
    r_vehicle_back = vehicle_back - vehicle_front
    r_vehicle_left = vehicle_left - vehicle_buffer_y
    r_vehicle_right = vehicle_right + vehicle_buffer_y
    r_vehicle_v_x = curr_v
    r_vehicle_v_y = 0

    r_pedestrain_x = obj_x - vehicle_front
    r_pedestrain_y = -obj_y
    r_pedestrain_v_x = obj_v_x
    r_pedestrain_v_y = -obj_v_y

    r_velocity_x_from_vehicle = r_vehicle_v_x - r_pedestrain_v_x
    r_velocity_y_from_vehicle = r_vehicle_v_y - r_pedestrain_v_y

    t_to_r_pedestrain_x = (r_pedestrain_x - r_vehicle_front) / r_velocity_x_from_vehicle
    
    p_vehicle_left_y_after_t = r_vehicle_left + r_velocity_y_from_vehicle * t_to_r_pedestrain_x
    p_vehicle_right_y_after_t = r_vehicle_right + r_velocity_y_from_vehicle * t_to_r_pedestrain_x

    collision_flag = detect_collision_analytical(r_pedestrain_x, r_pedestrain_y, p_vehicle_left_y_after_t, p_vehicle_right_y_after_t, vehicle_buffer_y)
    if collision_flag == False:
        print("No collision", curr_x, curr_y, r_pedestrain_x, r_pedestrain_y, r_vehicle_left, r_vehicle_right, p_vehicle_left_y_after_t, p_vehicle_right_y_after_t)
        return 0.0, r_pedestrain_x
    elif collision_flag == 'max':
        return max_deceleration, r_pedestrain_x
    
    print("Collision", curr_x, curr_y, r_pedestrain_x, r_pedestrain_y, r_vehicle_left, r_vehicle_right, p_vehicle_left_y_after_t, p_vehicle_right_y_after_t)
    yaw = None
    minimum_deceleration = None
    if yaw is None:
        if abs(r_velocity_y_from_vehicle) > 0.1:
            if r_velocity_y_from_vehicle > 0.1:
                # Vehicle Left would be used to yield
                r_pedestrain_y_temp = r_pedestrain_y + abs(r_vehicle_left)
            elif r_velocity_y_from_vehicle < -0.1:
                # Vehicle Right would be used to yield
                r_pedestrain_y_temp = r_pedestrain_y - abs(r_vehicle_right)
            
            softest_accleration = 2 * r_velocity_y_from_vehicle * (r_velocity_y_from_vehicle * r_pedestrain_x - r_velocity_x_from_vehicle * r_pedestrain_y_temp) / r_pedestrain_y_temp**2
            peak_y = -(r_velocity_x_from_vehicle * r_velocity_y_from_vehicle) / softest_accleration
            # if the peak is within the position of the pedestrian, 
            # then it indicates the path had already collided with the pestrain, 
            # and so the softest acceleration should be the one the peak of the path is the same as the pedestrain's x position
            # and the vehicle should be stopped exactly before the pedestrain's x position
            if abs(peak_y) > abs(r_pedestrain_y_temp):
                minimum_deceleration = abs(softest_accleration)
            # else: the vehicle should be stopped exactly before the pedestrain's x position the same case as the pedestrain barely move laterally
        if minimum_deceleration is None:
            minimum_deceleration = r_velocity_x_from_vehicle**2 / (2 * r_pedestrain_x)
    else:
        pass

    print("calculatedminimum_deceleration: ", minimum_deceleration)

    if minimum_deceleration < min_deceleration:
        return 0.0, r_pedestrain_x
    else:
        return max(min(minimum_deceleration, max_deceleration), min_deceleration), r_pedestrain_x
    

################################################################################
########## Pedestrian Yielder ##################################################
################################################################################


class PedestrianYielder(Component):
    """Yields for all pedestrians in the scene.
    
    Result is stored in the relations graph.
    """
    def __init__(self, mode : str = 'real', params : dict = {}):
        # Planner mode
        self.mode = mode
        self.planner = params["planner"]
        self.acceleration = params["acceleration"]
        self.desired_speed = params["desired_speed"]

        # Update current.yaml settings with the given parameters in yaml
        settings.set("planning.longitudinal_plan.mode", self.mode)
        settings.set("planning.longitudinal_plan.planner", self.planner)
        settings.set("planning.longitudinal_plan.acceleration", self.acceleration)
        settings.set("planning.longitudinal_plan.desired_speed", self.desired_speed)

        self.min_deceleration   = settings.get("planning.longitudinal_plan.min_deceleration")
        self.max_deceleration   = settings.get("planning.longitudinal_plan.max_deceleration")
        self.deceleration       = settings.get("planning.longitudinal_plan.deceleration")
        self.yield_deceleration = settings.get("planning.longitudinal_plan.yield_deceleration")

    def rate(self):
        return None
    def state_inputs(self):
        return ['all']
    def state_outputs(self):
        return ['relations']
    def update(self, state : AllState) -> List[EntityRelation]:
        res = []
        vehicle = state.vehicle
        agents = state.agents

        # Position in vehicle frame (Start (0,0) to (15,0))
        curr_x = vehicle.pose.x
        curr_y = vehicle.pose.y
        curr_v = vehicle.v

       # Yielding speed [..., 1.0, 0.8, ..., 0.2]
        yield_speed        = [v for v in np.arange(self.desired_speed, 0.1, -0.25)]
 
        for n,a in agents.items():
            if a.type == AgentEnum.PEDESTRIAN:

                ##### Henry's Code #####
                # output_decel, output_dist = get_minimum_deceleration_for_collision_avoidance(curr_x, curr_y, curr_v, a, self.min_deceleration, self.max_deceleration)
                # output_speed = None

                ##### Yudai's Code #####
                x1, y1 = curr_x, curr_y
                v1 = [curr_v, 0]     # Vehicle speed vector
                x2, y2 = a.pose.x, a.pose.y
                v2 = [a.velocity[0], a.velocity[1]]     # Pedestrian speed vector
                """
                class ObjectFrameEnum(Enum):
                    START = 0                  #position / yaw in m / radians relative to starting pose of vehicle 
                    CURRENT = 1                #position / yaw in m / radians relative to current pose of vehicle
                    GLOBAL = 2                 #position in longitude / latitude, yaw=heading in radians with respect to true north (used in GNSS)
                    ABSOLUTE_CARTESIAN = 3     #position / yaw  in m / radians relative to a global cartesian reference frame (used in simulation)
                """
                # If the pedestrian's frame is ABSOLUTE, convert the pedestrian's position to START.
                if a.pose.frame == ObjectFrameEnum.ABSOLUTE_CARTESIAN:
                    x1 = curr_x + state.start_vehicle_pose.x
                    y1 = curr_y + state.start_vehicle_pose.y

                # If the pedestrian's frame is CURRENT, convert the pedestrian's position to START.
                if a.pose.frame == ObjectFrameEnum.CURRENT:
                    x2 = a.pose.x + curr_x
                    y2 = a.pose.y + curr_y

                # Create and run the simulation.
                print(f"Vehicle: ({x1:.1f}, {y1:.1f}, ({v1[0]:.1f}, {v1[1]:.1f}))")
                print(f"Pedestrian: ({x2:.1f}, {y2:.1f}, ({v2[0]:.1f}, {v2[1]:.1f}))")

                # Simulate if a collision will occur when the vehicle accelerate to desired speed.
                sim = CollisionDetector(x1, y1, 0, x2, y2, 0, v1, v2, 10.0, self.desired_speed, self.acceleration)
                collision_distance = sim.run()
                
                # No collision detected with acceleration to desired speed.
                if collision_distance < 0:
                    print("No collision detected.")
                    output_decel = self.deceleration
                    output_speed = self.desired_speed
                    output_dist  = collision_distance

                # Collision detected with acceleration to desired speed.
                # => Check if the vehicle can yield to the pedestrian with deceleration.
                else:

                    ###############################################
                    # # UNCOMMENT NOT TO YIELD: JUST STOP FOR PART1
                    # # Decide the deceleration based on the collision distance.
                    # # To stop perfectly, assume the vehicle is running at the desired speed. 
                    # brake_deceleration = max(self.deceleration, self.desired_speed**2 / (2 * (collision_distance)))
                    # if brake_deceleration > self.max_deceleration:
                    #     brake_deceleration = self.max_deceleration
                    # if brake_deceleration > self.deceleration:
                    #     print("The vehicle is Stopping.")
                    #     output_decel = brake_deceleration
                    #     output_dist  = collision_distance
                    #     output_speed = 0.0
                    #     res.append(EntityRelation(type=EntityRelationEnum.YIELDING,obj1='',obj2=n,
                    #         yield_decel=round(output_decel, 2),
                    #         yield_dist=round(output_dist, 2),
                    #         yield_speed=round(output_speed, 2)))       
                    # continue
                    ###############################################

                    print("Collision detected. Try to find yielding speed.")

                    collision_distance_after_yield = -1

                    # Simulate with different yield speeds.
                    # Try to maximize the yield speed to avoid collision.
                    for v in yield_speed:
                        # Simulate if the vehicle can yield to the pedestrian with acceleration to yielding speed.
                        if v > v1[0]:
                            sim.set_params(v, self.acceleration)
                        # Simulate if the vehicle can yield to the pedestrian with deceleration to yielding speed.
                        else:
                            sim.set_params(v, self.yield_deceleration * -1.0)
                        collision_distance_after_yield = sim.run()
                        if collision_distance_after_yield < 0:
                            print(f"Yielding at speed: {v}")
                            output_decel = self.yield_deceleration
                            output_dist  = collision_distance
                            output_speed = v
                            break
                    
                    # Collision detected with any yielding speed.
                    # => Brake to avoid collision.
                    if collision_distance_after_yield >= 0:
                        print("The vehicle is Stopping.")
                        # Decide the deceleration based on the collision distance.
                        brake_deceleration = max(self.deceleration, v1[0]**2 / (2 * (collision_distance)))
                        if brake_deceleration > self.max_deceleration:
                            brake_deceleration = self.max_deceleration
                        output_decel = brake_deceleration
                        output_dist  = collision_distance
                        output_speed = 0.0
                        should_brake = True

                """                
                yield_decel : float # Deceleration at which obj1 yields to obj2
                yield_dist  : float # Distance at which obj1 yields to obj2
                yield_speed : float # Speed at which obj1 yields to obj2
                """
                res.append(EntityRelation(type=EntityRelationEnum.YIELDING, obj1='', obj2=n,
                                          yield_decel=round(output_decel, 2),
                                          yield_dist=round(output_dist, 2),
                                          yield_speed=round(output_speed, 2)))
                
                # res.append(EntityRelation(type=EntityRelationEnum.YIELDING,obj1='',obj2=n))
        
        return res
