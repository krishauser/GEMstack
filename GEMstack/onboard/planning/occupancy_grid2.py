from __future__ import print_function

# Python Headers
import os
import cv2 
import numpy as np

import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

import cv2
import numpy as np

image_path = os.path.join(os.path.dirname(__file__), "highbay_image.pgm")

class OccupancyGrid2:

    global image_path
    
    def __init__(self):

        # Read image in BGR format
        self.map_image = cv2.imread(image_path)

        # Create the cv_bridge object
        self.bridge  = CvBridge()
        self.map_image_pub = rospy.Publisher("/motion_image2", Image, queue_size=1) 

        # Subscribe information from sensors
        self.lat     = 0
        self.lon     = 0
        self.heading = 0

        self.lat_start_bt = 40.092722  # 40.09269  
        self.lon_start_l  = -88.236365 # -88.23628
        self.lat_scale    = 0.00062   # 0.0007    
        self.lon_scale    = 0.00136  # 0.00131   

        self.arrow        = 40 
        self.img_width    = 2107
        self.img_height   = 1313

    def image_to_gnss(self, pix_x, pix_y):
        """
        Given image coordinates (pix_x, pix_y), return (latitude, longitude).
        Inverse of your:
            pix_x = img_width * (lon - lon_start_l) / lon_scale
            pix_y = img_height - img_height*(lat - lat_start_bt) / lat_scale
        """
        # longitude:
        lon = self.lon_start_l + (pix_x / float(self.img_width)) * self.lon_scale
        lat = self.lat_start_bt + ((self.img_height - pix_y) / float(self.img_height)) * self.lat_scale

        return lat, lon

    def image_heading(self, lon_x, lat_y, heading):
        
        if(heading >=0 and heading < 90):
            angle  = np.radians(90-heading)
            lon_xd = lon_x + int(self.arrow * np.cos(angle))
            lat_yd = lat_y - int(self.arrow * np.sin(angle))

        elif(heading >= 90 and heading < 180):
            angle  = np.radians(heading-90)
            lon_xd = lon_x + int(self.arrow * np.cos(angle))
            lat_yd = lat_y + int(self.arrow * np.sin(angle))  

        elif(heading >= 180 and heading < 270):
            angle = np.radians(270-heading)
            lon_xd = lon_x - int(self.arrow * np.cos(angle))
            lat_yd = lat_y + int(self.arrow * np.sin(angle))

        else:
            angle = np.radians(heading-270)
            lon_xd = lon_x - int(self.arrow * np.cos(angle))
            lat_yd = lat_y - int(self.arrow * np.sin(angle)) 

        return lon_xd, lat_yd         


    def gnss_to_image_with_heading(self, lon, lat, heading):
        
        lon_x = int(self.img_width*(lon - self.lon_start_l)/self.lon_scale)
        lat_y = int(self.img_height-self.img_height*(lat - self.lat_start_bt)/self.lat_scale)
        lon_xd, lat_yd = self.image_heading(lon_x, lat_y, heading)

        pub_image = np.copy(self.map_image)

        if(lon_x >= 0 and lon_x <= self.img_width and lon_xd >= 0 and lon_xd <= self.img_width and 
            lat_y >= 0 and lat_y <= self.img_height and lat_yd >= 0 and lat_yd <= self.img_height):
            cv2.arrowedLine(pub_image, (lon_x, lat_y), (lon_xd, lat_yd), (0, 0, 255), 2)
            cv2.circle(pub_image, (lon_x, lat_y), 12, (0,0,255), 2)

            ## Debug check if you can convert back to GNSS
            # tmp_lat, tmp_lon = self.image_to_gnss(lon_x, lat_y)
            # print(f'Lat: {self.lat}, Lon: {self.lon}, Converted Lat: {tmp_lat}, Converted Lon: {tmp_lon}')
            ## End Debug check
        try:
            # Convert OpenCV image to ROS image and publish
            self.map_image_pub.publish(self.bridge.cv2_to_imgmsg(pub_image, "bgr8"))
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))

    def gnss_to_image(self, lon, lat):
        
        lon_x = int(self.img_width*(lon - self.lon_start_l) /self.lon_scale)
        lat_y = int(self.img_height  -  self.img_height*(lat - self.lat_start_bt)/self.lat_scale)
        print(f"GNSS hiiiii ({lat}, {lon}) → pixel ({lon_x}, {lat_y})")
        pub_image = np.copy(self.map_image)

        if(lon_x >= 0 and lon_x <= self.img_width and 
            lat_y >= 0 and lat_y <= self.img_height):
            cv2.circle(pub_image, (lon_x, lat_y), 12, (0,0,255), 2)


        try:
            # Convert OpenCV image to ROS image and publish
            self.map_image_pub.publish(self.bridge.cv2_to_imgmsg(pub_image, "bgr8"))
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))

    def gnss_to_image_coords(self, lon, lat):
        
        lon_x = int(self.img_width*(lon - self.lon_start_l) /self.lon_scale)
        lat_y = int(self.img_height  -  self.img_height*(lat - self.lat_start_bt)/self.lat_scale)
        print(f"GNSS ({lat}, {lon}) → pixel ({lon_x}, {lat_y})")
        return lon_x, lat_y







'''

import math
import random
import time
import numpy as np
import matplotlib.pyplot as plt
from .utils import normalize_angle
from .collision import fast_collision_check
import scipy.spatial

class RRTNode:
    def __init__(self, x, y, theta, v=0.0, phi=0.0, cost=0.0, parent=None):
        """
        Initialize RRT node with full kinodynamic state for car-like robot.
        
        Args:
            x, y: Position coordinates
            theta: Heading angle (radians)
            v: Velocity (pixels/step)
            phi: Steering angle (radians)
            cost: Cost to reach this node
            parent: Parent node
        """
        self.x = x
        self.y = y
        self.theta = theta
        self.v = v
        self.phi = phi
        self.cost = cost
        self.parent = parent
        self.path_from_parent = []
        
    def get_state(self):
        """Get complete state as (x, y, theta, v, phi)."""
        return (self.x, self.y, self.theta, self.v, self.phi)
    
    def get_position(self):
        """Get position as (x, y)."""
        return (self.x, self.y)
        
    def distance_to(self, other):
        """Simple Euclidean distance for nearest neighbor search."""
        return math.sqrt((self.x - other.x)**2 + (self.y - other.y)**2)

class OptimizedKinodynamicRRT:
    def __init__(self, occupancy_grid, collision_lookup, start, goal,
                max_iter=3000, step_size=2.0, goal_sample_rate=0.25,
                turning_radius=3.0, vehicle_length=4.0):
        """
        Initialize Optimized Kinodynamic RRT planner.
        
        Args:
            occupancy_grid: Binary grid (1=obstacle, 0=free)
            collision_lookup: Collision lookup table
            start: (x, y, theta) start state
            goal: (x, y, theta) goal state
            max_iter: Maximum iterations
            step_size: Step size for path sampling
            goal_sample_rate: Probability of sampling the goal
            turning_radius: Minimum turning radius
            vehicle_length: Length of the vehicle
        """
        self.grid = occupancy_grid
        print(self.grid.shape,"HIIIIIIIIIIIIIIIIIIIIIii")
        self.collision_lookup = collision_lookup
        
        # Initialize start and goal
        self.start = RRTNode(start[0], start[1], start[2])
        self.goal = RRTNode(goal[0], goal[1], goal[2])
        
        # Parameters
        self.max_iter = max_iter
        self.step_size = step_size
        self.goal_sample_rate = goal_sample_rate
        self.turning_radius = turning_radius
        self.vehicle_length = vehicle_length
        self.dt = 0.1  # Time step for simulation
        
        # Initialize RRT tree with start node
        self.nodes = [self.start]
        self.kdtree = None
        
        # Goal-related parameters
        self.goal_found = False
        self.goal_node = None
        self.position_boundary = 8.0
        self.orientation_boundary = math.pi/4  
        
        # System constraints
        self.max_steering_angle = math.pi/6  # Maximum steering angle
        self.max_velocity = 20.0  # Maximum velocity
        self.min_velocity = 0.0  # Minimum velocity (no reverse)
        
        # Grid dimensions
        self.x_min, self.y_min = 0, 0
        self.x_max, self.y_max = occupancy_grid.shape
        
        # KD-Tree related parameters
        self.kdtree_rebuild_freq = 50  # Rebuild KD-tree every X nodes
        
        # Time budget
        self.time_budget = 1.8
        
        # Direct distance for informed sampling
        self.direct_distance = math.sqrt((self.goal.x - self.start.x)**2 + 
                                        (self.goal.y - self.start.y)**2)
                                        
        self._initialize_motion_primitives()
        
        self.use_pff_controller = True
        self.pff_control_weights = {
            'velocity': 0.7,
            'steering': 0.8 
        }
    
    def _initialize_motion_primitives(self):
        """
        Pre-compute a set of motion primitives for faster expansion.
        Each primitive is a trajectory generated with different control inputs.
        """
        self.primitives = []
        
        # Generate primitives with various steering and velocity combinations
        velocities = np.linspace(self.max_velocity * 0.3, self.max_velocity, 5)
        steering_angles = np.linspace(-self.max_steering_angle, self.max_steering_angle, 7)
        
        # Initialize with standard primitives
        for v in velocities:
            for phi in steering_angles:
                # Generate primitive trajectory
                primitive = self._generate_primitive(v, phi)
                if primitive:
                    self.primitives.append(primitive)
        
        # Add special primitives for common maneuvers
        # Straight line motion
        straight = self._generate_primitive(self.max_velocity * 0.8, 0.0)
        if straight:
            self.primitives.append(straight)
        
        # Sharp turns
        sharp_left = self._generate_primitive(self.max_velocity * 0.4, self.max_steering_angle)
        sharp_right = self._generate_primitive(self.max_velocity * 0.4, -self.max_steering_angle)
        if sharp_left:
            self.primitives.append(sharp_left)
        if sharp_right:
            self.primitives.append(sharp_right)
            
        print(f"Initialized {len(self.primitives)} motion primitives")
    
    def _generate_primitive(self, velocity, steering_angle, steps=15):
        """
        Generate a single motion primitive with fixed velocity and steering angle.
        
        Args:
            velocity: Constant velocity for this primitive
            steering_angle: Constant steering angle for this primitive
            steps: Number of steps in the primitive
            
        Returns:
            List of states (x, y, theta, v, phi) representing the primitive
        """
        # Initial state (at origin with zero heading)
        x, y, theta = 0.0, 0.0, 0.0
        
        primitive = [(x, y, theta, velocity, steering_angle)]
        
        # Forward simulate with the given controls
        for _ in range(steps):
            x_next = x + velocity * math.cos(theta) * self.dt
            y_next = y + velocity * math.sin(theta) * self.dt
            theta_next = normalize_angle(theta + (velocity * math.tan(steering_angle) / self.vehicle_length) * self.dt)
            
            # Update state
            x, y, theta = x_next, y_next, theta_next
            primitive.append((x, y, theta, velocity, steering_angle))
            
        return primitive
    
    def _compute_optimal_controls(self, x, y, theta, v, phi, target_position):
        """
        PFF controller - compute optimal controls to reach target position.
        
        Args:
            x, y, theta, v, phi: Current state
            target_position: Target position (x, y)
            
        Returns:
            Tuple of (optimal_velocity, optimal_steering_angle)
        """
        dx = target_position[0] - x
        dy = target_position[1] - y
        target_angle = math.atan2(dy, dx)
        
        heading_error = normalize_angle(target_angle - theta)

        distance = math.sqrt(dx*dx + dy*dy)
        alignment_factor = math.cos(heading_error)

        alignment_factor = max(0.0, alignment_factor)

        v_optimal = min(
            self.max_velocity,
            self.max_velocity * min(1.0, distance / (5.0 * self.step_size)) * alignment_factor
        )
        
        steering_gain = 1.5
        phi_optimal = steering_gain * heading_error
        
        phi_optimal = max(-self.max_steering_angle, min(self.max_steering_angle, phi_optimal))
        
        v_optimal = v * (1 - self.pff_control_weights['velocity']) + v_optimal * self.pff_control_weights['velocity']
        phi_optimal = phi * (1 - self.pff_control_weights['steering']) + phi_optimal * self.pff_control_weights['steering']
        
        return v_optimal, phi_optimal
    
    def _informed_sampling(self):
        """
        Sample from an informed ellipsoidal region when a path is found.
        Otherwise, sample from entire space.
        """
        if not self.goal_found or random.random() < 0.2:
            return self._random_state()
            
        # Calculate ellipsoid parameters
        c_best = self.goal_node.cost
        c_min = self.direct_distance
        
        # Ellipse with foci at start and goal
        x_center = (self.start.x + self.goal.x) / 2
        y_center = (self.start.y + self.goal.y) / 2
        
        a = c_best / 2
        c = c_min / 2
        b = math.sqrt(max(0.001, a*a - c*c))
        
        # Sample from unit ball and transform
        while True:
            r = random.random()
            angle = random.uniform(0, 2*math.pi)
            x = r * math.cos(angle)
            y = r * math.sin(angle)
            
            if x*x + y*y <= 1:
                # Direction from start to goal
                theta = math.atan2(self.goal.y - self.start.y, 
                                self.goal.x - self.start.x)
                
                # Transform to ellipse
                x_rot = x * math.cos(theta) - y * math.sin(theta)
                y_rot = x * math.sin(theta) + y * math.cos(theta)
                
                x_ellipse = x_center + a * x_rot
                y_ellipse = y_center + b * y_rot
                
                # Verify bounds and collision
                if (self.x_min <= x_ellipse <= self.x_max and
                    self.y_min <= y_ellipse <= self.y_max and
                    not fast_collision_check(x_ellipse, y_ellipse, self.collision_lookup)):
                    return (x_ellipse, y_ellipse)
    
    def _random_state(self):
        """Generate a random state in the grid."""
        if random.random() < self.goal_sample_rate:
            return self.goal.get_position()
            
        while True:
            x = random.uniform(self.x_min, self.x_max)
            y = random.uniform(self.y_min, self.y_max)
            
            # Check if the sampled point is in free space
            if not fast_collision_check(x, y, self.collision_lookup):
                return (x, y)
    
    def _build_kdtree(self):
        """Build KD-tree for fast nearest neighbor search."""
        positions = np.array([(node.x, node.y) for node in self.nodes])
        self.kdtree = scipy.spatial.cKDTree(positions)
    
    def _nearest_node(self, rand_position):
        """Find the nearest node to the random position using KD-tree."""
        if self.kdtree is None or len(self.nodes) % self.kdtree_rebuild_freq == 0:
            self._build_kdtree()
            
        _, idx = self.kdtree.query([rand_position[0], rand_position[1]])
        return self.nodes[idx]
    
    def _select_best_primitive(self, from_node, target_position):
        """
        Select the best motion primitive to reach the target position.
        
        Args:
            from_node: Starting node
            target_position: Target position (x, y)
            
        Returns:
            Transformed primitive (list of states) and its end node
        """
        best_distance = float('inf')
        best_primitive = None
        best_end_node = None
        
        # Current state
        x, y, theta, v, phi = from_node.get_state()
        
        # For each primitive
        for primitive in self.primitives:
            # Transform primitive to start at from_node
            transformed_primitive = self._transform_primitive(primitive, from_node)
            
            # Get the end state
            end_state = transformed_primitive[-1]
            
            # Calculate distance to target
            dist_to_target = math.sqrt((end_state[0] - target_position[0])**2 + 
                                    (end_state[1] - target_position[1])**2)

            # Check if this primitive is better
            if dist_to_target < best_distance:
                # Check if primitive is collision-free
                collision = False
                for state in transformed_primitive:
                    if fast_collision_check(state[0], state[1], self.collision_lookup):
                        collision = True
                        break
                
                if not collision:
                    best_distance = dist_to_target
                    best_primitive = transformed_primitive
                    
                    # Create end node
                    primitive_cost = len(primitive) * self.dt
                    best_end_node = RRTNode(
                        end_state[0], end_state[1], end_state[2], 
                        end_state[3], end_state[4],
                        cost=from_node.cost + primitive_cost,
                        parent=from_node
                    )
                    best_end_node.path_from_parent = transformed_primitive
        
        return best_primitive, best_end_node
    
    def _transform_primitive(self, primitive, from_node):
        """
        Transform a primitive to start at from_node's state.
        
        Args:
            primitive: Motion primitive (list of states)
            from_node: Node to start from
            
        Returns:
            Transformed primitive
        """
        # Starting state
        x0, y0, theta0 = from_node.x, from_node.y, from_node.theta
        
        # Transform each state in the primitive
        transformed = []
        
        for state in primitive:
            # Original state in primitive's local frame
            local_x, local_y, local_theta, v, phi = state
            
            # Transform to global frame
            # Rotate
            rotated_x = local_x * math.cos(theta0) - local_y * math.sin(theta0)
            rotated_y = local_x * math.sin(theta0) + local_y * math.cos(theta0)
            
            # Translate
            global_x = rotated_x + x0
            global_y = rotated_y + y0
            
            # Add theta offset
            global_theta = normalize_angle(local_theta + theta0)
            
            # Add to transformed primitive
            transformed.append((global_x, global_y, global_theta, v, phi))
            
        return transformed
    
    def _new_state_pff(self, from_node, rand_position):
        """
        Generate a new state using the PFF controller.
        Only samples position, automatically determines optimal velocity and steering.
        
        Args:
            from_node: Node to expand from
            rand_position: Target position (x, y)
            
        Returns:
            New node or None if no valid state found
        """
        x, y, theta, v, phi = from_node.get_state()
    
        # Let PFF controller determine optimal controls
        v_optimal, phi_optimal = self._compute_optimal_controls(x, y, theta, v, phi, rand_position)
        
        # Ensure v_optimal is never zero to avoid division by zero
        min_velocity = 1e-3  # Small positive value
        if abs(v_optimal) < min_velocity:
            v_optimal = min_velocity
        
        # Now safely calculate max_steps
        max_steps = max(1, int(self.step_size / (v_optimal * self.dt)))
        steps = min(max_steps, 15)  # Cap to 15 steps
        
        # Path storage
        path = [(x, y, theta, v, phi)]
        
        # Forward simulation with reduced collision checks
        check_interval = 2  # Only check every 2 steps
        
        for i in range(steps):
            # Car-like robot kinematic model
            x_next = x + v_optimal * math.cos(theta) * self.dt
            y_next = y + v_optimal * math.sin(theta) * self.dt
            theta_next = normalize_angle(theta + (v_optimal * math.tan(phi_optimal) / self.vehicle_length) * self.dt)
            
            # Check for collision at reduced frequency
            if i % check_interval == 0 and fast_collision_check(x_next, y_next, self.collision_lookup):
                return None
            
            # Update state
            x, y, theta = x_next, y_next, theta_next
            path.append((x, y, theta, v_optimal, phi_optimal))
        
        # Create new node
        new_node = RRTNode(x, y, theta, v_optimal, phi_optimal, cost=from_node.cost + steps*self.dt, parent=from_node)
        new_node.path_from_parent = path
        
        return new_node
    
    def _new_state_motion_primitive(self, from_node, rand_position):
        """
        Generate a new state using pre-computed motion primitives.
        
        Args:
            from_node: Node to expand from
            rand_position: Target position (x, y)
            
        Returns:
            New node or None if no valid primitive found
        """
        # Select best primitive to reach target
        _, end_node = self._select_best_primitive(from_node, rand_position)
        
        return end_node
    
    def _new_state(self, from_node, rand_position):
        """
        Generate a new state by applying controls and forward simulating.
        Dispatches to PFF controller or motion primitives based on configuration.
        
        Args:
            from_node: Node to expand from
            rand_position: Target position (x, y) to expand toward
            
        Returns:
            New node or None if no valid state found
        """
        # Randomly choose between PFF controller and motion primitives
        # PFF is generally more optimal but motion primitives are faster
        if random.random() < 0.7:  # 70% chance to use PFF
            return self._new_state_pff(from_node, rand_position)
        else:
            return self._new_state_motion_primitive(from_node, rand_position)
    
    def _is_goal_reached(self, node):
        """
        Two-phase goal reaching with precise heading alignment.
        First phase: Reach goal position with relaxed heading
        Second phase: Achieve exact heading orientation
        """
        # Distance to goal
        dist = math.sqrt((node.x - self.goal.x)**2 + (node.y - self.goal.y)**2)
        
        # Angle difference
        angle_diff = abs(normalize_angle(node.theta - self.goal.theta))
        
        # Make boundaries more lenient as time passes
        elapsed = time.time() - self.start_time
        position_boundary = min(self.position_boundary * (1 + elapsed/2), 15.0)
        
        # Phase 1: Position-focused with relaxed heading constraints
        if dist <= position_boundary / 3:
            # Phase 2: When very close to goal position, require precise heading
            # Stricter angle tolerance when close to goal position
            angle_threshold = math.radians(5.0)
            return dist <= position_boundary and angle_diff <= angle_threshold
        else:
            # Relaxed heading constraint when far from goal
            angle_boundary = min(self.orientation_boundary * (1 + elapsed/2), math.pi/2)
            return dist <= position_boundary and angle_diff <= angle_boundary
    
    def _refine_final_heading(self, near_goal_node):
        """
        Refine the final heading to match goal orientation exactly.
        Uses in-place steering to adjust the heading at the final position.
        """
        # Get final position
        x, y = near_goal_node.x, near_goal_node.y
        current_theta = near_goal_node.theta
        goal_theta = self.goal.theta
        
        # Calculate heading difference
        heading_diff = normalize_angle(goal_theta - current_theta)
        
        # Avoid unnecessary adjustments for very small differences
        if abs(heading_diff) < math.radians(1.0):
            return
        
        # Create a turning-in-place maneuver
        # Use reduced velocity and appropriate steering to turn in place
        turning_velocity = 2.0  # Slow velocity for precise turning
        
        # Determine steering direction based on shortest turn
        steering_angle = math.copysign(self.max_steering_angle/2, heading_diff)
        
        # Create path for turning in place
        path = [(x, y, current_theta, turning_velocity, steering_angle)]
        theta = current_theta
        
        # Simulate turning until aligned with goal heading
        max_steps = 30  # Prevent infinite loops
        for _ in range(max_steps):
            # Apply steering but maintain position (turn in place approximation)
            new_theta = normalize_angle(theta + (turning_velocity * math.tan(steering_angle) / 
                                            self.vehicle_length) * self.dt)
            
            # Check if we've aligned with goal heading
            if abs(normalize_angle(new_theta - goal_theta)) < math.radians(1.0):
                path.append((x, y, goal_theta, 0.0, 0.0))  # Final state with exact heading
                break
                
            path.append((x, y, new_theta, turning_velocity, steering_angle))
            theta = new_theta
        
        # Create a new goal node with the refined heading
        refined_node = RRTNode(x, y, goal_theta, 0.0, 0.0, 
                        cost=near_goal_node.cost + len(path)*self.dt,
                        parent=near_goal_node.parent)
        refined_node.path_from_parent = path
        
        # Replace the goal node
        self.goal_node = refined_node
    
    def _extract_path(self):
        """Extract the full path from start to goal."""
        if not self.goal_found:
            return None
            
        path = []
        node = self.goal_node
        
        while node is not None:
            # Add node's path from parent in reverse order
            if node.path_from_parent:
                for point in reversed(node.path_from_parent):
                    # Only keep (x, y, theta)
                    if not path or (point[0], point[1], point[2]) != path[0]:
                        path.insert(0, (point[0], point[1], point[2]))
            # If no path from parent, add the node itself
            elif node.parent is None and (not path or (node.x, node.y, node.theta) != path[0]):
                path.insert(0, (node.x, node.y, node.theta))
                
            node = node.parent
            
        return path
    
    def _visualize_tree(self, title="Optimized RRT Tree", show_goal_path=False):
        """Visualize the RRT tree with path if found."""
        plt.figure(figsize=(12, 12))
        plt.imshow(self.grid.T, origin='lower', cmap='gray')
        
        # Plot a subset of edges for visualization efficiency
        max_edges = min(300, len(self.nodes))
        sample_step = max(1, len(self.nodes) // max_edges)
        
        for i in range(0, len(self.nodes), sample_step):
            node = self.nodes[i]
            if node.parent:
                if node.path_from_parent:
                    xs = [p[0] for p in node.path_from_parent]
                    ys = [p[1] for p in node.path_from_parent]
                    plt.plot(xs, ys, 'b-', alpha=0.3, linewidth=0.5)
        
        # Plot start and goal
        plt.scatter(self.start.x, self.start.y, color='green', s=100, marker='o', label='Start')
        plt.scatter(self.goal.x, self.goal.y, color='red', s=100, marker='x', label='Goal')
        
        # Plot goal path if found
        if show_goal_path and self.goal_found:
            path = self._extract_path()
            if path:
                xs = [p[0] for p in path]
                ys = [p[1] for p in path]
                plt.plot(xs, ys, 'g-', linewidth=2, label='Path')
                
        plt.title(title)
        plt.legend()
        plt.tight_layout()
        plt.show()
    
    def plan(self, visualize_steps=False, visualize_final=True):
        """
        Run the Optimized Kinodynamic RRT algorithm.
        
        Args:
            visualize_steps: Whether to visualize intermediate steps
            visualize_final: Whether to visualize the final tree
            
        Returns:
            List of (x, y, theta) poses if path found, None otherwise
        """
        print(f"Starting Optimized Kinodynamic RRT with {self.max_iter} iterations")
        print(f"Using PFF controller and motion primitives for faster planning")
        self.start_time = time.time()
        
        # Initial KD-tree build
        self._build_kdtree()
        
        # Main loop with time budget
        for i in range(self.max_iter):
            # Check time budget
            elapsed = time.time() - self.start_time
            if elapsed > self.time_budget:
                if self.goal_found:
                    print(f"Time budget reached after {i} iterations, path found")
                    break
                else:
                    # Increase goal bias if no path found yet
                    self.goal_sample_rate = min(0.7, self.goal_sample_rate * 1.2)
            
            # Progress reporting (reduced frequency)
            if (i+1) % 500 == 0:
                print(f"Iteration {i+1}, nodes: {len(self.nodes)}, time: {elapsed:.2f}s")
                
            if visualize_steps and (i+1) % 500 == 0:
                self._visualize_tree(title=f"RRT Tree at iteration {i+1}")
                
            # 1. Sample random position with informed sampling when possible
            rand_position = self._informed_sampling() if self.goal_found else self._random_state()
            
            # 2. Find nearest node using KD-tree
            nearest_node = self._nearest_node(rand_position)
            
            # 3. Generate new state with kinodynamic constraints
            new_node = self._new_state(nearest_node, rand_position)
            
            # Skip if no valid state
            if new_node is None:
                continue
                
            # 4. Add node to the tree
            self.nodes.append(new_node)
            
            # 5. Check if goal reached
            if not self.goal_found and self._is_goal_reached(new_node):
                self.goal_found = True
                self.goal_node = new_node
                print(f"Found path to goal at iteration {i+1}, time: {elapsed:.2f}s")
                
                # Check if heading needs refinement
                angle_diff = abs(normalize_angle(new_node.theta - self.goal.theta))
                if angle_diff > math.radians(5.0):  # More than 5 degrees off
                    print("Path found but refining final heading...")
                    self._refine_final_heading(new_node)
                
                # Early termination when a "good enough" path is found
                direct_dist = self.direct_distance
                path_length = new_node.cost
                
                # If path is within 30% of optimal length estimate or time is running out
                if path_length < direct_dist * 1.3 or elapsed > 0.8 * self.time_budget:
                    print("Found good enough path, terminating early")
                    break
        
        # Check if a path was found
        if not self.goal_found:
            print(f"No path found in {time.time() - self.start_time:.2f}s")
            if visualize_final:
                self._visualize_tree(title="RRT Tree (No path found)")
            return None
            
        # Extract path
        path = self._extract_path()
        total_time = time.time() - self.start_time
        
        print(f"Path found with {len(path)} points in {total_time:.2f} seconds")
        
        if visualize_final:
            self._visualize_tree(title=f"Path found in {total_time:.2f}s", show_goal_path=True)
            
        return path
        
    # def replan(self, updated_occupancy_grid, collision_lookup, current_state):
    #     """
    #     Replan efficiently when occupancy grid changes.
        
    #     Args:
    #         updated_occupancy_grid: New occupancy grid
    #         collision_lookup: New collision lookup table
    #         current_state: Current robot state (x, y, theta)
            
    #     Returns:
    #         New path or None if no path found
    #     """
    #     # Update environment information
    #     self.grid = updated_occupancy_grid
    #     self.collision_lookup = collision_lookup
        
    #     # Set new starting point (robot's current position)
    #     self.start = RRTNode(current_state[0], current_state[1], current_state[2])
        
    #     # Set shorter time budget for replanning
    #     old_time_budget = self.time_budget
    #     self.time_budget = 0.5  # 500ms for replanning
        
    #     # Check if the current path is still valid
    #     if self.goal_found:
    #         old_path = self._extract_path()
    #         if old_path:
    #             # Check if old path is collision-free
    #             valid = True
    #             for state in old_path:
    #                 if fast_collision_check(state[0], state[1], self.collision_lookup):
    #                     valid = False
    #                     break
                
    #             if valid:
    #                 # Path is still valid, no need to replan
    #                 print("Current path still valid, no replanning needed")
    #                 return old_path
        
    #     # Path invalid or not found, replan from current position
    #     # Try to reuse valid parts of the tree
    #     valid_nodes = []
    #     for node in self.nodes:
    #         if not fast_collision_check(node.x, node.y, self.collision_lookup):
    #             valid_nodes.append(node)
        
    #     # Reset tree with valid nodes
    #     self.nodes = [self.start]
    #     if valid_nodes and len(valid_nodes) > 50:  # Only reuse if enough valid nodes
    #         self.nodes.extend(valid_nodes[:min(500, len(valid_nodes))])  # Limit to 500 nodes
        
    #     # Reset goal found status
    #     self.goal_found = False
    #     self.goal_node = None
        
    #     # Plan new path
    #     new_path = self.plan()
        
    #     # Restore original time budget
    #     self.time_budget = old_time_budget
        
    #     return new_path



from typing import List, Tuple, Union
from ..component import Component
from ...state import (
    AllState,
    VehicleState,
    EntityRelation,
    EntityRelationEnum,
    Path,
    Trajectory,
    Route,
    ObjectFrameEnum,
    AgentState,
    MissionEnum,
)
from ...utils import serialization
from ...mathutils.transforms import vector_madd
from ...mathutils.quadratic_equation import quad_root


import time
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import math
from scipy.optimize import minimize


# Global variables
PEDESTRIAN_LENGTH = 0.5
PEDESTRIAN_WIDTH = 0.5

VEHICLE_LENGTH = 3.5
VEHICLE_WIDTH = 2

VEHICLE_BUFFER_X = 3.0
VEHICLE_BUFFER_Y = 1.5

YIELD_BUFFER_Y = 1.0
V_MAX = 5
COMFORT_DECELERATION = 1.5


def detect_collision(
    curr_x: float,
    curr_y: float,
    curr_v: float,
    obj: AgentState,
    min_deceleration: float,
    max_deceleration: float,
    acceleration: float,
    max_speed: float,
) -> Tuple[bool, Union[float, List[float]]]:
    """Detects if a collision will occur with the given object and return deceleration to avoid it."""

    # Get the object's position and velocity
    obj_x = obj.pose.x
    obj_y = obj.pose.y
    obj_v_x = obj.velocity[0]
    obj_v_y = obj.velocity[1]

    if obj.pose.frame == ObjectFrameEnum.CURRENT:
        # Simulation: Current
        obj_x = obj.pose.x + curr_x
        obj_y = obj.pose.y + curr_y
        print("PEDESTRIAN", obj_x, obj_y)

    vehicle_front = curr_x + VEHICLE_LENGTH
    vehicle_back = curr_x
    vehicle_left = curr_y + VEHICLE_WIDTH / 2
    vehicle_right = curr_y - VEHICLE_WIDTH / 2

    pedestrian_front = obj_x + PEDESTRIAN_LENGTH / 2
    pedestrian_back = obj_x - PEDESTRIAN_LENGTH / 2
    pedestrian_left = obj_y + PEDESTRIAN_WIDTH / 2
    pedestrian_right = obj_y - PEDESTRIAN_WIDTH / 2

    # Check if the object is in front of the vehicle
    if vehicle_front > pedestrian_back:
        if vehicle_back > pedestrian_front:
            # The object is behind the vehicle
            print("Object is behind the vehicle")
            return False, 0.0
        if (
            vehicle_right - VEHICLE_BUFFER_Y > pedestrian_left
            or vehicle_left + VEHICLE_BUFFER_Y < pedestrian_right
        ):
            # The object is to the side of the vehicle
            print("Object is to the side of the vehicle")
            return False, 0.0
        # The object overlaps with the vehicle's buffer
        return True, max_deceleration

    if vehicle_right - VEHICLE_BUFFER_Y > pedestrian_left and obj_v_y <= 0:
        # The object is to the right of the vehicle and moving away
        print("Object is to the right of the vehicle and moving away")
        return False, 0.0

    if vehicle_left + VEHICLE_BUFFER_Y < pedestrian_right and obj_v_y >= 0:
        # The object is to the left of the vehicle and moving away
        print("Object is to the left of the vehicle and moving away")
        return False, 0.0

    if vehicle_front + VEHICLE_BUFFER_X >= pedestrian_back and (
        vehicle_right - VEHICLE_BUFFER_Y <= pedestrian_left
        and vehicle_left + VEHICLE_BUFFER_Y >= pedestrian_right
    ):
        # The object is in front of the vehicle and within the buffer
        print("Object is in front of the vehicle and within the buffer")
        return True, max_deceleration

    # Calculate the deceleration needed to avoid a collision
    print("Object is in front of the vehicle and outside the buffer")
    distance = pedestrian_back - vehicle_front
    distance_with_buffer = pedestrian_back - vehicle_front - VEHICLE_BUFFER_X

    relative_v = curr_v - obj_v_x
    if relative_v <= 0:
        return False, 0.0

    if obj_v_y == 0:
        # The object is in front of the vehicle blocking it
        deceleration = relative_v**2 / (2 * distance_with_buffer)
        if deceleration > max_deceleration:
            return True, max_deceleration
        if deceleration < min_deceleration:
            return False, 0.0

        return True, deceleration

    print(relative_v, distance_with_buffer)

    if obj_v_y > 0:
        # The object is to the right of the vehicle and moving towards it
        time_to_get_close = (
            vehicle_right - VEHICLE_BUFFER_Y - YIELD_BUFFER_Y - pedestrian_left
        ) / abs(obj_v_y)
        time_to_pass = (
            vehicle_left + VEHICLE_BUFFER_Y + YIELD_BUFFER_Y - pedestrian_right
        ) / abs(obj_v_y)
    else:
        # The object is to the left of the vehicle and moving towards it
        time_to_get_close = (
            pedestrian_right - vehicle_left - VEHICLE_BUFFER_Y - YIELD_BUFFER_Y
        ) / abs(obj_v_y)
        time_to_pass = (
            pedestrian_left - vehicle_right + VEHICLE_BUFFER_Y + YIELD_BUFFER_Y
        ) / abs(obj_v_y)

    time_to_accel_to_max_speed = (max_speed - curr_v) / acceleration
    distance_to_accel_to_max_speed = (
        (max_speed + curr_v - 2 * obj_v_x) * time_to_accel_to_max_speed / 2
    )  # area of trapezoid

    if distance_to_accel_to_max_speed > distance_with_buffer:
        # The object will reach the buffer before reaching max speed
        time_to_buffer_when_accel = (
            -relative_v
            + (relative_v * relative_v + 2 * distance_with_buffer * acceleration) ** 0.5
        ) / acceleration
    else:
        # The object will reach the buffer after reaching max speed
        time_to_buffer_when_accel = time_to_accel_to_max_speed + (
            distance_with_buffer - distance_to_accel_to_max_speed
        ) / (max_speed - obj_v_x)

    if distance_to_accel_to_max_speed > distance:
        # We will collide before reaching max speed
        time_to_collide_when_accel = (
            -relative_v + (relative_v * relative_v + 2 * distance * acceleration) ** 0.5
        ) / acceleration
    else:
        # We will collide after reaching max speed
        time_to_collide_when_accel = time_to_accel_to_max_speed + (
            distance - distance_to_accel_to_max_speed
        ) / (max_speed - obj_v_x)

    if time_to_get_close > time_to_collide_when_accel:
        # We can do normal driving and will pass the object before it gets in our way
        print(
            "We can do normal driving and will pass the object before it gets in our way"
        )
        return False, 0.0

    if vehicle_front + VEHICLE_BUFFER_X >= pedestrian_back:
        # We cannot move pass the pedestrian before it reaches the buffer from side
        return True, max_deceleration

    if time_to_pass < time_to_buffer_when_accel:
        # The object will pass through our front before we drive normally and reach it
        print(
            "The object will pass through our front before we drive normally and reach it"
        )
        return False, 0.0

    distance_to_move = distance_with_buffer + time_to_pass * obj_v_x

    if curr_v**2 / (2 * distance_to_move) >= COMFORT_DECELERATION:
        return True, curr_v**2 / (2 * distance_to_move)

    print("Calculating cruising speed")
    return True, [distance_to_move, time_to_pass]


def detect_collision_analytical(
    r_pedestrain_x: float,
    r_pedestrain_y: float,
    p_vehicle_left_y_after_t: float,
    p_vehicle_right_y_after_t: float,
    lateral_buffer: float,
) -> Union[bool, str]:
    """Detects if a collision will occur with the given object and return deceleration to avoid it. Analytical"""
    if r_pedestrain_x < 0 and abs(r_pedestrain_y) > lateral_buffer:
        return False
    elif r_pedestrain_x < 0:
        return "max"
    if (
        r_pedestrain_y >= p_vehicle_left_y_after_t
        and r_pedestrain_y <= p_vehicle_right_y_after_t
    ):
        return True

    return False


def get_minimum_deceleration_for_collision_avoidance(
    curr_x: float,
    curr_y: float,
    curr_v: float,
    obj: AgentState,
    min_deceleration: float,
    max_deceleration: float,
) -> Tuple[bool, float]:
    """Detects if a collision will occur with the given object and return deceleration to avoid it. Via Optimization"""

    # Get the object's position and velocity
    obj_x = obj.pose.x
    obj_y = obj.pose.y
    obj_v_x = obj.velocity[0]
    obj_v_y = obj.velocity[1]

    if obj.pose.frame == ObjectFrameEnum.CURRENT:
        obj_x = obj.pose.x + curr_x
        obj_y = obj.pose.y + curr_y

    obj_x = obj_x - curr_x
    obj_y = obj_y - curr_y

    curr_x = curr_x - curr_x
    curr_y = curr_y - curr_y

    vehicle_front = curr_x + VEHICLE_LENGTH + VEHICLE_BUFFER_X
    vehicle_back = curr_x
    vehicle_left = curr_y - VEHICLE_WIDTH / 2
    vehicle_right = curr_y + VEHICLE_WIDTH / 2

    r_vehicle_front = vehicle_front - vehicle_front
    r_vehicle_back = vehicle_back - vehicle_front
    r_vehicle_left = vehicle_left - VEHICLE_BUFFER_Y
    r_vehicle_right = vehicle_right + VEHICLE_BUFFER_Y
    r_vehicle_v_x = curr_v
    r_vehicle_v_y = 0

    r_pedestrain_x = obj_x - vehicle_front
    r_pedestrain_y = -obj_y
    r_pedestrain_v_x = obj_v_x
    r_pedestrain_v_y = -obj_v_y

    r_velocity_x_from_vehicle = r_vehicle_v_x - r_pedestrain_v_x
    r_velocity_y_from_vehicle = r_vehicle_v_y - r_pedestrain_v_y

    t_to_r_pedestrain_x = (r_pedestrain_x - r_vehicle_front) / r_velocity_x_from_vehicle

    p_vehicle_left_y_after_t = (
        r_vehicle_left + r_velocity_y_from_vehicle * t_to_r_pedestrain_x
    )
    p_vehicle_right_y_after_t = (
        r_vehicle_right + r_velocity_y_from_vehicle * t_to_r_pedestrain_x
    )

    collision_flag = detect_collision_analytical(
        r_pedestrain_x,
        r_pedestrain_y,
        p_vehicle_left_y_after_t,
        p_vehicle_right_y_after_t,
        VEHICLE_BUFFER_Y,
    )
    if collision_flag == False:
        print(
            "No collision",
            curr_x,
            curr_y,
            r_pedestrain_x,
            r_pedestrain_y,
            r_vehicle_left,
            r_vehicle_right,
            p_vehicle_left_y_after_t,
            p_vehicle_right_y_after_t,
        )
        return 0.0, r_pedestrain_x
    elif collision_flag == "max":
        return max_deceleration, r_pedestrain_x

    print(
        "Collision",
        curr_x,
        curr_y,
        r_pedestrain_x,
        r_pedestrain_y,
        r_vehicle_left,
        r_vehicle_right,
        p_vehicle_left_y_after_t,
        p_vehicle_right_y_after_t,
    )

    minimum_deceleration = None
    if abs(r_velocity_y_from_vehicle) > 0.1:
        if r_velocity_y_from_vehicle > 0.1:
            # Vehicle Left would be used to yield
            r_pedestrain_y_temp = r_pedestrain_y + abs(r_vehicle_left)
        elif r_velocity_y_from_vehicle < -0.1:
            # Vehicle Right would be used to yield
            r_pedestrain_y_temp = r_pedestrain_y - abs(r_vehicle_right)

        softest_accleration = (
            2
            * r_velocity_y_from_vehicle
            * (
                r_velocity_y_from_vehicle * r_pedestrain_x
                - r_velocity_x_from_vehicle * r_pedestrain_y_temp
            )
            / r_pedestrain_y_temp**2
        )
        peak_y = (
            -(r_velocity_x_from_vehicle * r_velocity_y_from_vehicle)
            / softest_accleration
        )
        # if the peak is within the position of the pedestrian,
        # then it indicates the path had already collided with the pedestrian,
        # and so the softest acceleration should be the one the peak of the path is the same as the pedestrain's x position
        # and the vehicle should be stopped exactly before the pedestrain's x position
        if abs(peak_y) > abs(r_pedestrain_y_temp):
            minimum_deceleration = abs(softest_accleration)
        # else: the vehicle should be stopped exactly before the pedestrain's x position the same case as the pedestrain barely move laterally
    if minimum_deceleration is None:
        minimum_deceleration = r_velocity_x_from_vehicle**2 / (2 * r_pedestrain_x)

    print("calculated minimum deceleration: ", minimum_deceleration)

    if minimum_deceleration < min_deceleration:
        return 0.0, r_pedestrain_x
    else:
        return (
            max(min(minimum_deceleration, max_deceleration), min_deceleration),
            r_pedestrain_x,
        )


################################################################################
########## Longitudinal Planning ###############################################
################################################################################


def longitudinal_plan(
    path: Path,
    acceleration: float,
    deceleration: float,
    max_speed: float,
    current_speed: float,
    method: str,
) -> Trajectory:
    """Generates a longitudinal trajectory for a path with a
    trapezoidal velocity profile.

    1. accelerates from current speed toward max speed
    2. travel along max speed
    3. if at any point you can't brake before hitting the end of the path,
       decelerate with accel = -deceleration until velocity goes to 0.
    """

    if method == "milestone":
        return longitudinal_plan_milestone(
            path, acceleration, deceleration, max_speed, current_speed
        )
    elif method == "dt":
        return longitudinal_plan_dt(
            path, acceleration, deceleration, max_speed, current_speed
        )
    elif method == "dx":
        return longitudinal_plan_dx(
            path, acceleration, deceleration, max_speed, current_speed
        )
    else:
        raise NotImplementedError(
            "Invalid method, only milestone, dt, adn dx are implemented."
        )


def longitudinal_plan_milestone(
    path: Path,
    acceleration: float,
    deceleration: float,
    max_speed: float,
    current_speed: float,
) -> Trajectory:
    """Generates a longitudinal trajectory for a path with a
    trapezoidal velocity profile.

    1. accelerates from current speed toward max speed
    2. travel along max speed
    3. if at any point you can't brake before hitting the end of the path,
       decelerate with accel = -deceleration until velocity goes to 0.
    """
    # Extrapolation factor for the points
    factor = 5.0
    new_points = []
    for idx, point in enumerate(path.points[:-1]):
        next_point = path.points[idx + 1]
        if point[0] == next_point[0]:
            break
        xarange = np.arange(
            point[0], next_point[0], (next_point[0] - point[0]) / factor
        )
        if point[1] == next_point[1]:
            yarange = [point[1]] * len(xarange)
        else:
            yarange = np.arange(
                point[1], next_point[1], (next_point[1] - point[1]) / factor
            )
        print(yarange)
        for x, y in zip(xarange, yarange):
            new_points.append((x, y))
    new_points.append(path.points[-1])

    print("new points", new_points)
    path = Path(path.frame, new_points)

    path_normalized = path.arc_length_parameterize()
    points = [p for p in path_normalized.points]
    times = [t for t in path_normalized.times]
    # =============================================

    print("-----LONGITUDINAL PLAN-----")
    # print("path length: ", path.length())
    length = path.length()

    # If the path is too short, just return the path for preventing sudden halt of simulation
    if length < 0.05:
        return Trajectory(path.frame, points, times)

    # Starting point
    x0 = points[0][0]
    cur_point = points[0]
    cur_time = times[0]
    cur_index = 0

    new_points = []
    new_times = []
    velocities = []  # for graphing and debugging purposes

    while current_speed > 0 or cur_index == 0:
        # we want to iterate through all the points and add them
        # to the new points. However, we also want to add "critical points"
        # where we reach top speed, begin decelerating, and stop
        new_points.append(cur_point)
        new_times.append(cur_time)
        velocities.append(current_speed)
        # print("=====================================")
        # print("new points: ", new_points)
        # print("current index: ", cur_index)
        # print("current speed: ", current_speed)
        # print("current position: ", cur_point)

        # Information we will need:
        # Calculate how much time it would take to stop
        # Calculate how much distance it would take to stop
        min_delta_t_stop = current_speed / deceleration
        min_delta_x_stop = (
            current_speed * min_delta_t_stop - 0.5 * deceleration * min_delta_t_stop**2
        )
        # print(min_delta_x_stop)
        assert min_delta_x_stop >= 0

        # Check if we are done

        # If we cannot stop before or stop exactly at the final position requested
        if cur_point[0] + min_delta_x_stop >= points[-1][0]:
            # put on the breaks

            # Calculate the next point in a special manner because of too-little time to stop
            if cur_index == len(points) - 1:
                # the next point in this instance would be when we stop
                next_point = (cur_point[0] + min_delta_x_stop, 0)
            else:
                next_point = points[cur_index + 1]

            # keep breaking until the next milestone in path
            if next_point[0] <= points[-1][0]:
                # print("continuing to next point")
                delta_t_to_next_x = compute_time_to_x(
                    cur_point[0], next_point[0], current_speed, -deceleration
                )
                cur_time += delta_t_to_next_x
                cur_point = next_point
                current_speed -= deceleration * delta_t_to_next_x
                cur_index += 1
            else:
                # continue to the point in which we would stop (current_velocity = 0)
                # update to the next point
                delta_t_to_next_x = compute_time_to_x(
                    cur_point[0], next_point[0], current_speed, -deceleration
                )
                cur_point = next_point
                cur_time += delta_t_to_next_x
                # current_speed would not be exactly zero error would be less than 1e-4 but perfer to just set to zero
                # current_speed -= delta_t_to_next_x*deceleration
                current_speed = 0
                assert current_speed == 0

        # This is the case where we are accelerating to max speed
        # because the first if-statement covers for when we decelerating,
        # the only time current_speed < max_speed is when we are accelerating
        elif current_speed < max_speed:
            # print("In case two")
            # next point
            next_point = points[cur_index + 1]
            # accelerate to max speed

            # calculate the time it would take to reach max speed
            delta_t_to_max_speed = (max_speed - current_speed) / acceleration
            # calculate the distance it would take to reach max speed
            delta_x_to_max_speed = (
                current_speed * delta_t_to_max_speed
                + 0.5 * acceleration * delta_t_to_max_speed**2
            )

            delta_t_to_stop_from_max_speed = max_speed / deceleration
            delta_x_to_stop_from_max_speed = (
                max_speed * delta_t_to_stop_from_max_speed
                - 0.5 * deceleration * delta_t_to_stop_from_max_speed**2
            )

            delta_t_to_next_point = compute_time_to_x(
                cur_point[0], next_point[0], current_speed, acceleration
            )
            velocity_at_next_point = (
                current_speed + delta_t_to_next_point * acceleration
            )
            time_to_stop_from_next_point = velocity_at_next_point / deceleration
            delta_x_to_stop_from_next_point = (
                velocity_at_next_point * time_to_stop_from_next_point
                - 0.5 * deceleration * time_to_stop_from_next_point**2
            )
            # if we would reach max speed after the next point,
            # just move to the next point and update the current speed and time
            if (
                next_point[0] + delta_x_to_stop_from_next_point < points[-1][0]
                and cur_point[0] + delta_x_to_max_speed >= next_point[0]
            ):
                # ("go to next point")
                # accelerate to max speed
                delta_t_to_next_x = compute_time_to_x(
                    cur_point[0], next_point[0], current_speed, acceleration
                )
                cur_time += delta_t_to_next_x
                cur_point = [next_point[0], 0]
                current_speed += delta_t_to_next_x * acceleration
                cur_index += 1

            # This is the case where we would need to start breaking before reaching
            # top speed and before the next point (i.e. triangle shape velocity)
            elif (
                cur_point[0] + delta_x_to_max_speed + delta_x_to_stop_from_max_speed
                >= points[-1][0]
            ):
                # print(delta_x_to_max_speed)
                # print(delta_x_to_stop_from_max_speed)
                # Add a new point at the point where we should start breaking
                # print("Adding new point to start breaking")
                delta_t_to_next_x = compute_time_triangle(
                    cur_point[0],
                    points[-1][0],
                    current_speed,
                    0,
                    acceleration,
                    deceleration,
                )
                # print(delta_t_to_next_x)
                # delta_t_to_next_x = compute_time_to_x(cur_point[0], points[-1][0] - min_delta_x_stop, current_speed, acceleration)
                next_x = (
                    cur_point[0]
                    + current_speed * delta_t_to_next_x
                    + 0.5 * acceleration * delta_t_to_next_x**2
                )
                cur_time += delta_t_to_next_x
                cur_point = [next_x, 0]
                current_speed += delta_t_to_next_x * acceleration

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
            next_point = points[cur_index + 1]
            # continue on with max speed
            # print("In case three")

            # add point to start decelerating
            if next_point[0] + min_delta_x_stop >= points[-1][0]:
                # print("Adding new point to start decelerating")
                cur_time += (
                    points[-1][0] - min_delta_x_stop - cur_point[0]
                ) / current_speed
                cur_point = [points[-1][0] - min_delta_x_stop, 0]
                current_speed = max_speed
            else:
                # Continue on to next point
                # print("Continuing on to next point")
                cur_time += (next_point[0] - cur_point[0]) / current_speed
                cur_point = next_point
                cur_index += 1

        # This is an edge case and should only be reach
        # if the initial speed is greater than the max speed
        elif current_speed > max_speed:
            # We need to hit the breaks

            next_point = points[cur_index + 1]
            # print("In case four")
            # slow down to max speed
            delta_t_to_max_speed = (current_speed - max_speed) / deceleration
            delta_x_to_max_speed = (
                current_speed * delta_t_to_max_speed
                - 0.5 * deceleration * delta_t_to_max_speed**2
            )

            # If we would reach the next point before slowing down to max speed
            # keep going until we reach the next point
            if cur_point[0] + delta_x_to_max_speed >= next_point[0]:
                delta_t_to_next_x = compute_time_to_x(
                    cur_point[0], next_point[0], current_speed, -deceleration
                )
                cur_time += delta_t_to_next_x
                cur_point = [next_point[0], 0]
                current_speed -= delta_t_to_next_x * deceleration
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

    # =============================================

    trajectory = Trajectory(path.frame, points, times, velocities)
    return trajectory


def compute_time_to_x(x0: float, x1: float, v: float, a: float) -> float:
    """Computes the time to go from x0 to x1 with initial velocity v0 and final velocity v1
    with constant acceleration a. I am assuming that we will always have a solution by settings
    discriminant equal to zero, i'm not sure if this is an issue."""

    """Consider changing the system to use linear operators instead of explicitly calculating because of instances here"""

    t1 = (-v + max(0, (v**2 - 2 * a * (x0 - x1))) ** 0.5) / a
    t2 = (-v - max(0, (v**2 - 2 * a * (x0 - x1))) ** 0.5) / a

    if math.isnan(t1):
        t1 = 0
    if math.isnan(t2):
        t2 = 0

    valid_times = [n for n in [t1, t2] if n > 0]
    if valid_times:
        return min(valid_times)
    else:
        return 0.0


def compute_time_triangle(
    x0: float, xf: float, v0: float, vf: float, acceleration: float, deceleration: float
) -> float:
    """
    Compute the time to go from current point assuming we are accelerating to the point at which
    we would need to start breaking in order to reach the final point with velocity 0.
    """
    roots = quad_root(
        0.5 * acceleration
        + acceleration**2 / deceleration
        - 0.5 * acceleration**2 / deceleration,
        v0 + 2 * acceleration * v0 / deceleration - acceleration * v0 / deceleration,
        x0 - xf + v0**2 / deceleration - 0.5 * v0**2 / deceleration,
    )
    t1 = max(roots)
    assert t1 > 0
    return t1


def solve_for_v_peak(
    v0: float, acceleration: float, deceleration: float, total_length: float
) -> float:

    if acceleration <= 0 or deceleration <= 0:
        raise ValueError("Acceleration and deceleration cant be negative")

    # Formuala: (v_peak^2 - v0^2)/(2*a) + v_peak^2/(2*d) = total_length
    numerator = deceleration * v0**2 + 2 * acceleration * deceleration * total_length
    denominator = acceleration + deceleration
    v_peak_sq = numerator / denominator

    if v_peak_sq < 0:
        return 0.0

    return math.sqrt(v_peak_sq)


def compute_dynamic_dt(acceleration, speed, k=0.01, a_min=0.5):
    position_step = k * max(speed, 1.0)  # Ensures position step is speed-dependent
    return np.sqrt(2 * position_step / max(acceleration, a_min))


def longitudinal_plan_dt(
    path,
    acceleration: float,
    deceleration: float,
    max_speed: float,
    current_speed: float,
):
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
        t_accel = (
            (max_speed - current_speed) / acceleration
            if max_speed > current_speed
            else 0.0
        )
        t_decel = max_speed / deceleration
        d_cruise = total_length - d_accel - d_decel
        t_cruise = d_cruise / max_speed if max_speed != 0 else 0.0
        t_final = t_accel + t_cruise + t_decel
        profile_type = "trapezoidal"
    else:
        # Triangular profile: not enough distance to reach max_speed so we will calculate peak speed.
        peak_speed = solve_for_v_peak(
            current_speed, acceleration, deceleration, total_length
        )
        # choose the min just in case
        peak_speed = min(peak_speed, max_speed)
        t_accel = (
            (peak_speed - current_speed) / acceleration
            if peak_speed > current_speed
            else 0.0
        )
        t_decel = peak_speed / deceleration
        t_final = t_accel + t_decel
        profile_type = "triangular"

    t = 0
    times = []
    s_vals = []
    velocities = []  # for graphing and debugging purposes

    num_time_steps = 0
    speed = current_speed
    while t < t_final:
        times.append(t)
        velocities.append(speed)
        if profile_type == "trapezoidal":
            if t < t_accel:
                # Acceleration phase.
                s = current_speed * t + 0.5 * acceleration * t**2
                speed = current_speed + acceleration * t
            elif t < t_accel + t_cruise:
                # Cruise phase.
                s = d_accel + max_speed * (t - t_accel)
            else:
                # Deceleration phase.
                t_decel_phase = t - (t_accel + t_cruise)
                s = total_length - 0.5 * deceleration * (t_decel - t_decel_phase) ** 2
                speed = speed - deceleration * (t_decel - t_decel_phase)
        else:  # Triangular profile.
            if t < t_accel:
                # Acceleration phase.
                s = current_speed * t + 0.5 * acceleration * t**2
                speed = current_speed + acceleration * t
            else:
                t_decel_phase = t - t_accel
                s_accel = current_speed * t_accel + 0.5 * acceleration * t_accel**2
                s = (
                    s_accel
                    + peak_speed * t_decel_phase
                    - 0.5 * deceleration * t_decel_phase**2
                )
                speed = speed - deceleration * t_decel_phase

        s_vals.append(min(s, total_length))
        if s >= total_length:
            break

        dt = compute_dynamic_dt(
            acceleration if t < t_accel else deceleration, current_speed
        )
        t = t + dt

        num_time_steps += 1

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

    trajectory = Trajectory(path_norm.frame, points, list(times), velocities=velocities)
    return trajectory


def longitudinal_plan_dx(
    path: Path,
    acceleration: float,
    deceleration: float,
    max_speed: float,
    current_speed: float,
) -> Trajectory:
    """Generates a longitudinal trajectory for a path with a
    trapezoidal velocity profile.

    1. accelerates from current speed toward max speed
    2. travel along max speed
    3. if at any point you can't brake before hitting the end of the path,
       decelerate with accel = -deceleration until velocity goes to 0.
    """
    path_normalized = path.arc_length_parameterize()
    points = [p for p in path_normalized.points]
    times = [t for t in path_normalized.times]

    # =============================================
    # Adjust these two numbers to choose between computation speed or smoothness
    rq = 0.1  # Smaller, smoother
    multi = 5  # Larger, smoother
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
    velocities = []  # for graphing and debugging purposes

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
        min_delta_t_stop = current_speed / deceleration
        min_delta_x_stop = (
            current_speed * min_delta_t_stop - 0.5 * deceleration * min_delta_t_stop**2
        )
        assert min_delta_x_stop >= 0

        # Check if we are done

        # If we cannot stop before or stop exactly at the final position requested
        if cur_point[0] + min_delta_x_stop >= points[-1][0] - 0.0001:
            acc = deceleration
            flag = 1
            print("In case one")
            # put on the breaks
            # Calculate the next point in a special manner because of too-little time to stop
            if cur_index >= len(points) - 1:
                # the next point in this instance would be when we stop
                print(1)
                if min_delta_x_stop < rq * acc:
                    next_point = (cur_point[0] + min_delta_x_stop, 0)
                else:
                    next_point = (cur_point[0] + (min_delta_x_stop / (acc * multi)), 0)
                    flag = 0
            else:
                print(2)
                next_point = points[cur_index + 1]
                if next_point[0] - cur_point[0] > rq * acc:
                    tmp = cur_point[0] + (next_point[0] - cur_point[0]) / (acc * multi)
                    flag = 0
                    next_point = [tmp, next_point[1]]

            # keep breaking until the next milestone in path
            print("continuing to next point")
            delta_t_to_next_x = compute_time_to_x(
                cur_point[0], next_point[0], current_speed, -deceleration
            )
            cur_time += delta_t_to_next_x
            cur_point = next_point
            current_speed -= deceleration * delta_t_to_next_x
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
            next_point = points[cur_index + 1]
            if next_point[0] - cur_point[0] > rq * acc:
                tmp = cur_point[0] + (next_point[0] - cur_point[0]) / (acc * multi)
                flag = 0
                next_point = [tmp, next_point[1]]
            # accelerate to max speed

            # calculate the time it would take to reach max speed
            delta_t_to_max_speed = (max_speed - current_speed) / acceleration
            # calculate the distance it would take to reach max speed
            delta_x_to_max_speed = (
                current_speed * delta_t_to_max_speed
                + 0.5 * acceleration * delta_t_to_max_speed**2
            )

            # if we would reach max speed after the next point,
            # just move to the next point and update the current speed and time
            if cur_point[0] + delta_x_to_max_speed >= next_point[0]:
                print("go to next point")
                # accelerate to max speed
                delta_t_to_next_x = compute_time_to_x(
                    cur_point[0], next_point[0], current_speed, acceleration
                )
                cur_time += delta_t_to_next_x
                cur_point = [next_point[0], 0]
                current_speed += delta_t_to_next_x * acceleration
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
            next_point = points[cur_index + 1]
            # continue on with max speed
            print("In case three")

            # add point to start decelerating
            if next_point[0] + min_delta_x_stop >= points[-1][0]:
                print("Adding new point to start decelerating")
                cur_time += (
                    points[-1][0] - min_delta_x_stop - cur_point[0]
                ) / current_speed
                cur_point = [points[-1][0] - min_delta_x_stop, 0]
                current_speed = max_speed
            else:
                # Continue on to next point
                print("Continuing on to next point")
                cur_time += (next_point[0] - cur_point[0]) / current_speed
                cur_point = next_point
                cur_index += 1

        # This is an edge case and should only be reach
        # if the initial speed is greater than the max speed
        elif current_speed > max_speed:
            # We need to hit the breaks
            acc = deceleration
            flag = 1
            # next point
            next_point = points[cur_index + 1]
            if next_point[0] - cur_point[0] > rq * acc:
                tmp = cur_point[0] + (next_point[0] - cur_point[0]) / (acc * multi)
                flag = 0
                next_point = [tmp, next_point[1]]
            print("In case four")
            # slow down to max speed
            delta_t_to_max_speed = (current_speed - max_speed) / deceleration
            delta_x_to_max_speed = (
                current_speed * delta_t_to_max_speed
                - 0.5 * deceleration * delta_t_to_max_speed**2
            )

            # If we would reach the next point before slowing down to max speed
            # keep going until we reach the next point
            if cur_point[0] + delta_x_to_max_speed >= next_point[0]:
                delta_t_to_next_x = compute_time_to_x(
                    cur_point[0], next_point[0], current_speed, -deceleration
                )
                cur_time += delta_t_to_next_x
                cur_point = [next_point[0], 0]
                current_speed -= delta_t_to_next_x * deceleration
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

    # =============================================

    trajectory = Trajectory(path.frame, points, times)
    return trajectory


def longitudinal_brake(
    path: Path, deceleration: float, current_speed: float
) -> Trajectory:
    """Generates a longitudinal trajectory for braking along a path."""
    path_normalized = path.arc_length_parameterize()
    points = [p for p in path_normalized.points]
    times = [t for t in path_normalized.times]

    # =============================================

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

    # =============================================

    trajectory = Trajectory(path.frame, points, times, velocities)
    return trajectory


################################################################################
########## Yield Trajectory Planner ############################################
################################################################################


##########################
##### Patrick's Code #####
##########################


class YieldTrajectoryPlanner(Component):
    """Follows the given route.  Brakes if you have to yield or
    you are at the end of the route, otherwise accelerates to
    the desired speed.
    """

    def __init__(
        self,
        mode: str = "real",
        params: dict = {"planner": "dt", "desired_speed": 1.0, "acceleration": 0.5},
    ):
        self.route_progress = None
        self.t_last = None
        self.acceleration = 3.0
        self.desired_speed = 5.0
        self.deceleration = 2.0

        self.min_deceleration = 1.0
        self.max_deceleration = 8.0

        self.mode = mode
        self.planner = params["planner"]

    def state_inputs(self):
        return ["all"]

    def state_outputs(self) -> List[str]:
        return ["trajectory"]

    def rate(self):
        return 10.0

    def update(self, state: AllState):
        start_time = time.time()

        vehicle = state.vehicle  # type: VehicleState
        route = state.route  # type: Route
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

        ###############################################
        # print("@@@@@ VEHICLE STATE @@@@@")
        # print(vehicle)
        # print("@@@@@@@@@@@@@@@@@@@@@@@@@")

        if self.mode == "real":
            abs_x = curr_x
            abs_y = curr_y
        ###############################################

        if state.mission.type == MissionEnum.IDLE:
            return Trajectory(
                times=[0, 0], frame=ObjectFrameEnum.START, points=[[0, 0]]
            )

        # figure out where we are on the route
        if self.route_progress is None:
            self.route_progress = 0.0
        closest_dist, closest_parameter = state.route.closest_point_local(
            (curr_x, curr_y), [self.route_progress - 5.0, self.route_progress + 5.0]
        )
        self.route_progress = closest_parameter

        lookahead_distance = max(10, curr_v**2 / (2 * self.deceleration))
        route_with_lookahead = route.trim(
            closest_parameter, closest_parameter + lookahead_distance
        )
        # print("Lookahead distance:", lookahead_distance)

        route_to_end = route.trim(closest_parameter, len(route.points) - 1)

        should_yield = False
        yield_deceleration = 0.0

        # print("Current Speed: ", curr_v)

        for r in state.relations:
            if r.type == EntityRelationEnum.YIELDING and r.obj1 == "":
                # get the object we are yielding to
                obj = state.agents[r.obj2]

                detected, deceleration = detect_collision(
                    abs_x,
                    abs_y,
                    curr_v,
                    obj,
                    self.min_deceleration,
                    self.max_deceleration,
                    self.acceleration,
                    self.desired_speed,
                )
                if isinstance(deceleration, list):
                    print("@@@@@ INPUT", deceleration)
                    time_collision = deceleration[1]
                    distance_collision = deceleration[0]
                    b = 3 * time_collision - 2 * curr_v
                    c = curr_v**2 - 3 * distance_collision
                    desired_speed = (-b + (b**2 - 4 * c) ** 0.5) / 2
                    deceleration = 1.5
                    print("@@@@@ YIELDING", desired_speed)
                    route_yield = route.trim(
                        closest_parameter, closest_parameter + distance_collision
                    )
                    traj = longitudinal_plan(
                        route_yield,
                        self.acceleration,
                        deceleration,
                        desired_speed,
                        curr_v,
                        self.planner,
                    )
                    return traj
                else:
                    if detected and deceleration > 0:
                        yield_deceleration = deceleration
                        should_yield = True

                print("should yield: ", should_yield)

        should_accelerate = not should_yield and curr_v < self.desired_speed

        # choose whether to accelerate, brake, or keep at current velocity
        if should_accelerate:
            traj = longitudinal_plan(
                route_to_end,
                self.acceleration,
                self.deceleration,
                self.desired_speed,
                curr_v,
                self.planner,
            )
        elif should_yield:
            traj = longitudinal_brake(route_to_end, yield_deceleration, curr_v)
        else:
            traj = longitudinal_plan(
                route_to_end,
                0.0,
                self.deceleration,
                self.desired_speed,
                curr_v,
                self.planner,
            )

        return traj





from __future__ import print_function

# Python Headers
import os
import cv2 
import numpy as np

import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

import cv2
import numpy as np

image_path = os.path.join(os.path.dirname(__file__), "highbay_image.pgm")

class OccupancyGrid2:

    global image_path
    
    def __init__(self):

        # Read image in BGR format
        self.map_image = cv2.imread(image_path)

        # Create the cv_bridge object
        self.bridge  = CvBridge()
        self.map_image_pub = rospy.Publisher("/motion_image2", Image, queue_size=1) 

        # Subscribe information from sensors
        self.lat     = 0
        self.lon     = 0
        self.heading = 0

        self.lat_start_bt = 40.092722  # 40.09269  
        self.lon_start_l  = -88.236365 # -88.23628
        self.lat_scale    = 0.00062   # 0.0007    
        self.lon_scale    = 0.00136  # 0.00131   

        self.arrow        = 40 
        self.img_width    = 2107
        self.img_height   = 1313

    def image_to_gnss(self, pix_x, pix_y):
        """
        Given image coordinates (pix_x, pix_y), return (latitude, longitude).
        Inverse of your:
            pix_x = img_width * (lon - lon_start_l) / lon_scale
            pix_y = img_height - img_height*(lat - lat_start_bt) / lat_scale
        """
        # longitude:
        lon = self.lon_start_l + (pix_x / float(self.img_width)) * self.lon_scale
        lat = self.lat_start_bt + ((self.img_height - pix_y) / float(self.img_height)) * self.lat_scale

        return lat, lon

    def image_heading(self, lon_x, lat_y, heading):
        
        if(heading >=0 and heading < 90):
            angle  = np.radians(90-heading)
            lon_xd = lon_x + int(self.arrow * np.cos(angle))
            lat_yd = lat_y - int(self.arrow * np.sin(angle))

        elif(heading >= 90 and heading < 180):
            angle  = np.radians(heading-90)
            lon_xd = lon_x + int(self.arrow * np.cos(angle))
            lat_yd = lat_y + int(self.arrow * np.sin(angle))  

        elif(heading >= 180 and heading < 270):
            angle = np.radians(270-heading)
            lon_xd = lon_x - int(self.arrow * np.cos(angle))
            lat_yd = lat_y + int(self.arrow * np.sin(angle))

        else:
            angle = np.radians(heading-270)
            lon_xd = lon_x - int(self.arrow * np.cos(angle))
            lat_yd = lat_y - int(self.arrow * np.sin(angle)) 

        return lon_xd, lat_yd         


    def gnss_to_image_with_heading(self, lon, lat, heading):
        
        lon_x = int(self.img_width*(lon - self.lon_start_l)/self.lon_scale)
        lat_y = int(self.img_height-self.img_height*(lat - self.lat_start_bt)/self.lat_scale)
        lon_xd, lat_yd = self.image_heading(lon_x, lat_y, heading)

        pub_image = np.copy(self.map_image)

        if(lon_x >= 0 and lon_x <= self.img_width and lon_xd >= 0 and lon_xd <= self.img_width and 
            lat_y >= 0 and lat_y <= self.img_height and lat_yd >= 0 and lat_yd <= self.img_height):
            cv2.arrowedLine(pub_image, (lon_x, lat_y), (lon_xd, lat_yd), (0, 0, 255), 2)
            cv2.circle(pub_image, (lon_x, lat_y), 12, (0,0,255), 2)

            ## Debug check if you can convert back to GNSS
            # tmp_lat, tmp_lon = self.image_to_gnss(lon_x, lat_y)
            # print(f'Lat: {self.lat}, Lon: {self.lon}, Converted Lat: {tmp_lat}, Converted Lon: {tmp_lon}')
            ## End Debug check
        try:
            # Convert OpenCV image to ROS image and publish
            self.map_image_pub.publish(self.bridge.cv2_to_imgmsg(pub_image, "bgr8"))
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))

    def gnss_to_image(self, lon, lat):
        
        lon_x = int(self.img_width*(lon - self.lon_start_l) /self.lon_scale)
        lat_y = int(self.img_height  -  self.img_height*(lat - self.lat_start_bt)/self.lat_scale)
        print(f"GNSS hiiiii ({lat}, {lon}) → pixel ({lon_x}, {lat_y})")
        pub_image = np.copy(self.map_image)

        if(lon_x >= 0 and lon_x <= self.img_width and 
            lat_y >= 0 and lat_y <= self.img_height):
            cv2.circle(pub_image, (lon_x, lat_y), 12, (0,0,255), 2)


        try:
            # Convert OpenCV image to ROS image and publish
            self.map_image_pub.publish(self.bridge.cv2_to_imgmsg(pub_image, "bgr8"))
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))

    def gnss_to_image_coords(self, lon, lat):
        
        lon_x = int(self.img_width*(lon - self.lon_start_l) /self.lon_scale)
        lat_y = int(self.img_height  -  self.img_height*(lat - self.lat_start_bt)/self.lat_scale)
        print(f"GNSS ({lat}, {lon}) → pixel ({lon_x}, {lat_y})")
        return lon_x, lat_y

import os
import argparse
import time
import math
import matplotlib

from .collision import build_collision_lookup
from .map_utils import world_to_grid, grid_to_world
from .kinodynamic_rrt_star import OptimizedKinodynamicRRT  # Updated import
#from summon_rrt import BiRRT
from .map_utils import load_pgm_to_occupancy_grid
from .visualization import visualize_path, animate_path
# from test_dubins import run_tests

def optimized_kinodynamic_rrt_planning(start_world, goal_world, occupancy_grid,
                                safety_margin=2, vehicle_width=1.7, step_size=1.0,
                                turning_radius=3.0, max_iterations=3000):
    """
    Optimized Kinodynamic RRT planning from start to goal
    
    Args:
        start_world: (x, y, theta) start position in world coordinates
        goal_world: (x, y, theta) goal position in world coordinates
        occupancy_grid: Binary occupancy grid (1=obstacle, 0=free)
        metadata: Map metadata
        safety_margin: Safety margin in grid cells
        vehicle_width: Vehicle width in meters
        step_size: Step size for path sampling
        turning_radius: Minimum turning radius
        max_iterations: Maximum RRT iterations
        
    Returns:
        List of (x, y, theta) world coordinates for the full path
    """
    print(f"Starting optimized kinodynamic RRT planning on {occupancy_grid.shape} grid")
    t_start = time.time()
    
    # === STEP 1: Convert world coordinates to grid coordinates ===
    start_grid = start_world
    goal_grid = goal_world
    print(f"Start grid: {start_grid}")
    print(f"Goal grid: {goal_grid}")
    
    # === STEP 2: Build collision lookup ===
    t_lookup = time.time()
    collision_lookup = build_collision_lookup(occupancy_grid, safety_margin, vehicle_width)
    print(f"Built collision lookup in {time.time() - t_lookup:.2f}s")
    
    # === STEP 3: Run Optimized Kinodynamic RRT ===
    t_rrt_start = time.time()
    
    # Initialize planner with optimized parameters
    planner = OptimizedKinodynamicRRT(
        occupancy_grid=occupancy_grid,
        collision_lookup=collision_lookup,
        start=start_grid,
        goal=goal_grid,
        max_iter=max_iterations,
        step_size=step_size * 2.0,
        goal_sample_rate=0.25,
        turning_radius=turning_radius,
        vehicle_length=vehicle_width * 2.5
    )

    ## Testing Summoning RRT implementation
    # planner = BiRRT(start_grid, goal_grid, 1-occupancy_grid, [0,4384,0,4667])
    # grid_path = planner.search()
    
    grid_path = planner.plan(visualize_final=True)
    t_rrt = time.time() - t_rrt_start
    print(f"RRT planning completed in {t_rrt:.2f}s")
    
    if grid_path is None:
        print("Failed to find path")
        return None
    
    # === STEP 4: Convert path to world coordinates ===
    # t_convert = time.time()
    # world_path = []
    # for grid_point in grid_path:
    #     world_point = grid_to_world(grid_point[0], grid_point[1], grid_point[2], metadata)
    #     world_path.append(world_point)
    
    # print(f"Conversion completed in {time.time() - t_convert:.2f}s")
    # print(f"Final path has {len(world_path)} points")
    # print(f"Total planning time: {time.time() - t_start:.2f}s")
    
    return grid_path

def main():
    """Main function for running the planner."""
    parser = argparse.ArgumentParser(description="Optimized Kinodynamic RRT planner")
    parser.add_argument("--test", action="store_true", help="run unit tests and exit")
    parser.add_argument("--vis", action="store_true", help="show visualizations for tests or planning result")
    parser.add_argument("--animate", "-a", action="store_true", help="animate planned path")
    parser.add_argument("--max-iter", type=int, default=20000, help="maximum RRT iterations")
    parser.add_argument("--pad", type=int, default=100, help="crop padding (cells)")
    parser.add_argument("--safety", type=int, default=2, help="safety margin (cells)")
    parser.add_argument("--map-pgm", type=str, default="rrt_occupancy_map.pgm", help="path to PGM map file")
    parser.add_argument("--map-yaml", type=str, default="rrt_occupancy_map.yaml", help="path to YAML metadata file")
    parser.add_argument("--step-size", type=float, default=1.0, help="step size for path sampling")
    parser.add_argument("--turning-radius", type=float, default=1.0, help="minimum turning radius")
    parser.add_argument("--vehicle-width", type=float, default=1.7, help="vehicle width in meters")
    
    if "--vis" not in os.sys.argv and "--animate" not in os.sys.argv and "-a" not in os.sys.argv:
        matplotlib.use("Agg")
    
    args = parser.parse_args()
    
    if args.test:
        run_tests(show=args.vis)
        return
    
    if not (os.path.exists(args.map_pgm) and os.path.exists(args.map_yaml)):
        print(f"Map files not found: {args.map_pgm} and/or {args.map_yaml}")
        print("Use --test for CI runs or provide map files with --map-pgm and --map-yaml")
        return
    
    occupancy_grid, meta = load_pgm_to_occupancy_grid(args.map_pgm, args.map_yaml)
    
    start_w = (-15.0, -35.0, 2*math.pi/2)
    goal_w = (5.0, 45.0, 2*math.pi/2)
    # goal_w = (10.0, -25.0, 0*math.pi/2)
    
    path = optimized_kinodynamic_rrt_planning(
        start_w, goal_w, occupancy_grid, meta,
        safety_margin=args.safety,
        vehicle_width=args.vehicle_width,
        step_size=args.step_size,
        turning_radius=args.turning_radius,
        max_iterations=args.max_iter
    )
    
    if path:
        print(f"Planned path with {len(path)} poses")
        if args.vis:
            visualize_path(occupancy_grid, path, meta, start_w, goal_w)
        if args.animate:
            animate_path(occupancy_grid, path, meta, pad_cells=args.pad)
    else:
        print("Failed to find path")

if __name__ == "__main__":
    main()






import os
from typing import Dict, List

import numpy as np
from GEMstack.onboard.component import Component
from GEMstack.state.agent import AgentState
from GEMstack.state.mission import MissionEnum
from GEMstack.state.all import AllState
from GEMstack.state.mission_plan import MissionPlan
from GEMstack.state.physical_object import ObjectFrameEnum, ObjectPose
from GEMstack.state.route import PlannerEnum, Route
from GEMstack.state.vehicle import VehicleState
from .planner import optimized_kinodynamic_rrt_planning
from .map_utils import load_pgm_to_occupancy_grid
from .rrt_star import RRTStar
from typing import List
from ..component import Component
from ...utils import serialization
from ...state import Route, ObjectFrameEnum
import math
import requests

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from .occupancy_grid2 import OccupancyGrid2
import cv2


ORIGIN_PX = (190, 80)
SCALE_PX_PER_M = 6.5  # px per meter


ORIGIN_PX       = (190, 80)
SCALE_PX_PER_M  = 6.5  # px per meter

class RoutePlanningComponent(Component):
    """Reads a route from disk and returns it as the desired route."""
    def __init__(self):
        print("Route Planning Component init")
        self.planner = None
        self.route = None
        self.bridge = CvBridge()
        self.img_pub = rospy.Publisher("/image_with_car_xy", Image, queue_size=1)
        self.occupancy_grid = OccupancyGrid2()

    def state_inputs(self):
        return ["vehicle", "agents", "mission_plan"]

    def state_outputs(self) -> List[str]:
        return ['route']

    def rate(self):
        return 10.0

    def _car_to_pixel(self, x, y, img_w, img_h):
        # (x,y)[m] → (u,v)[px]
        u0, v0 = ORIGIN_PX
        u = int(round(u0 + SCALE_PX_PER_M * x))
        v = int(round(v0 - SCALE_PX_PER_M * y))

        # clamp to image bounds
        u = max(0, min(u, img_w - 1))
        v = max(0, min(v, img_h - 1))
        return u, v

    def _pixel_to_car(self, u, v, img_w, img_h):
        # clamp to image bounds
        u = max(0, min(u, img_w - 1))
        v = max(0, min(v, img_h - 1))

        # origin and scale (same as in _car_to_pixel)
        u0, v0 = ORIGIN_PX
        scale = SCALE_PX_PER_M

        # invert:
        #   u = u0 + scale * x   →   x = (u - u0) / scale
        #   v = v0 - scale * y   →   y = (v0 - v) / scale
        x = (u - u0) / scale
        y = (v0 - v) / scale

        return x, y

    def visualize_route_pixels(self, route_pts, start_pt, goal_pt):
        """
        route_pts: list of (u, v) in pixels
        start_pt:  (u, v) in pixels
        goal_pt:   (u, v) in pixels
        """
        # 1. Copy the base image
        script_dir = os.path.dirname(os.path.abspath(__file__))
        frame_path  = os.path.join(script_dir, "out.pgm")
        frame = cv2.imread(frame_path, cv2.IMREAD_COLOR)
        img_h, img_w = frame.shape[:2]
        print("frame shape", frame.shape)
        print("frame dtype", frame.dtype)

        # 2. Optionally clamp all points to image bounds
        def clamp(pt):
            u, v = pt
            u = max(0, min(int(round(u)), img_w - 1))
            v = max(0, min(int(round(v)), img_h - 1))
            return (u, v)

        pts = np.array([clamp(p) for p in route_pts], dtype=np.int32).reshape(-1, 1, 2)

        # 3. Draw the route polyline in green
        cv2.polylines(
            frame,
            [pts],
            isClosed=False,
            color=(0, 255, 0),
            thickness=2
        )

        # 4. Draw start marker in blue (star)
        u_s, v_s = clamp(start_pt)
        cv2.drawMarker(
            frame,
            (u_s, v_s),
            color=(255, 0, 0),
            markerType=cv2.MARKER_STAR,
            markerSize=20,
            thickness=3
        )

        # 5. Draw goal marker in red (tilted cross)
        u_g, v_g = clamp(goal_pt)
        cv2.drawMarker(
            frame,
            (u_g, v_g),
            color=(0, 0, 255),
            markerType=cv2.MARKER_TILTED_CROSS,
            markerSize=20,
            thickness=3
        )

        # 6. Publish via ROS
        out = self.bridge.cv2_to_imgmsg(frame, "bgr8")
        print("Publishing out: ", out)
        out.header.stamp = rospy.Time.now()
        self.img_pub.publish(out)

    def update(self, vehicle: VehicleState, agents: Dict[str, AgentState], mission_plan: MissionPlan) -> Route:
        print("agent", agents.items())
        print("vehicle", vehicle.pose.frame)
        print("Converting to GNSS frame", vehicle.pose.to_frame(ObjectFrameEnum.GLOBAL, start_pose_abs = mission_plan.start_vehicle_pose))
        vehicle_global_pose = vehicle.pose.to_frame(ObjectFrameEnum.GLOBAL, start_pose_abs = mission_plan.start_vehicle_pose)
        # for n, a in agents.items():
        #     print("==========================\nAgent:", n, a.pose, a.velocity)
        #     print('==============', a.pose.frame==ObjectFrameEnum.START)
        #     print('==============', a.type==AgentEnum.PEDESTRIAN)
        #     if a.type == AgentEnum.PEDESTRIAN:
        #         print("Pedestrian detected")
        #         # Do something with the pedestrian information

        print("Route Planner's mission:", mission_plan.planner_type.value)
        print("type of mission plan:", type(PlannerEnum.RRT_STAR))
        print("Route Planner's mission:", mission_plan.planner_type.value == PlannerEnum.RRT_STAR.value)
        print("Route Planner's mission:", mission_plan.planner_type.value == PlannerEnum.PARKING.value)
        print("Mission plan:", mission_plan)
        print("Vehicle x:", vehicle.pose.x)
        print("Vehicle y:", vehicle.pose.y)
        print("Vehicle yaw:", vehicle.pose.yaw)

        ## Step. 1 Convert vehicle pose to global frame
        vehicle_global_pose = vehicle.pose.to_frame(ObjectFrameEnum.GLOBAL, start_pose_abs = mission_plan.start_vehicle_pose)
        # self.occupancy_grid.gnss_to_image(vehicle_global_pose.x, vehicle_global_pose.y) # Brijesh check if x corresponds to lon and y corresponds to lat. If not SWAP
        self.occupancy_grid.gnss_to_image(vehicle_global_pose.x, vehicle_global_pose.y)

        ## Step 2: Get start image coordinates aka position of vehicle in image
        start_x, start_y = self.occupancy_grid.gnss_to_image_coords(vehicle_global_pose.x, vehicle_global_pose.y) # Brijesh check if x corresponds to lon and y corresponds to lat. If not SWAP
        start_yaw = vehicle_global_pose.yaw
        print("Start image coordinates", start_x, start_y, "yaw", start_yaw)

        ## Step 3. Convert goal to global frame
        goal_global_pose = mission_plan.goal_vehicle_pose.to_frame(ObjectFrameEnum.GLOBAL, start_pose_abs = mission_plan.start_vehicle_pose)
        goal_x, goal_y = self.occupancy_grid.gnss_to_image_coords(goal_global_pose.x, goal_global_pose.y) # Brijesh check if x corresponds to lon and y corresponds to lat. If not SWAP
        goal_yaw = goal_global_pose.yaw
        print("Goal image coordinates", goal_x, goal_y, "yaw", goal_yaw)
        # self.occupancy_grid[start_x-5:start_x +5][start_y-5] = 1
        if mission_plan.planner_type.value == PlannerEnum.PARKING.value:
            print("I am in PARKING mode")
            # Return a route after doing some processing based on mission plan REMOVE ONCE OTHER PLANNERS ARE IMPLEMENTED
            base_path = os.path.dirname(__file__)
            file_path = os.path.join(base_path, "../../knowledge/routes/forward_15m_extra.csv")

            waypoints = np.loadtxt(file_path, delimiter=',', dtype=float)
            if waypoints.shape[1] == 3:
                waypoints = waypoints[:,:2]
            print("waypoints", waypoints)
            self.route = Route(frame=ObjectFrameEnum.START,points=waypoints.tolist())
        elif mission_plan.planner_type.value == PlannerEnum.RRT_STAR.value:
            print("I am in RRT mode")
            # start = (state.vehicle.pose.x, state.vehicle.pose.y)
            script_dir = os.path.dirname(os.path.abspath(__file__))
            map_path  = os.path.join(script_dir, "highbay_image.pgm")

            print("map_path", map_path)

            map_img = cv2.imread(map_path, cv2.IMREAD_UNCHANGED)
            occupancy_grid = (map_img > 0).astype(np.uint8) #Brijesh check what this does, I assume black is free and white is occupied so this would return white for everything that is not 100% black
            x_bounds = (0,occupancy_grid.shape[1])
            y_bounds = (0,occupancy_grid.shape[0])
            start = (start_x, start_y) # add start_yaw. @Sai I have not integrated yaw into kinodynamic yet. but plls add
            goal = (goal_x, goal_y) # add goal_yaw. @Sai I have not integrated yaw into kinodynamic yet. but plls add
            step_size = 1.0
            max_iter = 2000
            self.planner = RRTStar(start, goal, x_bounds, y_bounds, max_iter=max_iter, step_size=step_size, vehicle_width=1, occupancy_grid=occupancy_grid)
            print("RRT mode")
            rrt_resp = self.planner.plan()
            # self.visualize_route_pixels(rrt_resp, start, goal)
            for i in range(len(rrt_resp)):
                x, y = rrt_resp[i]
                # Convert to car coordinates
                waypoint_lat, waypoint_lon = self.occupancy_grid.image_to_gnss(x, y) # Converts pixel to global frame. Brijesh check again what x corresponds to. Is x lat or is x lon? Change accordingly. Same as above comments
                # Convert global to start frame
                waypoint_start_pose = ObjectPose.from_frame(ObjectFrameEnum.GLOBAL, ObjectFrameEnum.START, waypoint_lat, waypoint_lon, 0.0, start_pose_abs=mission_plan.start_vehicle_pose) #not handling yaw cuz we don't know how to
                rrt_resp[i] = (waypoint_start_pose.x, waypoint_start_pose.y)

            print("Route points in start frame: ", rrt_resp) # Comment this out once you are done debugging
            self.route = Route(frame=ObjectFrameEnum.START, points=rrt_resp)
            # print("Route planning complete")

        else:
            print("Unknown mode")

        return self.route


def is_inside_geofence(x, y, xmin, xmax, ymin, ymax):
    return xmin < x < xmax and ymin < y < ymax


def max_visible_arc(circle_center, radius, geofence):
    xc, yc = circle_center
    (xmin, ymin), (xmax, ymax) = geofence

    angles = np.linspace(0, 2 * np.pi, 500, endpoint=False)
    arc_segments = []
    curr_segment = []

    first_inside = last_inside = False
    flag_full_circle = True
    tangent_min = 0
    min_index = 0

    for i, theta in enumerate(angles):
        x = xc + radius * np.cos(theta)
        y = yc + radius * np.sin(theta)

        inside = is_inside_geofence(x, y, xmin, xmax, ymin, ymax)

        if i == 0:
            first_inside = inside
        if i == len(angles) - 1:
            last_inside = inside

        if inside:
            curr_segment.append((x, y))
            # Calculate the tangent heading in a clockwise direction
            tangent_heading = -np.arctan2(y - yc, x - xc)  # Clockwise heading
            if abs(1 - tangent_heading) > abs(1 - tangent_min):
                if np.arctan2(yc, xc) < np.arctan2(y, x):
                    tangent_min = tangent_heading
                    min_index = i
        else:
            flag_full_circle = False
            if curr_segment:
                arc_segments.append(curr_segment)
                curr_segment = []

    if curr_segment:
        arc_segments.append(curr_segment)

    # If arc wraps around from 2π back to 0, combine first and last segments
    if first_inside and last_inside and len(arc_segments) > 1:
        arc_segments[0] = arc_segments[-1] + arc_segments[0]
        arc_segments.pop()

    if not arc_segments:
        return []

    max_arc = list(max(arc_segments, key=len))
    for i in range(len(max_arc)):
        max_arc[i] = np.array(max_arc[i])
        np.append(max_arc[i], heading_on_circle(xc, yc, max_arc[0][0], max_arc[1][0]))
        # np.append(max_arc[i], 0)

    if flag_full_circle:
        max_arc = max_arc[min_index:] + max_arc[:min_index]

    return max_arc[::-1]


def heading_on_circle(cx, cy, px, py):
    dx = px - cx
    dy = py - cy
    tx = -dy
    ty = dx
    return math.atan2(ty, tx)  # Heading in radians


def check_point_exists(vehicle, start_pose, server_url="http://localhost:8000"):
    print("Vehicle pose frame", vehicle.pose.frame)
    # vehicle_global_pose = vehicle.pose.to_frame(ObjectFrameEnum.GLOBAL, start_frame=vehicle.pose)
    try:
        response = requests.get(f"{server_url}/api/inspect")
        response.raise_for_status()
        points = response.json().get("coords", [])

        if points:
            pt1 = ObjectPose(
                frame=ObjectFrameEnum.GLOBAL,
                t=0,
                x=points[0]["lng"],
                y=points[0]["lat"],
            )
            pt2 = ObjectPose(
                frame=ObjectFrameEnum.GLOBAL,
                t=0,
                x=points[1]["lng"],
                y=points[1]["lat"],
            )
            pt1 = pt1.to_frame(ObjectFrameEnum.START, start_pose_abs=start_pose)
            pt2 = pt2.to_frame(ObjectFrameEnum.START, start_pose_abs=start_pose)
            return True, [[pt1.x, pt1.y], [pt2.x, pt2.y]]
        return False, []

    except requests.exceptions.RequestException as e:
        print("Error contacting server:", e)
        return False, []


class InspectRoutePlanner(Component):
    """Reads a route from disk and returns it as the desired route."""

    def __init__(self, state_machine, frame: str = "start"):
        self.geofence_area = [[0, 0], [20, 20]]
        self.state_list = state_machine
        self.index = 1
        self.mission = self.state_list[self.index]
        self.circle_center = [10, 17.5]
        self.radius = 5
        self.inspection_route = max_visible_arc(
            self.circle_center, self.radius, self.geofence_area
        )
        self.start = [0, 0]
        self.bridge = CvBridge()
        self.img_pub = rospy.Publisher("/image_with_car_xy", Image, queue_size=1)
        self.occupancy_grid = OccupancyGrid2()
        self.planned_path_already = False
        self.x = None

    def state_inputs(self):
        return ["all"]

    def state_outputs(self) -> List[str]:
        return ["route", "mission"]

    def rate(self):
        return 2.0

    def _car_to_pixel(self, x, y, img_w, img_h):
        # (x,y)[m] → (u,v)[px]
        u0, v0 = ORIGIN_PX
        u = int(round(u0 + SCALE_PX_PER_M * x))
        v = int(round(v0 - SCALE_PX_PER_M * y))

        # clamp to image bounds
        u = max(0, min(u, img_w - 1))
        v = max(0, min(v, img_h - 1))
        return u, v

    def _pixel_to_car(self, u, v, img_w, img_h):
        # clamp to image bounds
        u = max(0, min(u, img_w - 1))
        v = max(0, min(v, img_h - 1))

        # origin and scale (same as in _car_to_pixel)
        u0, v0 = ORIGIN_PX
        scale = SCALE_PX_PER_M

        # invert:
        #   u = u0 + scale * x   →   x = (u - u0) / scale
        #   v = v0 - scale * y   →   y = (v0 - v) / scale
        x = (u - u0) / scale
        y = (v0 - v) / scale

        return x, y

    def visualize_route_pixels(self, route_pts, start_pt, goal_pt):
        """
        route_pts: list of (u, v) in pixels
        start_pt:  (u, v) in pixels
        goal_pt:   (u, v) in pixels
        """
        # 1. Copy the base image
        script_dir = os.path.dirname(os.path.abspath(__file__))
        frame_path = os.path.join(script_dir, "out.pgm")
        frame = cv2.imread(frame_path, cv2.IMREAD_COLOR)
        img_h, img_w = frame.shape[:2]
        # print("frame shape", frame.shape)
        # print("frame dtype", frame.dtype)

        # 2. Optionally clamp all points to image bounds
        def clamp(pt):
            u, v = pt
            u = max(0, min(int(round(u)), img_w - 1))
            v = max(0, min(int(round(v)), img_h - 1))
            return (u, v)

        pts = np.array([clamp(p) for p in route_pts], dtype=np.int32).reshape(-1, 1, 2)

        # 3. Draw the route polyline in green
        cv2.polylines(frame, [pts], isClosed=False, color=(0, 255, 0), thickness=2)

        # 4. Draw start marker in blue (star)
        u_s, v_s = clamp(start_pt)
        cv2.drawMarker(
            frame,
            (u_s, v_s),
            color=(255, 0, 0),
            markerType=cv2.MARKER_STAR,
            markerSize=20,
            thickness=3,
        )

        # 5. Draw goal marker in red (tilted cross)
        u_g, v_g = clamp(goal_pt)
        cv2.drawMarker(
            frame,
            (u_g, v_g),
            color=(0, 0, 255),
            markerType=cv2.MARKER_TILTED_CROSS,
            markerSize=20,
            thickness=3,
        )

        # 6. Publish via ROS
        out = self.bridge.cv2_to_imgmsg(frame, "bgr8")
        # print("Publishing out: ", out)
        out.header.stamp = rospy.Time.now()
        self.img_pub.publish(out)

    def rrt_route(self, state, goal_start_pose):
        ## Step. 1 Convert vehicle pose to global frame
        vehicle_global_pose = state.vehicle.pose.to_frame(ObjectFrameEnum.GLOBAL, start_pose_abs = state.start_vehicle_pose)
        self.occupancy_grid.gnss_to_image(vehicle_global_pose.x, vehicle_global_pose.y) # Brijesh check if x corresponds to lon and y corresponds to lat. If not SWAP
        # self.occupancy_grid.gnss_to_image(-88.236365, 40.092722)

        ## Step 2: Get start image coordinates aka position of vehicle in image
        start_x, start_y = self.occupancy_grid.gnss_to_image_coords(vehicle_global_pose.x, vehicle_global_pose.y) # Brijesh check if x corresponds to lon and y corresponds to lat. If not SWAP
        # start_x, start_y = self.occupancy_grid.gnss_to_image_coords(40.092722, -88.236365)
        start_yaw = vehicle_global_pose.yaw
        print("Start image coordinates", start_x, start_y, "yaw", start_yaw)

        ## Step 3. Convert goal to global frame
        goal_global_pose = goal_start_pose.to_frame(ObjectFrameEnum.GLOBAL, start_pose_abs = state.start_vehicle_pose)
        goal_x, goal_y = self.occupancy_grid.gnss_to_image_coords(goal_global_pose.x, goal_global_pose.y) # Brijesh check if x corresponds to lon and y corresponds to lat. If not SWAP
        # goal_x, goal_y = self.occupancy_grid.gnss_to_image_coords(40.092889, -88.235686)
        goal_yaw = goal_global_pose.yaw
        print("Goal image coordinates", goal_x, goal_y, "yaw", goal_yaw)

        script_dir = os.path.dirname(os.path.abspath(__file__))
        map_path  = os.path.join(script_dir, "highbay_image.pgm")

        print("map_path", map_path)

        map_img = cv2.imread(map_path, cv2.IMREAD_UNCHANGED)
        occupancy_grid = (map_img > 0).astype(np.uint8) #Brijesh check what this does, I assume black is free and white is occupied so this would return white for everything that is not 100% black
        # x_b
        self.t_last = Noneounds = (0,occupancy_grid.shape[1])
        # y_bounds = (0,occupancy_grid.shape[0])
        # start = (start_x, start_y) # add start_yaw. @Sai I have not integrated yaw into kinodynamic yet. but plls add
        # goal = (goal_x, goal_y) # add goal_yaw. @Sai I have not integrated yaw into kinodynamic yet. but plls add
        # step_size = 1.0
        # max_iter = 2000

        script_dir = os.path.dirname(os.path.abspath(__file__))
        map_path  = os.path.join(script_dir, "highbay_image.pgm")
        
        # occupancy_grid = load_pgm_to_occupancy_grid(map_path)
        start_w = [start_y, start_x, start_yaw]
        goal_w = [goal_y, goal_x, goal_yaw]
        # occupancy_grid[start_x-5:start_x +5][start_y-5] = 1

        path = optimized_kinodynamic_rrt_planning(
            start_w, goal_w, occupancy_grid
        )
        
        
        
        print("RRT mode", path)
        # rrt_resp = self.planner.plan()
        # self.visualize_route_pixels(rrt_resp, start, goal)
        waypoints = []
        for i in range(len(path)):
            x, y, theta = path[i]
            # Convert to car coordinates
            waypoint_lat, waypoint_lon = self.occupancy_grid.image_to_gnss(y, x) # Converts pixel to global frame. Brijesh check again what x corresponds to. Is x lat or is x lon? Change accordingly. Same as above comments
            # Convert global to start frame
            waypoint_global_pose = ObjectPose(
                frame=ObjectFrameEnum.GLOBAL,
                t=state.start_vehicle_pose.t,
                x=waypoint_lon,
                y=waypoint_lat,
                yaw=theta,
            )
            waypoint_start_pose = waypoint_global_pose.to_frame(ObjectFrameEnum.START, start_pose_abs=state.start_vehicle_pose) #not handling yaw cuz we don't know how to
            waypoints.append((waypoint_start_pose.x, waypoint_start_pose.y))

        print("Route points in start frame: ", waypoints) # Comment this out once you are done debugging
        return Route(frame=ObjectFrameEnum.START, points=waypoints)

    def update(self, state):
        self.flag = 0
        self.route = Route(frame=ObjectFrameEnum.START, points=((0, 0, 0)))
        print("Mode ", state.mission.type)
        print("Mission state:", self.mission)
        if self.mission == "IDLE":
            state.mission.type = MissionEnum.IDLE
            points_found, pts = check_point_exists(
                state.vehicle, state.start_vehicle_pose
            )
            if points_found:
                self.inspection_area = pts
                print("Inspection coordinates:", self.inspection_area)
                print(self.state_list[self.index + 1])
                self.mission = self.state_list[self.index + 1]
                self.index += 1
                print("CHANGING STATES", self.mission)
                self.start = [state.vehicle.pose.x, state.vehicle.pose.y]
                self.circle_center = [
                    (self.inspection_area[0][0] + self.inspection_area[1][0]) / 2,
                    (self.inspection_area[0][1] + self.inspection_area[1][1]) / 2,
                ]
                self.radius = (
                    (self.inspection_area[0][0] + self.inspection_area[1][0]) ** 2
                    + (self.inspection_area[0][1] + self.inspection_area[1][1]) ** 2
                ) ** 0.5 / 2
                self.inspection_route = max_visible_arc(
                    self.circle_center, self.radius, self.geofence_area
                )
        elif self.mission == "NAV":
            state.mission.type = MissionEnum.DRIVE
            
            print("I MMMMMMMMMMMMM HEEEEERRRRRRRRRRRRRRReeeeeeeeeeeee")
            
            start = (state.vehicle.pose.x, state.vehicle.pose.y)
            goal = ObjectPose(
                frame=ObjectFrameEnum.START,
                t=state.start_vehicle_pose.t,
                x=state.vehicle.pose.x + 15,
                y=state.vehicle.pose.y,
                yaw=0.0,
            )
            print("Current Position: ", start)
            print("Goal Position: ", goal)
            if abs(state.vehicle.pose.x - goal.x) <= 1 and abs(state.vehicle.pose.y - goal.y) <= 1:
                print(self.state_list[self.index + 1])
                self.mission = self.state_list[self.index + 1]
                self.index += 1
                print("CHANGING STATES", self.mission)
            
            if self.planned_path_already == False:
                self.x = self.rrt_route(state, goal)
                    # self.route = self.rrt_route(state, goal)
                self.planned_path_already = True
            # self.route = Route(
            #     frame=ObjectFrameEnum.START, points=self.route
            # )
            self.route = self.x
            
                

        elif self.mission == "INSPECT":
            state.mission.type = MissionEnum.INSPECT
            start = (state.vehicle.pose.x + 1, state.vehicle.pose.y + 1)
            goal = (self.inspection_route[-1][0], self.inspection_route[-1][1])
            # if abs(start[0] - goal[0]) <= 1 and abs(start[1] - goal[1]) <= 1: # and self.flag>1:
            #     print(self.state_list[self.index + 1])
            #     self.mission = self.state_list[self.index + 1]
            #     self.index += 1
            #     print("CHANGING STATES", self.mission)
            self.flag += 0.1
            self.route = Route(
                frame=ObjectFrameEnum.START, points=self.inspection_route
            )
        elif self.mission == "FINISH":
            state.mission.type = MissionEnum.INSPECT_UPLOAD
            goal = state.start_vehicle_pose

            if abs(state.vehicle.pose.x - goal.x) <= 1 and abs(state.vehicle.pose.y - goal.y) <= 1:
                print(self.state_list[self.index + 1])
                self.mission = self.state_list[self.index + 1]
                self.index += 1
                print("CHANGING STATES", self.mission)

            self.route = self.rrt_route(state, goal)

        print('-------------------------------------------------')
        print(self.route)
        return [self.route, state.mission]






import math
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from .map_utils import world_to_grid

def visualize_path(occupancy_grid, path, metadata, start_world, goal_world, show_headings=True):
    """
    Visualize the planned path.
    
    Args:
        occupancy_grid: Binary occupancy grid (1=obstacle, 0=free)
        path: List of (x, y, theta) world coordinates
        metadata: Map metadata
        start_world: (x, y, theta) start position in world coordinates
        goal_world: (x, y, theta) goal position in world coordinates
        show_headings: Whether to show heading arrows along the path
    """
    plt.figure(figsize=(12, 12))
    
    # Show occupancy grid
    plt.imshow(occupancy_grid, origin='lower', cmap='gray' )
    
    # Extract path points
    if path:
        grid_path = []
        for world_pt in path:
            grid_pt = world_to_grid(*world_pt, metadata)
            grid_path.append(grid_pt)
            
        xs = [p[0] for p in grid_path]
        ys = [p[1] for p in grid_path]
        
        # Plot path
        plt.plot(xs, ys, 'g-', linewidth=2)
        
        # Show heading arrows
        if show_headings:
            arrow_interval = max(1, len(path) // 20)  # Show ~20 arrows along path
            arrow_length = 10
            
            for i in range(0, len(grid_path), arrow_interval):
                x, y, theta = grid_path[i]
                dx = arrow_length * math.cos(theta)
                dy = arrow_length * math.sin(theta)
                plt.arrow(x, y, dx, dy, head_width=3, head_length=6, fc='blue', ec='blue')
    
    # Show start and goal
    start_grid = world_to_grid(*start_world, metadata)
    goal_grid = world_to_grid(*goal_world, metadata)
    
    plt.scatter(start_grid[0], start_grid[1], color='green', s=100, marker='o', label='Start')
    plt.scatter(goal_grid[0], goal_grid[1], color='red', s=100, marker='x', label='Goal')
    
    # Draw start and goal heading arrows
    dx_start = 15 * math.cos(start_grid[2])
    dy_start = 15 * math.sin(start_grid[2])
    plt.arrow(start_grid[0], start_grid[1], dx_start, dy_start, 
            head_width=5, head_length=10, fc='green', ec='green')
    
    dx_goal = 15 * math.cos(goal_grid[2])
    dy_goal = 15 * math.sin(goal_grid[2])
    plt.arrow(goal_grid[0], goal_grid[1], dx_goal, dy_goal, 
            head_width=5, head_length=10, fc='red', ec='red')
    
    plt.title('Path Planning Result')
    plt.legend()
    plt.tight_layout()
    plt.show()

def animate_path(occupancy_grid, path, metadata, interval=60, pad_cells=20,
                save=None, vehicle_len=10):
    """
    Animate the drive and crop axes to the path region.
    
    Args:
        occupancy_grid: Binary occupancy grid (1=obstacle, 0=free)
        path: List of (x, y, theta) world coordinates
        metadata: Map metadata
        interval: Animation interval in milliseconds
        pad_cells: Extra cells to pad around trajectory bbox
        save: If provided, save animation to this file
        vehicle_len: Visual length of car icon (cells)
    
    Returns:
        Animation object
    """
    def world_to_grid(x, y, th, meta):
        ox, oy, _ = meta['origin']; res = meta['resolution']
        return ((x-ox)/res, (y-oy)/res, th)

    gpath = [world_to_grid(*pt, metadata) for pt in path]
    xs = [p[0] for p in gpath]; ys = [p[1] for p in gpath]

    xmin, xmax = min(xs)-pad_cells, max(xs)+pad_cells
    ymin, ymax = min(ys)-pad_cells, max(ys)+pad_cells

    fig, ax = plt.subplots(figsize=((xmax-xmin)/50, (ymax-ymin)/50))
    ax.imshow(occupancy_grid.T, origin='lower', cmap='gray')
    ax.set_xlim(xmin, xmax); ax.set_ylim(ymin, ymax)
    ax.set_aspect('equal'); ax.set_xticks([]); ax.set_yticks([])

    # artists
    traj_line, = ax.plot([], [], 'g-', lw=2)
    car_line,  = ax.plot([], [], color='red', lw=3)

    def init():
        traj_line.set_data([], []); car_line.set_data([], [])
        return traj_line, car_line

    def update(i):
        traj_line.set_data(xs[:i+1], ys[:i+1])
        x, y, h = gpath[i]
        rear_x = x - vehicle_len*0.4*math.cos(h)
        rear_y = y - vehicle_len*0.4*math.sin(h)
        front_x = x + vehicle_len*0.6*math.cos(h)
        front_y = y + vehicle_len*0.6*math.sin(h)
        car_line.set_data([rear_x, front_x], [rear_y, front_y])
        return traj_line, car_line

    ani = animation.FuncAnimation(fig, update, frames=len(gpath), init_func=init,
                                interval=interval, blit=True, repeat=False)
    if save:  
        ani.save(save, dpi=150, fps=1000//interval)
    else:     
        plt.show()
    return ani
'''
