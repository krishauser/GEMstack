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
        #Save figure with timestamp
        plt.savefig(f"rrt_tree_{time.time()}.png") # SANJAY
        # plt.show()
    
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
        