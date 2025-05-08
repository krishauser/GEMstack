import math
import random
import time
import numpy as np
import matplotlib.pyplot as plt
from utils import normalize_angle # Assuming utils.py contains normalize_angle
from collision import fast_collision_check # Assuming collision.py contains fast_collision_check
import scipy.spatial

class RRTNode:
    def __init__(self, x, y, theta, v=0.0, phi=0.0, cost=0.0, parent=None):
        """Node in the RRT tree with position, orientation, velocity and steering."""
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

class OptimizedKinodynamicRRT:
    def __init__(self, occupancy_grid, collision_lookup, metadata, start, goal,
                 max_iter=3000, step_size=2.0, goal_sample_rate=0.25):
        """
        Simplified Kinodynamic RRT planner for autonomous vehicles.
        """
        self.grid = occupancy_grid
        self.collision_lookup = collision_lookup
        self.metadata = metadata
        
        self.start = RRTNode(start[0], start[1], start[2])
        self.goal = RRTNode(goal[0], goal[1], goal[2])
        
        self.max_iter = max_iter
        self.step_size = step_size # Target extension length for controls
        self.goal_sample_rate = goal_sample_rate
        self.vehicle_length = 4.0
        self.dt = 0.1 # Time step for simulation
        
        self.nodes = [self.start]
        self.kdtree = None
        self.kdtree_rebuild_freq = 50
        
        self.goal_found = False
        self.goal_node = None
        # Thresholds for considering goal reached
        self.goal_position_threshold = 2.0 # Original: 8.0. Consider tightening (e.g., 1.5-2.0m) for precise docking.
        self.goal_angle_threshold = math.radians(15.0) # Original: 15 deg. Consider tightening (e.g., 5-10 deg).
        
        self.max_steering_angle = math.pi / 2
        self.max_velocity = 8.0
        self.min_velocity = 1.0 
        
        self.grid_dims = occupancy_grid.shape
        self.time_budget = 1
        
        self.primitives = self._initialize_motion_primitives()
        
        self.steering_gain = 0.8
        self.smoothness_weight = 3.0 
        # NEW: Weight for penalizing heading error when targeting the goal state
        self.goal_heading_weight = 25.0 # Tune this: Higher values prioritize goal heading more. Start with 20-50.

    def _initialize_motion_primitives(self):
        primitives = []
        velocities = [self.max_velocity * 0.5, self.max_velocity * 0.8]
        steering_range = self.max_steering_angle * 0.8
        steering_angles = np.linspace(-steering_range, steering_range, 5) # Includes 0
        
        for v in velocities:
            for phi in steering_angles:
                primitive = self._generate_primitive(v, phi)
                if primitive:
                    primitives.append(primitive)
        
        straight = self._generate_primitive(self.max_velocity * 0.8, 0.0, steps=20)
        if straight:
            primitives.append(straight)
            primitives.append(straight) # Add twice for higher probability
            
        print(f"Initialized {len(primitives)} motion primitives")
        return primitives
        
    def _generate_primitive(self, velocity, steering_angle, steps=15):
        x, y, theta = 0.0, 0.0, 0.0
        primitive = [(x, y, theta, velocity, steering_angle)]
        
        for _ in range(steps):
            actual_steering = steering_angle * (0.7 if velocity > self.max_velocity*0.7 else 1.0)
            
            x_next = x + velocity * math.cos(theta) * self.dt
            y_next = y + velocity * math.sin(theta) * self.dt
            theta_next = normalize_angle(theta + (velocity * math.tan(actual_steering) / 
                                                  self.vehicle_length) * self.dt)
            
            x, y, theta = x_next, y_next, theta_next
            primitive.append((x, y, theta, velocity, actual_steering))
            
        return primitive

    def _build_kdtree(self):
        positions = np.array([(node.x, node.y) for node in self.nodes])
        self.kdtree = scipy.spatial.cKDTree(positions)
    
    def _nearest_node(self, position):
        if self.kdtree is None or len(self.nodes) % self.kdtree_rebuild_freq == 0:
            self._build_kdtree()
            
        _, idx = self.kdtree.query([position[0], position[1]])
        return self.nodes[idx]
    
    def _sample_position(self):
        if random.random() < self.goal_sample_rate:
            return self.goal.get_position() # Returns (x,y) tuple
            
        while True:
            x = random.uniform(0, self.grid_dims[0])
            y = random.uniform(0, self.grid_dims[1])
            
            if not fast_collision_check(y, x, self.collision_lookup): # Note: fast_collision_check(y,x,...)
                return (x, y)

    # MODIFIED: To handle target_state (x, y, optional_theta)
    def _generate_new_node(self, from_node, sampled_position):
        """Generate new node using optimal controls to reach target position/state."""
        target_state_for_steering = None
        is_targeting_final_goal_position = (abs(sampled_position[0] - self.goal.x) < 1e-3 and \
                                            abs(sampled_position[1] - self.goal.y) < 1e-3)

        if is_targeting_final_goal_position:
            target_state_for_steering = (self.goal.x, self.goal.y, self.goal.theta)
        else:
            target_state_for_steering = (sampled_position[0], sampled_position[1], None)

        # Try motion primitives first
        _, end_node_primitive = self._select_best_primitive(from_node, target_state_for_steering)
        
        if end_node_primitive is not None:
            return end_node_primitive
            
        # Fall back to direct PFF controller
        return self._apply_controls(from_node, target_state_for_steering)
    
    # MODIFIED: To accept target_state and use goal_heading_weight
    def _select_best_primitive(self, from_node, target_state):
        """Select best motion primitive considering distance, smoothness, and goal heading."""
        best_score = float('inf')
        best_primitive_transformed = None
        best_end_node = None
        
        x0, y0, theta0_parent, _, phi0_parent = from_node.get_state()
        target_x, target_y, target_heading_opt = target_state
        
        is_final_goal_with_heading = (target_heading_opt is not None and \
                                      abs(target_x - self.goal.x) < 1e-3 and \
                                      abs(target_y - self.goal.y) < 1e-3)

        for primitive_model in self.primitives: # primitive_model is relative to (0,0,0)
            transformed_primitive_path = self._transform_primitive(primitive_model, from_node)
            end_state_of_primitive = transformed_primitive_path[-1] # (x, y, theta, v, phi) in global frame
            
            distance_moved = math.hypot(end_state_of_primitive[0] - x0, end_state_of_primitive[1] - y0)
            # Ensure primitive makes some progress
            if distance_moved < self.step_size * 0.1: # Reduced min progress slightly
                continue
                
            # Cost 1: Distance from primitive's end to target_x, target_y (sampled point or goal position)
            distance_to_target_xy = math.hypot(
                end_state_of_primitive[0] - target_x,
                end_state_of_primitive[1] - target_y
            )
            
            # Cost 2: Smoothness (steering effort/change)
            # Using the steering angle of the primitive itself as a measure of effort
            primitive_steering_angle = primitive_model[0][4] # Steering angle of this base primitive
            # Optional: consider change from parent: abs(primitive_steering_angle - phi0_parent)
            smoothness_cost = abs(primitive_steering_angle)

            score = distance_to_target_xy + self.smoothness_weight * smoothness_cost

            # Cost 3: Heading error if targeting the final goal state with a specific heading
            if is_final_goal_with_heading:
                angle_diff_to_goal_orientation = abs(normalize_angle(end_state_of_primitive[2] - target_heading_opt))
                score += self.goal_heading_weight * angle_diff_to_goal_orientation
            
            if score < best_score:
                collision = False
                for state_in_path in transformed_primitive_path: # Check each point in the transformed path
                    if fast_collision_check(state_in_path[1], state_in_path[0], self.collision_lookup): # Check (y,x)
                        collision = True
                        break
                
                if not collision:
                    best_score = score
                    # best_primitive_transformed = transformed_primitive_path # This is the path segment
                    
                    best_end_node = RRTNode(
                        end_state_of_primitive[0], end_state_of_primitive[1], end_state_of_primitive[2],
                        end_state_of_primitive[3], end_state_of_primitive[4], # v, phi from end_state_of_primitive
                        cost=from_node.cost + len(primitive_model) * self.dt, # Cost based on primitive duration
                        parent=from_node
                    )
                    best_end_node.path_from_parent = transformed_primitive_path
        
        return None, best_end_node # Return (path_segment, node) - path segment not strictly needed by caller if node has it

    # MODIFIED: To accept target_state (but PFF mainly uses x,y part)
    def _apply_controls(self, from_node, target_state):
        """Apply PFF controller to generate trajectory toward target x,y."""
        x, y, theta, v, phi = from_node.get_state()
        target_x, target_y, _ = target_state # PFF primarily aims for position (x,y)
        
        dx = target_x - x
        dy = target_y - y
        dist_to_target_xy = math.hypot(dx, dy)

        # Adjusted minimum progress for PFF to be smaller than primitive, to allow finer moves if needed
        if dist_to_target_xy < self.step_size * 0.1: 
            return None
            
        target_angle_to_xy = math.atan2(dy, dx)
        heading_error = normalize_angle(target_angle_to_xy - theta)
        
        alignment = max(0.3, math.cos(heading_error))
        # Velocity profile: scale by distance to target, up to max_velocity
        velocity_scale = min(1.0, dist_to_target_xy / (3.0 * self.step_size)) # Reach max_vel if target is ~3 steps away
        velocity = self.max_velocity * velocity_scale
        velocity = max(self.min_velocity, velocity * alignment) 
        velocity = min(velocity, self.max_velocity) # Ensure it doesn't exceed max_velocity

        steering = self.steering_gain * heading_error
        max_allowed_steering = self.max_steering_angle * 0.8 # More conservative steering
        steering = max(-max_allowed_steering, min(max_allowed_steering, steering))
        
        # Determine number of steps for PFF extension
        # Aim to cover roughly self.step_size or until target, bounded
        if velocity < 1e-2: # Avoid division by zero or excessively many steps
            num_steps = 20
        else:
            # Steps to cover distance, or target self.step_size, or max 20
            steps_to_cover_dist = int(dist_to_target_xy / (velocity * self.dt))
            steps_for_step_size = int(self.step_size / (velocity * self.dt))
            num_steps = min(max(10, steps_for_step_size), steps_to_cover_dist, 20) 
            num_steps = max(1, num_steps) # Ensure at least one step

        path = [(x, y, theta, v, phi)] # Initial state of the segment
        current_x, current_y, current_theta = x, y, theta

        for _ in range(num_steps):
            x_next = current_x + velocity * math.cos(current_theta) * self.dt
            y_next = current_y + velocity * math.sin(current_theta) * self.dt
            theta_next = normalize_angle(current_theta + (velocity * math.tan(steering) / 
                                                          self.vehicle_length) * self.dt)
            
            # Collision check for each PFF step
            if fast_collision_check(y_next, x_next, self.collision_lookup): # Check (y,x)
                return None # Collision along PFF path segment
                
            current_x, current_y, current_theta = x_next, y_next, theta_next
            path.append((current_x, current_y, current_theta, velocity, steering))
        
        new_node = RRTNode(current_x, current_y, current_theta, velocity, steering,
                           cost=from_node.cost + num_steps * self.dt, 
                           parent=from_node)
        new_node.path_from_parent = path
        
        return new_node

    def plan(self, visualize_final=True):
        self.start_time = time.time()
        
        for i in range(self.max_iter):
            if time.time() - self.start_time > self.time_budget:
                print("Time budget exceeded.")
                break
            
            if (i+1) % 500 == 0:
                print(f"Iteration {i+1}, nodes: {len(self.nodes)}")
                
            rand_sampled_pos = self._sample_position() # This is an (x,y) tuple
            nearest = self._nearest_node(rand_sampled_pos)
            newly_generated_node = self._generate_new_node(nearest, rand_sampled_pos) # Pass (x,y) sample
            
            if newly_generated_node is None:
                continue
            
            self.nodes.append(newly_generated_node)
            
            if self._is_goal_reached(newly_generated_node):
                self.goal_found = True
                self.goal_node = newly_generated_node # This node met the criteria
                print(f"Path found at iteration {i+1}")
                break
        
        if self.goal_found:
            path = self._extract_path()
            if visualize_final:
                self._visualize_result(path)
            return path
        else:
            print("No path found within iterations/time.")
            if visualize_final:
                self._visualize_result(None) # Visualize tree even if no path
            return None

    # MODIFIED: Stricter goal condition
    def _is_goal_reached(self, node):
        """Check if node has reached goal satisfying both position and heading."""
        dist_to_goal_pos = math.hypot(node.x - self.goal.x, node.y - self.goal.y)
        angle_diff_to_goal_theta = abs(normalize_angle(node.theta - self.goal.theta))
        
        # For goal to be reached, both position and orientation must be within their respective thresholds.
        if dist_to_goal_pos <= self.goal_position_threshold and \
           angle_diff_to_goal_theta <= self.goal_angle_threshold:
            return True
        return False
        
    def _extract_path(self):
        if not self.goal_found or self.goal_node is None:
            return None
            
        path = []
        current_node = self.goal_node
        
        while current_node is not None:
            if current_node.path_from_parent: # If it has a path segment leading to it
                # Insert points from this segment in correct order (from parent to current_node)
                # The path_from_parent should already be in forward order.
                # We are backtracking, so add to the front of the main path.
                for point in reversed(current_node.path_from_parent):
                    path.insert(0, (point[0], point[1], point[2])) # x, y, theta
            elif current_node == self.start: # For the very start node, add its state
                 path.insert(0, (current_node.x, current_node.y, current_node.theta))
            
            current_node = current_node.parent
        
        # Remove consecutive duplicates that arise from path segments starting with the parent's state
        if not path: return []
        
        deduplicated_path = [path[0]]
        for i in range(1, len(path)):
            # Compare (x,y,theta) for duplication
            if not (abs(path[i][0] - path[i-1][0]) < 1e-3 and \
                    abs(path[i][1] - path[i-1][1]) < 1e-3 and \
                    abs(normalize_angle(path[i][2] - path[i-1][2])) < 1e-3) :
                deduplicated_path.append(path[i])
        
        return deduplicated_path


    def _transform_primitive(self, primitive, from_node):
        x0, y0, theta0 = from_node.x, from_node.y, from_node.theta
        
        transformed = []
        
        path_segment = []

        for rel_x, rel_y, rel_theta, rel_v, rel_phi in primitive:
            # Rotate relative position
            rotated_x = rel_x * math.cos(theta0) - rel_y * math.sin(theta0)
            rotated_y = rel_x * math.sin(theta0) + rel_y * math.cos(theta0)
            
            # Translate to global frame
            global_x = rotated_x + x0
            global_y = rotated_y + y0
            global_theta = normalize_angle(rel_theta + theta0)
            
            # v and phi are typically absolute controls or resulting velocities, not needing transformation
            transformed.append((global_x, global_y, global_theta, rel_v, rel_phi)) 
            
        return transformed
    
    def _visualize_result(self, path):
        plt.figure(figsize=(12, 12)) # Increased size slightly
        plt.imshow(self.grid, origin='lower', cmap='gray', extent=[0, self.grid_dims[1], 0, self.grid_dims[0]]) # Match grid dims if not square
        
        # Plot tree edges (subset for efficiency)
        # subsample = max(1, len(self.nodes) // 200) # Original
        for node in self.nodes: # Plot all for better visualization if not too many
            if node.parent and node.path_from_parent and len(node.path_from_parent) > 1:
                xs = [p[0] for p in node.path_from_parent]
                ys = [p[1] for p in node.path_from_parent]
                plt.plot(xs, ys, color='skyblue', alpha=0.4, linewidth=0.8) # Lighter color for tree
        
        plt.scatter(self.start.x, self.start.y, color='lime', s=150, marker='o', edgecolor='black', label='Start')
        plt.arrow(self.start.x, self.start.y, 
                  2.0 * math.cos(self.start.theta), 2.0 * math.sin(self.start.theta), # Length of arrow: 2 units
                  color='lime', head_width=0.8, head_length=1.0, edgecolor='black')

        plt.scatter(self.goal.x, self.goal.y, color='red', s=150, marker='X', edgecolor='black', label='Goal')
        plt.arrow(self.goal.x, self.goal.y, 
                  2.0 * math.cos(self.goal.theta), 2.0 * math.sin(self.goal.theta), 
                  color='red', head_width=0.8, head_length=1.0, edgecolor='black')
        
        if path and len(path) > 1:
            xs = [p[0] for p in path]
            ys = [p[1] for p in path]
            plt.plot(xs, ys, color='green', linestyle='-', linewidth=2.5, label='Path')
            # Plot heading indicators along the path
            for i in range(0, len(path), max(1, len(path)//20)): # ~20 arrows on path
                 plt.arrow(path[i][0], path[i][1],
                           1.0 * math.cos(path[i][2]), 1.0 * math.sin(path[i][2]),
                           color='darkgreen', head_width=0.4, head_length=0.6, alpha=0.7)

        plt.title("Optimized Kinodynamic RRT Path Planning")
        plt.xlabel("X-position (m)")
        plt.ylabel("Y-position (m)")
        plt.legend()
        plt.axis('equal') # Ensure aspect ratio is maintained
        plt.grid(True, linestyle='--', alpha=0.5)
        plt.tight_layout()
        plt.show()