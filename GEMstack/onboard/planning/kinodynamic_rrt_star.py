import math
import random
import time
import numpy as np
import matplotlib.pyplot as plt
import scipy.spatial

from scipy.interpolate import splprep, splev
from .collision import fast_collision_check

def normalize_angle(angle):
    while angle > math.pi: angle -= 2.0 * math.pi
    while angle < -math.pi: angle += 2.0 * math.pi
    return angle

class RRTNode:
    """Represents a node in the RRT tree."""
    def __init__(self, x, y, theta, velocity=0.0, phi_steering=0.0, cost=0.0, parent=None):
        self.x = x
        self.y = y
        self.theta = theta  # Orientation in radians
        self.velocity = velocity
        self.phi_steering = phi_steering # Steering angle
        self.cost = cost  # Cost from the start node to this node
        self.parent = parent  # Parent node
        self.path_from_parent = []  # List of states (x, y, theta, v, phi) forming the path from the parent

    def get_state(self):
        """Returns the full state of the node."""
        return (self.x, self.y, self.theta, self.velocity, self.phi_steering)

    def get_position(self):
        """Returns the (x, y) position of the node."""
        return (self.x, self.y)

class OptimizedKinodynamicRRT:
    """
    Implements an optimized Bidirectional Kinodynamic RRT* algorithm
    for path planning with vehicle dynamics.
    """
    def __init__(self,
                 occupancy_grid,
                 collision_lookup_table,
                 start_pose_tuple, # (x, y, theta)
                 goal_pose_tuple,  # (x, y, theta)
                 max_iterations=100000,
                 local_sampling_step_size=1.0,
                 vehicle_width=20.0,
                 vehicle_length=45.0,
                 goal_biasing_rate=0.5):

        self.occupancy_grid = occupancy_grid
        self.collision_lookup = collision_lookup_table

        self.start_node = RRTNode(*start_pose_tuple, cost=0.0)
        self.true_goal_configuration = RRTNode(*goal_pose_tuple) # Goal in world frame
        # Goal node for the goal tree (reversed orientation for backward expansion)
        self.goal_tree_root_node = RRTNode(
            goal_pose_tuple[0], goal_pose_tuple[1],
            self._calculate_inverse_angle(goal_pose_tuple[2]), cost=0.0
        )

        self.max_iterations = max_iterations
        self.visualization_sampling_step_size = local_sampling_step_size
        self.vehicle_length = vehicle_length  # Vehicle wheelbase
        self.vehicle_width = vehicle_width  # Vehicle width
        self.time_step_simulation = 0.1 # dt for simulation
        self.goal_biasing_rate = goal_biasing_rate # Chance to sample goal directly

        self.nodes_in_start_tree = [self.start_node]
        self.nodes_in_goal_tree = [self.goal_tree_root_node]
        self.kdtree_for_start_tree = None
        self.kdtree_for_goal_tree = None
        self.kdtree_rebuild_frequency = 50 # Rebuild KD-tree every N nodes added

        self.is_path_found = False
        self.tree_connection_nodes = None # (node_from_start_tree, node_from_goal_tree)

        # Thresholds for considering a connection or goal reached
        self.goal_position_tolerance = 20.0
        self.goal_angle_tolerance_rad = math.radians(60.0)

        # Vehicle motion constraints
        self.max_steering_angle_rad = math.pi / 3
        self.max_velocity = 10.0
        self.min_velocity = 1.0 # Minimum velocity for PFF fallback

        self.grid_dimensions = self.collision_lookup["collision_mask"].shape # (height, width)
        self.planning_time_budget_sec = 10.0

        self.motion_primitives = self._initialize_motion_primitives()

        # Control and cost parameters
        self.proportional_steering_gain = 0.7 # For PFF fallback controller
        self.steering_smoothness_weight = 0.1 # Penalty for steering in primitive selection
        self.goal_heading_alignment_weight = 250.0 # Strong weight for aligning with target heading

        # Parameters
        self.nominal_connection_attempt_distance = 15.0 # Used in _try_connect_trees
        self.connection_max_distance_factor = 1.5 # Multiplier for nominal_connection_attempt_distance
        self.endpoint_weight = 1e4
        self.base_max_s = 1e4

    def _calculate_inverse_angle(self, theta_rad):
        """Calculates an angle roughly opposite to the input, for backward expansion."""
        angle = normalize_angle(theta_rad)
        if angle < 0:
            return angle + math.pi
        return angle-math.pi

    def _initialize_motion_primitives(self):
        """Generates a discrete set of motion primitives."""
        primitives_list = []
        velocities_to_try = [0.5 * self.max_velocity, 0.8 * self.max_velocity, self.max_velocity]
        steering_range_limit = 0.9 * self.max_steering_angle_rad
        steering_angles_to_try_rad = np.array([
            0.0,
            0.25 * steering_range_limit, -0.25 * steering_range_limit,
            0.6 * steering_range_limit, -0.6 * steering_range_limit,
            steering_range_limit, -steering_range_limit
        ])

        for velocity in velocities_to_try:
            for steering_angle in steering_angles_to_try_rad:
                num_steps = 15
                if abs(steering_angle) > 0.5 * steering_range_limit:
                    num_steps = 20
                elif steering_angle == 0.0:
                    num_steps = 20 if velocity == self.max_velocity else 15

                primitive_path = self._generate_single_primitive_path(velocity, steering_angle, num_steps)
                if primitive_path:
                    primitives_list.append(primitive_path)

        # Ensure at least one long straight primitive exists
        has_long_straight_primitive = any(
            p[0][4] == 0.0 and p[0][3] == self.max_velocity and len(p) > 15
            for p in primitives_list
        )
        if not has_long_straight_primitive:
            primitives_list.append(self._generate_single_primitive_path(self.max_velocity, 0.0, steps=20))

        print(f"Initialized {len(primitives_list)} motion primitives.")
        return primitives_list

    def _generate_single_primitive_path(self, velocity, steering_angle_rad, steps=15):
        """Simulates a vehicle path for a given velocity and steering angle."""
        path_states = [(0.0, 0.0, 0.0, velocity, steering_angle_rad)] # Relative initial state
        current_x, current_y, current_theta = 0.0, 0.0, 0.0

        for _ in range(steps):
            current_x += velocity * math.cos(current_theta) * self.time_step_simulation
            current_y += velocity * math.sin(current_theta) * self.time_step_simulation
            current_theta = normalize_angle(
                current_theta + (velocity * math.tan(steering_angle_rad) / self.vehicle_length) * self.time_step_simulation
            )
            path_states.append((current_x, current_y, current_theta, velocity, steering_angle_rad))
        return path_states

    def _build_kdtree(self, tree_identifier='start'):
        """Builds or rebuilds the KD-tree for the specified tree."""
        node_list = self.nodes_in_start_tree if tree_identifier == 'start' else self.nodes_in_goal_tree
        if not node_list:
            return

        positions = np.array([(node.x, node.y) for node in node_list])
        if positions.ndim == 1: # Single point
            positions = positions.reshape(1, -1)
        if not positions.size: # Empty array
            return

        kdtree_instance = scipy.spatial.cKDTree(positions)
        if tree_identifier == 'start':
            self.kdtree_for_start_tree = kdtree_instance
        else:
            self.kdtree_for_goal_tree = kdtree_instance

    def _get_nearest_node_in_tree(self, query_position_xy, tree_identifier='start'):
        """Finds the nearest node in the specified tree to the query_position_xy."""
        target_node_list = self.nodes_in_start_tree if tree_identifier == 'start' else self.nodes_in_goal_tree
        target_kdtree = self.kdtree_for_start_tree if tree_identifier == 'start' else self.kdtree_for_goal_tree

        if not target_node_list:
            return None

        if target_kdtree is None or len(target_node_list) % self.kdtree_rebuild_frequency == 0:
            self._build_kdtree(tree_identifier)
            target_kdtree = self.kdtree_for_start_tree if tree_identifier == 'start' else self.kdtree_for_goal_tree

        if target_kdtree is None: # Fallback if KD-tree build failed or list is too small
            return min(
                target_node_list,
                key=lambda node: math.hypot(node.x - query_position_xy[0], node.y - query_position_xy[1]),
                default=None
            )

        _, nearest_indices = target_kdtree.query(query_position_xy)
        nearest_index = nearest_indices[0] if isinstance(nearest_indices, (np.ndarray, list)) else nearest_indices
        nearest_index = int(nearest_index)

        if 0 <= nearest_index < len(target_node_list):
            return target_node_list[nearest_index]
        return None

    def _get_random_global_target_sample(self):
        """Samples a random (x, y) position within the grid boundaries."""
        random_x = random.uniform(0, self.grid_dimensions[1] - 1)  # Grid width
        random_y = random.uniform(0, self.grid_dimensions[0] - 1)  # Grid height
        return (random_x, random_y)

    def _generate_new_node_kinodynamically(self,
                                        source_node,
                                        target_position_xy,
                                        optional_target_theta_rad=None,
                                        tree_identifier='start', # Unused in this version, kept for signature consistency
                                        is_direct_connection_attempt=False,
                                        force_heading_guidance_for_local_exploration=False): # Unused in this version
        """
        Generates a new node by applying motion primitives or a fallback controller
        from source_node towards target_position_xy.
        """
        target_state_for_scoring = (target_position_xy[0], target_position_xy[1], optional_target_theta_rad)
        best_score_for_primitive = float('inf')
        best_resulting_node_from_primitive = None

        # Determine if heading cost should be applied for primitive selection
        apply_heading_cost = (optional_target_theta_rad is not None) and \
                             (is_direct_connection_attempt or force_heading_guidance_for_local_exploration)

        # Try motion primitives first
        for primitive_path_model in self.motion_primitives:
            if not primitive_path_model:
                continue

            transformed_primitive_path = self._transform_primitive_to_global_frame(primitive_path_model, source_node)
            if not transformed_primitive_path or len(transformed_primitive_path) < 2:
                continue

            end_state_of_primitive = transformed_primitive_path[-1]
            # Avoid trivial primitives that don't move
            if math.hypot(end_state_of_primitive[0] - source_node.x, end_state_of_primitive[1] - source_node.y) < 0.01:
                continue

            # Calculate cost for this primitive
            position_distance_cost = math.hypot(
                end_state_of_primitive[0] - target_state_for_scoring[0],
                end_state_of_primitive[1] - target_state_for_scoring[1]
            )
            current_primitive_score = position_distance_cost + \
                                      abs(primitive_path_model[0][4]) * self.steering_smoothness_weight # Penalty for steering

            if apply_heading_cost:
                angle_difference_rad = abs(normalize_angle(end_state_of_primitive[2] - target_state_for_scoring[2]))
                current_primitive_score += angle_difference_rad * self.goal_heading_alignment_weight

            if current_primitive_score < best_score_for_primitive:
                is_collision_free = True
                # Check collision for all waypoints in the transformed primitive, skipping the first (source_node)
                for point_state in transformed_primitive_path[1:]:
                    if fast_collision_check(point_state[1], point_state[0], self.collision_lookup):
                        is_collision_free = False
                        break
                
                if is_collision_free:
                    best_score_for_primitive = current_primitive_score
                    segment_time_cost = (len(primitive_path_model) - 1) * self.time_step_simulation
                    best_resulting_node_from_primitive = RRTNode(
                        *end_state_of_primitive, # x, y, theta, v, phi
                        cost=source_node.cost + segment_time_cost,
                        parent=source_node
                    )
                    best_resulting_node_from_primitive.path_from_parent = transformed_primitive_path

        if best_resulting_node_from_primitive:
            return best_resulting_node_from_primitive

        # Fallback: If no primitive was suitable, try Proportional Feedback Controller (PFF) / "apply_controls"
        return self._apply_proportional_feedback_control(source_node, target_state_for_scoring, is_direct_connection_attempt)


    def _apply_proportional_feedback_control(self,
                                             source_node,
                                             target_state_tuple, # (x, y, optional_theta)
                                             is_direct_connection_attempt=False): # Unused in this version
        """
        Fallback controller: Steers the vehicle from source_node towards target_state_tuple.
        This is a simplified Proportional Feedback Controller (PFF).
        """
        current_x, current_y, current_theta, _, _ = source_node.get_state()
        target_x, target_y, _ = target_state_tuple # Target theta not directly used by this PFF for steering cmd

        delta_x, delta_y = target_x - current_x, target_y - current_y
        distance_to_target_xy = math.hypot(delta_x, delta_y)

        if distance_to_target_xy < 0.1: # Already close enough
            return None

        path_segment_states = [source_node.get_state()]

        # Scale velocity based on distance, similar to original step_size logic
        # This PFF generates segments of length comparable to one visualization sampling step
        velocity_scale_factor = min(1.0, distance_to_target_xy / (2.0 * self.visualization_sampling_step_size))
        applied_velocity = self.max_velocity * velocity_scale_factor

        # Adjust velocity based on alignment with target direction
        alignment_with_target_direction = max(0.1, math.cos(normalize_angle(math.atan2(delta_y, delta_x) - current_theta)))
        applied_velocity = max(self.min_velocity, applied_velocity * alignment_with_target_direction)
        applied_velocity = min(applied_velocity, self.max_velocity)

        # Proportional steering control
        heading_error_rad = normalize_angle(math.atan2(delta_y, delta_x) - current_theta)
        applied_steering_angle = max(
            -self.max_steering_angle_rad * 0.9,
            min(self.proportional_steering_gain * heading_error_rad, self.max_steering_angle_rad * 0.9)
        )

        # Determine number of simulation steps for this segment
        # Aim for segment length related to visualization_sampling_step_size or distance to target
        effective_step_length = applied_velocity * self.time_step_simulation
        target_segment_length = min(distance_to_target_xy, self.visualization_sampling_step_size)
        num_simulation_steps = max(5, min(int(target_segment_length / (effective_step_length + 1e-6)), 25))


        sim_x, sim_y, sim_theta = current_x, current_y, current_theta
        for _ in range(num_simulation_steps):
            next_x = sim_x + applied_velocity * math.cos(sim_theta) * self.time_step_simulation
            next_y = sim_y + applied_velocity * math.sin(sim_theta) * self.time_step_simulation
            next_theta = normalize_angle(
                sim_theta + (applied_velocity * math.tan(applied_steering_angle) / self.vehicle_length) * self.time_step_simulation
            )

            if fast_collision_check(next_y, next_x, self.collision_lookup):
                return None # Collision detected

            sim_x, sim_y, sim_theta = next_x, next_y, next_theta
            path_segment_states.append((sim_x, sim_y, sim_theta, applied_velocity, applied_steering_angle))

        if len(path_segment_states) <= 1: # No valid step taken
            return None

        final_state_in_segment = path_segment_states[-1]
        new_node_from_pff = RRTNode(
            *final_state_in_segment, # x, y, theta, v, phi
            cost=source_node.cost + (len(path_segment_states) - 1) * self.time_step_simulation,
            parent=source_node
        )
        new_node_from_pff.path_from_parent = path_segment_states
        return new_node_from_pff


    def _extend_tree(self,
                     tree_node_list,
                     tree_identifier, # 'start' or 'goal'
                     global_target_position_xy):
        """Extends the specified tree towards the global_target_position_xy."""
        nearest_node_in_tree = self._get_nearest_node_in_tree(global_target_position_xy, tree_identifier)
        if not nearest_node_in_tree:
            return None

        optional_target_theta_for_expansion = None
        is_attempting_direct_connection_to_overall_goal = False

        # If sampling the actual start/goal, provide its orientation for guidance
        if tree_identifier == 'start' and \
           global_target_position_xy == (self.true_goal_configuration.x, self.true_goal_configuration.y):
            optional_target_theta_for_expansion = self.true_goal_configuration.theta
            is_attempting_direct_connection_to_overall_goal = True
        elif tree_identifier == 'goal' and \
             global_target_position_xy == (self.start_node.x, self.start_node.y):
            # For goal tree expanding towards start, use start_node's true world theta (not inverse)
            optional_target_theta_for_expansion = self.start_node.theta
            is_attempting_direct_connection_to_overall_goal = True

        newly_generated_node = self._generate_new_node_kinodynamically(
            source_node=nearest_node_in_tree,
            target_position_xy=global_target_position_xy,
            optional_target_theta_rad=optional_target_theta_for_expansion,
            tree_identifier=tree_identifier, # Passed along, though _generate_new_node doesn't currently use it directly
            is_direct_connection_attempt=is_attempting_direct_connection_to_overall_goal,
            force_heading_guidance_for_local_exploration=False # Typically false for general extension
        )

        if newly_generated_node:
            tree_node_list.append(newly_generated_node)
            return newly_generated_node
        return None


    def _try_connect_trees(self,
                           node_from_active_tree, # The newly added node
                           active_tree_identifier): # 'start' or 'goal'
        """Attempts to connect node_from_active_tree to the other tree."""
        other_tree_identifier = 'goal' if active_tree_identifier == 'start' else 'start'

        nearest_node_in_other_tree = self._get_nearest_node_in_tree(
            (node_from_active_tree.x, node_from_active_tree.y), other_tree_identifier
        )
        if not nearest_node_in_other_tree:
            return False

        distance_between_nodes = math.hypot(
            node_from_active_tree.x - nearest_node_in_other_tree.x,
            node_from_active_tree.y - nearest_node_in_other_tree.y
        )

        # Check if nodes are too far apart for a connection attempt
        max_allowed_connection_distance = self.connection_max_distance_factor * self.nominal_connection_attempt_distance
        if distance_between_nodes > max_allowed_connection_distance:
            return False

        # Define connection target based on the 'other' tree's node
        target_connection_x = nearest_node_in_other_tree.x
        target_connection_y = nearest_node_in_other_tree.y
        # Target theta must match the frame of the 'other' tree's node
        # If 'other' is goal tree, its nodes have inverse angles. We need to aim for that inverse angle.
        # If 'other' is start tree, its nodes have world angles. Aim for that.
        target_connection_theta_rad = nearest_node_in_other_tree.theta # This is correct because nodes store their 'effective' forward theta

        # Attempt to generate a path segment from node_from_active_tree to nearest_node_in_other_tree
        connection_candidate_node = self._generate_new_node_kinodynamically(
            source_node=node_from_active_tree,
            target_position_xy=(target_connection_x, target_connection_y),
            optional_target_theta_rad=target_connection_theta_rad,
            tree_identifier=active_tree_identifier,
            is_direct_connection_attempt=True, # This is a focused connection
            force_heading_guidance_for_local_exploration=True # Strongly guide heading
        )

        if connection_candidate_node:
            # The connection_candidate_node is an extension of node_from_active_tree.
            # Its theta is in the 'forward' frame of the active_tree.
            # We need to check if this new node (connection_candidate_node) is close enough
            # in pose to the nearest_node_in_other_tree.

            final_connection_node_theta = connection_candidate_node.theta

            position_criteria_met = math.hypot(
                connection_candidate_node.x - target_connection_x,
                connection_candidate_node.y - target_connection_y
            ) <= self.goal_position_tolerance # Using goal_position_tolerance for connection tightness

            # Angle check: connection_candidate_node's theta should align with target_connection_theta_rad
            angle_diff_to_target_node = abs(normalize_angle(final_connection_node_theta - target_connection_theta_rad))
            angle_criteria_to_target_met = angle_diff_to_target_node <= self.goal_angle_tolerance_rad
            
            # Sanity check: the connection segment itself should be somewhat aligned with the source node's direction
            # This helps prevent awkward, immediate reverse connections.
            # node_from_active_tree.theta is already in its correct forward frame.
            angle_diff_from_source_node = abs(normalize_angle(final_connection_node_theta - node_from_active_tree.theta))
            angle_criteria_from_source_met = angle_diff_from_source_node <= self.goal_angle_tolerance_rad * 1.5 # Slightly more lenient

            if position_criteria_met and angle_criteria_to_target_met and angle_criteria_from_source_met:
                if active_tree_identifier == 'start':
                    self.tree_connection_nodes = (connection_candidate_node, nearest_node_in_other_tree)
                else: # active_tree was 'goal'
                    self.tree_connection_nodes = (nearest_node_in_other_tree, connection_candidate_node)
                self.is_path_found = True
                return True
        return False


    def plan(self, visualize_planning_output=True):
        """Main planning loop for the Bidirectional Kinodynamic RRT."""
        print("Starting optimized Kinodynamic RRT* (bidirectional) planning...")
        self.planning_start_time = time.time()

        for iteration in range(self.max_iterations):
            if time.time() - self.planning_start_time > self.planning_time_budget_sec:
                print("Planning time budget exceeded.")
                break

            if (iteration + 1) % 200 == 0:
                print(f"Iteration {iteration + 1}, "
                      f"Start Tree Size: {len(self.nodes_in_start_tree)}, "
                      f"Goal Tree Size: {len(self.nodes_in_goal_tree)}")

            # Determine which tree to extend: prioritize smaller tree, otherwise alternate
            extend_start_tree_this_iteration = len(self.nodes_in_start_tree) <= len(self.nodes_in_goal_tree)
            if abs(len(self.nodes_in_start_tree) - len(self.nodes_in_goal_tree)) <= 5 : # If balanced
                 extend_start_tree_this_iteration = (iteration % 2 == 0)


            active_tree_nodes, active_tree_identifier = \
                (self.nodes_in_start_tree, 'start') if extend_start_tree_this_iteration \
                else (self.nodes_in_goal_tree, 'goal')

            # Determine the overall bias target (actual goal for start tree, actual start for goal tree)
            # This is used by the _sample_position_for_visualization for its internal scoring
            overall_bias_target_position_xy = \
                (self.true_goal_configuration.x, self.true_goal_configuration.y) if active_tree_identifier == 'start' \
                else (self.start_node.x, self.start_node.y)

            # 1. Get a global random sample (or biased sample towards goal/start)
            if random.random() < self.goal_biasing_rate:
                 sampled_global_target_position = overall_bias_target_position_xy
            else:
                sampled_global_target_position = self._get_random_global_target_sample()

            # Debug prints for early iterations
            if iteration < 10:
                print(f"Iter {iteration}: Global RRT Target: {sampled_global_target_position}")


            # 3. Extend the chosen RRT tree using the sampled_global_target_position
            newly_added_node_to_tree = self._extend_tree(
                active_tree_nodes,
                active_tree_identifier,
                sampled_global_target_position
            )

            if newly_added_node_to_tree:
                # 4. Try to connect the new node to the other tree
                if self._try_connect_trees(newly_added_node_to_tree, active_tree_identifier):
                    print(f"Path found and connected at iteration {iteration + 1}!")
                    break # Exit main planning loop

        if not self.is_path_found:
             print(f"Path planning finished. No path found within {self.max_iterations} iterations or time budget.")

        final_path_tuples = None
        if self.is_path_found:
            final_path_tuples = self._extract_final_path()

        if visualize_planning_output:
            self._visualize_bidirectional_rrt_result(final_path_tuples)

        return final_path_tuples

    def _extract_final_path(self):
        """Reconstructs the path from start to goal once trees are connected."""
        if not self.is_path_found or not self.tree_connection_nodes:
            return None

        # node_at_start_tree_conn_point is the node in the start-tree part of the connection
        # node_at_goal_tree_conn_point is the node in the goal-tree part of the connection
        node_at_start_tree_conn_point, node_at_goal_tree_conn_point = self.tree_connection_nodes

        # Part 1: Path from actual start to the connection point on the start tree side
        path_start_to_connection_tuples = []
        current_node = node_at_start_tree_conn_point
        while current_node:
            if current_node.path_from_parent:
                # path_from_parent is (x,y,theta,v,phi), we need (x,y,theta)
                segment = [(p[0], p[1], p[2]) for p in current_node.path_from_parent]
                path_start_to_connection_tuples = segment + path_start_to_connection_tuples
            elif current_node == self.start_node: # Ensure start node itself is included if no path_from_parent
                 path_start_to_connection_tuples.insert(0, (current_node.x, current_node.y, current_node.theta))
            current_node = current_node.parent
        path_start_to_connection_tuples = self._deduplicate_path_waypoints(path_start_to_connection_tuples)


        # Part 2: Path from actual goal to the connection point on the goal tree side (then reverse it)
        path_goal_to_connection_reversed_tuples = []
        current_node = node_at_goal_tree_conn_point # This node is part of the GOAL tree
        while current_node:
            if current_node.path_from_parent:
                 # Path from parent in goal tree is kinematically forward, but angle is inverse.
                 # So, when reconstructing, convert angles back to world frame.
                segment = [(p[0], p[1], self._calculate_inverse_angle(p[2])) for p in current_node.path_from_parent]
                path_goal_to_connection_reversed_tuples = segment + path_goal_to_connection_reversed_tuples
            elif current_node == self.goal_tree_root_node:
                path_goal_to_connection_reversed_tuples.insert(0, (current_node.x, current_node.y,
                                                                self._calculate_inverse_angle(current_node.theta)))
            current_node = current_node.parent
        path_goal_to_connection_reversed_tuples = self._deduplicate_path_waypoints(path_goal_to_connection_reversed_tuples)

        # Reverse the goal-to-connection path to get connection-to-goal path
        path_connection_to_goal_tuples = list(reversed(path_goal_to_connection_reversed_tuples))

        # Combine the two path segments
        final_path_tuples = path_start_to_connection_tuples
        if path_start_to_connection_tuples and path_connection_to_goal_tuples:
            # Check if the last point of start_path and first of goal_path are too close (likely duplicates)
            last_of_start = final_path_tuples[-1]
            first_of_goal = path_connection_to_goal_tuples[0]
            if (abs(last_of_start[0] - first_of_goal[0]) < 0.1 and
                abs(last_of_start[1] - first_of_goal[1]) < 0.1 and
                abs(normalize_angle(last_of_start[2] - first_of_goal[2])) < math.radians(5)):
                final_path_tuples.extend(path_connection_to_goal_tuples[1:]) # Skip duplicate
            else:
                final_path_tuples.extend(path_connection_to_goal_tuples)
        elif path_connection_to_goal_tuples: # If start path was empty (e.g. start IS connection)
            final_path_tuples.extend(path_connection_to_goal_tuples)

        return self._smooth_path(final_path_tuples)

    def _smooth_path(self, path):
        """
        Fits a cubic smoothing spline through (x,y) that
        *always* passes exactly through the first and last points,
        while still smoothing the interior subject to your max-curvature constraint.
        """
        if len(path) < 3:
            return path

        # 1) extract raw waypoints
        xs = np.array([p[0] for p in path])
        ys = np.array([p[1] for p in path])

        # Build a weight vector so that endpoints are “hard” constraints
        w = np.ones(len(xs))
        w[0] = w[-1] = self.endpoint_weight

        # 2) compute curvature limit
        L = self.vehicle_length
        phi_max = self.max_steering_angle_rad
        kappa_max = math.tan(phi_max) / L

        # 3) fit with increasing smoothness until curvature is OK
        s_val = 0.0
        while True:
            # pass our custom weights into splprep
            tck, u = splprep([xs, ys], s=s_val, w=w)
            u_fine = np.linspace(0, 1, len(path))
            xs_s, ys_s = splev(u_fine, tck)

            # curvature check
            dx  = np.gradient(xs_s, u_fine)
            dy  = np.gradient(ys_s, u_fine)
            ddx = np.gradient(dx,    u_fine)
            ddy = np.gradient(dy,    u_fine)
            curvature = np.abs(dx*ddy - dy*ddx) / (dx*dx + dy*dy)**1.5

            if np.nanmax(curvature) <= kappa_max or s_val >= self.base_max_s:
                break
            s_val = max(1e-3, s_val * 2 if s_val>0 else 1.0)

        # Just to be *absolutely* sure, re-enforce exact endpoints:
        xs_s[0], ys_s[0] = xs[0], ys[0]
        xs_s[-1], ys_s[-1] = xs[-1], ys[-1]

        # 4) reconstruct headings from smoothed trajectory
        thetas = np.arctan2(
            np.gradient(ys_s, u_fine),
            np.gradient(xs_s, u_fine)
        )

        return list(zip(xs_s, ys_s, thetas))

    def _deduplicate_path_waypoints(self, path_tuples_list):
        """Removes consecutive duplicate or very close waypoints from a path list."""
        if not path_tuples_list:
            return []
        
        deduplicated_path = [path_tuples_list[0]]
        for i in range(1, len(path_tuples_list)):
            p1_x, p1_y, p1_theta = deduplicated_path[-1]
            p2_x, p2_y, p2_theta = path_tuples_list[i]

            position_differs = abs(p1_x - p2_x) >= 1e-3 or abs(p1_y - p2_y) >= 1e-3
            angle_differs = abs(normalize_angle(p1_theta - p2_theta)) >= 1e-3

            if position_differs or angle_differs:
                deduplicated_path.append(path_tuples_list[i])
        return deduplicated_path


    def _transform_primitive_to_global_frame(self, relative_primitive_path, source_node_global_frame):
        """Transforms a relative motion primitive path to the global coordinate frame of source_node."""
        if not relative_primitive_path:
            return []

        transformed_path_segment_states = []
        base_x, base_y, base_theta_rad = source_node_global_frame.x, source_node_global_frame.y, source_node_global_frame.theta

        for i, (rel_x, rel_y, rel_theta, prim_v, prim_phi) in enumerate(relative_primitive_path):
            # Rotate
            rotated_relative_x = rel_x * math.cos(base_theta_rad) - rel_y * math.sin(base_theta_rad)
            rotated_relative_y = rel_x * math.sin(base_theta_rad) + rel_y * math.cos(base_theta_rad)
            # Translate
            global_x = rotated_relative_x + base_x
            global_y = rotated_relative_y + base_y
            # Combine angles
            global_theta_rad = normalize_angle(rel_theta + base_theta_rad)

            transformed_path_segment_states.append((global_x, global_y, global_theta_rad, prim_v, prim_phi))
        return transformed_path_segment_states


    def _visualize_bidirectional_rrt_result(self, final_path_tuples):
        """Visualizes the RRT trees and the final path."""
        plt.figure(figsize=(13, 13))
        # Plot occupancy grid
        plt.imshow(
            self.occupancy_grid, cmap='gray',
            extent=[0, self.grid_dimensions[1], 0, self.grid_dimensions[0]]
        )

        # Plot tree edges
        tree_configs = [
            (self.nodes_in_start_tree, 'deepskyblue', 'Start Tree Edge'),
            (self.nodes_in_goal_tree, 'lightcoral', 'Goal Tree Edge')
        ]
        for node_list, color_str, label_str in tree_configs:
            plotted_label = False
            for node in node_list:
                if node.parent and node.path_from_parent and len(node.path_from_parent) > 0:
                    path_xs = [p[0] for p in node.path_from_parent]
                    path_ys = [p[1] for p in node.path_from_parent]
                    if not plotted_label:
                        plt.plot(path_xs, path_ys, color=color_str, alpha=0.5, linewidth=1.5, label=label_str)
                        plotted_label = True
                    else:
                        plt.plot(path_xs, path_ys, color=color_str, alpha=0.5, linewidth=1.5)


        # Plot Start and Goal nodes
        plt.scatter(
            self.start_node.x, self.start_node.y, color='lime', s=120, marker='o',
            edgecolor='black', label='Start Pose', zorder=5
        )
        plt.arrow(
            self.start_node.x, self.start_node.y,
            20.0 * math.cos(self.start_node.theta), 20.0 * math.sin(self.start_node.theta), # Increased arrow length
            color='lime', head_width=7.0, head_length=9.0, edgecolor='black', linewidth=1.5, zorder=5 # Increased arrow size
        )
        plt.scatter(
            self.true_goal_configuration.x, self.true_goal_configuration.y, color='red', s=120, marker='X',
            edgecolor='black', label='Goal Pose', zorder=5
        )
        plt.arrow(
            self.true_goal_configuration.x, self.true_goal_configuration.y,
            20.0 * math.cos(self.true_goal_configuration.theta), 20.0 * math.sin(self.true_goal_configuration.theta), # Increased arrow length
            color='red', head_width=7.0, head_length=9.0, edgecolor='black', linewidth=1.5, zorder=5 # Increased arrow size
        )

        # Plot connection points if path found
        if self.tree_connection_nodes:
            node_s_conn, node_g_conn = self.tree_connection_nodes
            plt.scatter(
                node_s_conn.x, node_s_conn.y, color='gold', s=150, marker='*',
                edgecolor='black', label='Connection Pt (Start Tree Side)', zorder=6
            )
            plt.scatter(
                node_g_conn.x, node_g_conn.y, color='darkorange', s=150, marker='*', # Changed color for distinctness
                edgecolor='black', label='Connection Pt (Goal Tree Side)', zorder=6
            )

        # Plot the final path
        if final_path_tuples and len(final_path_tuples) > 1:
            path_xs = [p[0] for p in final_path_tuples]
            path_ys = [p[1] for p in final_path_tuples]
            plt.plot(path_xs, path_ys, color='green', linestyle='-', linewidth=2.0, label='Final Path')
            # Plot orientation arrows along the path
            for i in range(0, len(final_path_tuples), max(1, len(final_path_tuples) // 25)): # ~25 arrows
                plt.arrow(
                    final_path_tuples[i][0], final_path_tuples[i][1],
                    12.0 * math.cos(final_path_tuples[i][2]), 12.0 * math.sin(final_path_tuples[i][2]), # Increased arrow length
                    color='darkgreen', head_width=3.0, head_length=5.0, alpha=0.8, zorder=4 # Increased arrow size
                )

        plt.title("Bidirectional Kinodynamic RRT* Path Planning Result")
        plt.xlabel("X-position (grid units)")
        plt.ylabel("Y-position (grid units)")
        plt.legend(fontsize='small')
        plt.axis('equal')
        plt.tight_layout()
        
        # Save the figure
        plt.savefig(f"bidirectional_kinodynamic_rrt_path_{time.time()}.png", dpi=300)
        # plt.show() # Uncomment to display plot interactively