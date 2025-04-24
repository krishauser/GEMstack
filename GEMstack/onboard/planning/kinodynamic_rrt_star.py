import math
import random
import time
import numpy as np
import multiprocessing
import scipy.spatial
import matplotlib.pyplot as plt

from concurrent.futures import ProcessPoolExecutor, as_completed
from utils import normalize_angle
from dubins import generate_dubins_path
from collision import fast_collision_check

class RRTNode:
    def __init__(self, x, y, theta, cost=0.0, parent=None):
        self.x = x
        self.y = y
        self.theta = theta
        self.cost = cost
        self.parent = parent
        self.path_from_parent = []

    def get_state(self):
        """Get node state as (x, y, theta)."""
        return (self.x, self.y, self.theta)
    
    def distance_to(self, other):
        """Simple Euclidean distance for nearest neighbor search."""
        return math.sqrt((self.x - other.x)**2 + (self.y - other.y)**2)
    
    def set_path_from_parent(self, path):
        """Set the path from parent to this node."""
        if len(path) > 0:
            self.path_from_parent = path[1:]

class KinodynamicRRTStar:
    
    def __init__(self, occupancy_grid, collision_lookup, metadata, start, goal, 
                max_iter=1000, step_size=1.0, goal_sample_rate=0.1,
                turning_radius=3.0, rewire_radius=20.0, parallel=True, num_workers=None,
                adaptive_radius=True, shortcut_path=True):
        """
        Initialize Kinodynamic RRT* planner.
        
        Args:
            occupancy_grid: Binary grid (1=obstacle, 0=free)
            collision_lookup: Collision lookup table
            metadata: Map metadata
            start: (x, y, theta) start state
            goal: (x, y, theta) goal state
            max_iter: Maximum iterations
            step_size: Step size for path sampling
            goal_sample_rate: Probability of sampling the goal
            turning_radius: Minimum turning radius
            rewire_radius: Maximum radius for rewiring
            parallel: Whether to use parallel processing
            num_workers: Number of worker processes (None = auto)
            adaptive_radius: Whether to use adaptive turning radius based on obstacle proximity
            shortcut_path: Whether to apply path shortcutting/smoothing
        """
        self.grid = occupancy_grid
        self.collision_lookup = collision_lookup
        self.metadata = metadata
        self.start = RRTNode(*start)
        self.goal = RRTNode(*goal)
        self.max_iter = max_iter
        self.step_size = step_size
        self.goal_sample_rate = goal_sample_rate
        self.turning_radius = turning_radius
        self.rewire_radius = rewire_radius
        self.nodes = [self.start]
        self.parallel = parallel
        self.num_workers = num_workers if num_workers else max(1, multiprocessing.cpu_count() - 1)
        self.adaptive_radius = adaptive_radius
        self.shortcut_path = shortcut_path
        self.max_turning_radius = turning_radius * 3.0  # Maximum allowed turning radius
        self.min_turning_radius = turning_radius  # Minimum turning radius (vehicle constraint)
        self.path_found = False  # Add a flag to track if path is found
        
        # Set up KD-Tree for fast nearest neighbor search
        self.kdtree = scipy.spatial.KDTree(np.array([(self.start.x, self.start.y)]))
        
        # Grid dimensions
        self.x_min, self.y_min = 0, 0
        self.x_max, self.y_max = occupancy_grid.shape
        
    def _random_state(self):
        """Generate a random state in the grid."""
        if random.random() < self.goal_sample_rate:
            return self.goal.get_state()
        
        while True:
            x = random.uniform(self.x_min, self.x_max)
            y = random.uniform(self.y_min, self.y_max)
            theta = random.uniform(-math.pi, math.pi)
            
            # Check if the sampled point is in free space
            if not fast_collision_check(x, y, self.collision_lookup):
                return (x, y, theta)
    
    def _nearest_node(self, rand_state):
        """Find the nearest node to the random state."""
        _, idx = self.kdtree.query((rand_state[0], rand_state[1]))
        return self.nodes[idx]
    
    def _near_nodes(self, node, radius=None):
        """Find nodes within the specified radius."""
        if radius is None:
            radius = self.rewire_radius
            
        indices = self.kdtree.query_ball_point((node.x, node.y), radius)
        return [self.nodes[idx] for idx in indices if idx < len(self.nodes)]
    
    def _compute_adaptive_radius(self, x, y):
        """Compute adaptive turning radius based on obstacle proximity."""
        if not self.adaptive_radius:
            return self.turning_radius
            
        xi, yi = int(round(x)), int(round(y))
        dist_map = self.collision_lookup["distance"]

        if xi < 0 or yi < 0 or xi >= dist_map.shape[0] or yi >= dist_map.shape[1]:
            return self.turning_radius  # Use default if out of bounds
            
        distance = dist_map[xi, yi]
        
        radius = min(self.max_turning_radius, 
                     max(self.min_turning_radius, 
                         self.min_turning_radius + distance/5.0))
        
        return radius
    
    def _try_direct_path(self, from_node, to_state):
        """
        Try to connect nodes with a straight line path if possible.
        Returns path and cost if successful, None otherwise.
        """
        from_state = from_node.get_state()
        
        # Check if headings are roughly aligned with path direction
        dx = to_state[0] - from_state[0]
        dy = to_state[1] - from_state[1]
        
        if dx == 0 and dy == 0:
            return None, float("inf")
            
        path_angle = math.atan2(dy, dx)
        
        # Check if from_heading is aligned with path direction
        from_alignment = abs(normalize_angle(from_state[2] - path_angle))
        to_alignment = abs(normalize_angle(to_state[2] - path_angle))
        
        # If both headings are well-aligned with path direction (within 45 degrees)
        if from_alignment <= math.radians(45) and to_alignment <= math.radians(45):
            # Generate a straight line path
            distance = math.sqrt(dx*dx + dy*dy)
            num_points = max(3, int(distance / self.step_size))
            
            path = [from_state]
            for i in range(1, num_points):
                t = i / (num_points - 1)
                x = from_state[0] + t * dx
                y = from_state[1] + t * dy
                
                # Linearly interpolate heading
                # We normalize the heading difference to ensure smooth interpolation
                heading_diff = normalize_angle(to_state[2] - from_state[2])
                theta = normalize_angle(from_state[2] + t * heading_diff)
                
                path.append((x, y, theta))
            
            # Add final state
            if path[-1] != to_state:
                path.append(to_state)
            
            # Check if path is collision-free
            if self._check_collision_free(path):
                return path, distance
                
        return None, float("inf")
    
    def _steer(self, from_node, to_state, extend_length=None):
        """
        Create a path from from_node to to_state respecting kinodynamic constraints.
        
        Note: Only forward motion is allowed (no reversing).
        
        Returns:
            new_node: New node at the end of the path
            path: List of states along the path
            dubins_cost: Cost of the path
        """
        if extend_length is None:
            extend_length = self.rewire_radius
            
        from_state = from_node.get_state()
        
        # First try a direct path (more cost-optimal)
        direct_path, direct_cost = self._try_direct_path(from_node, to_state)
        
        if direct_path:
            # Create node with direct path
            new_node = RRTNode(
                direct_path[-1][0], direct_path[-1][1], direct_path[-1][2],
                cost=from_node.cost + direct_cost,
                parent=from_node
            )
            new_node.set_path_from_parent(direct_path)
            return new_node, direct_path, direct_cost
            
        # If direct path fails, use adaptive turning radius for Dubins path
        radius = self._compute_adaptive_radius(from_state[0], from_state[1])
        
        # Generate Dubins path (forward motion only)
        path = generate_dubins_path(
            from_state, to_state, 
            radius,  # Use adaptive radius 
            step_size=self.step_size
        )
        
        if path is None:
            return None, None, float("inf")
        
        # Verify forward motion by checking each segment
        for i in range(1, len(path)):
            prev_x, prev_y, prev_theta = path[i-1]
            curr_x, curr_y, curr_theta = path[i]
            
            # Calculate direction of motion
            dx = curr_x - prev_x
            dy = curr_y - prev_y
            
            if dx == 0 and dy == 0:
                continue  # Skip zero-length segments
                
            # Direction of motion
            motion_angle = math.atan2(dy, dx)
            
            # Check if motion is aligned with heading (forward motion)
            # Allow small deviation due to numerical issues
            heading_diff = abs(normalize_angle(prev_theta - motion_angle))
            if heading_diff > math.radians(90):  # If more than 90 degrees, it's backward
                return None, None, float("inf")
        
        # Calculate path length (cost)
        dubins_cost = 0
        for i in range(1, len(path)):
            dx = path[i][0] - path[i-1][0]
            dy = path[i][1] - path[i-1][1]
            dubins_cost += math.sqrt(dx*dx + dy*dy)
        
        # Create new node
        new_node = RRTNode(
            path[-1][0], path[-1][1], path[-1][2],
            cost=from_node.cost + dubins_cost,
            parent=from_node
        )
        new_node.set_path_from_parent(path)
        
        return new_node, path, dubins_cost
    
    def _check_collision_free(self, path):
        """Check if a path is collision-free."""
        if path is None:
            return False
            
        for x, y, _ in path:
            if fast_collision_check(x, y, self.collision_lookup):
                return False
                
        return True
    
    def _calculate_cost(self, node):
        """Calculate the cost from start to node."""
        if node.parent is None:
            return 0
            
        cost = node.cost
        return cost
    
    def _rewire(self, new_node, near_nodes):
        """Rewire the tree to optimize costs."""
        # Skip if no path from parent (should not happen normally)
        if not new_node.path_from_parent:
            return
            
        # Loop through near nodes
        for near_node in near_nodes:
            # Skip if the near node is the new node's parent
            if near_node is new_node.parent:
                continue
                
            # Calculate potential new cost
            _, path, path_cost = self._steer(new_node, near_node.get_state())
            
            # If path exists and is collision-free and reduces cost
            if path and self._check_collision_free(path) and \
               new_node.cost + path_cost < near_node.cost:
                
                # Rewire near_node through new_node
                near_node.parent = new_node
                near_node.cost = new_node.cost + path_cost
                near_node.set_path_from_parent(path)
                
                # Recursively update costs for all children
                self._update_children_costs(near_node)
    
    def _update_children_costs(self, node):
        """Recursively update costs for a node's children."""
        # Find all children of the node
        for potential_child in self.nodes:
            if potential_child.parent is node:
                # Update child's cost
                path_cost = 0
                for i in range(1, len(potential_child.path_from_parent)):
                    p1 = potential_child.path_from_parent[i-1]
                    p2 = potential_child.path_from_parent[i]
                    path_cost += math.sqrt((p2[0]-p1[0])**2 + (p2[1]-p1[1])**2)
                
                potential_child.cost = node.cost + path_cost
                
                # Recursively update grandchildren
                self._update_children_costs(potential_child)
    
    def _try_connect_to_goal(self, node):
        """Try to connect a node to the goal."""
        new_node, path, cost = self._steer(node, self.goal.get_state())
        
        if path and self._check_collision_free(path):
            # Update goal node
            self.goal.parent = node
            self.goal.cost = node.cost + cost
            self.goal.set_path_from_parent(path)
            
            # Add goal to tree
            self.nodes.append(self.goal)
            
            # Set the path found flag
            self.path_found = True
            
            return True
            
        return False
    
    def _parallel_steering(self, tasks):
        """Process steering tasks in parallel with early termination support"""
        results = []
        
        # Use a manager to share state across processes
        with multiprocessing.Manager() as manager:
            # Create a shared flag that workers can check
            path_found = manager.Value('b', False)
            
            with ProcessPoolExecutor(max_workers=self.num_workers) as executor:
                futures = {}
                
                # Submit tasks to the executor
                for node, rand_state in tasks:
                    future = executor.submit(
                        self._process_steering_task, 
                        node.get_state(), 
                        rand_state, 
                        node.cost,
                        self.turning_radius,
                        self.step_size
                    )
                    futures[future] = node
                
                # Process results as they complete
                try:
                    for future in as_completed(futures):
                        # Check if path already found (for quick exit)
                        if path_found.value or self.path_found:
                            # Cancel remaining futures if possible
                            for f in futures:
                                if not f.done():
                                    f.cancel()
                            break
                            
                        node = futures[future]
                        try:
                            result = future.result()
                            if result:
                                new_node_state, path, cost = result
                                if path and len(path) > 1:
                                    new_node = RRTNode(
                                        new_node_state[0], 
                                        new_node_state[1], 
                                        new_node_state[2],
                                        cost=node.cost + cost,
                                        parent=node
                                    )
                                    new_node.set_path_from_parent(path)
                                    results.append((new_node, path))
                        except Exception as e:
                            print(f"Error in parallel steering: {e}")
                except:
                    # Ensure we cancel all futures if something goes wrong
                    for f in futures:
                        if not f.done():
                            f.cancel()
                    raise
        
        return results
    
    @staticmethod
    def _process_steering_task(from_state, to_state, base_cost, turning_radius, step_size):
        """Static method for parallel processing of steering tasks."""
        # Try direct path first (more cost-optimal)
        dx = to_state[0] - from_state[0]
        dy = to_state[1] - from_state[1]
        
        if dx == 0 and dy == 0:
            return None  # Same point
            
        path_angle = math.atan2(dy, dx)
        
        # Check if headings are aligned with path direction
        from_alignment = abs(normalize_angle(from_state[2] - path_angle))
        to_alignment = abs(normalize_angle(to_state[2] - path_angle))
        
        # If both headings are well-aligned with path direction (within 45 degrees)
        if from_alignment <= math.radians(45) and to_alignment <= math.radians(45):
            # Generate a straight line path
            distance = math.sqrt(dx*dx + dy*dy)
            num_points = max(3, int(distance / step_size))
            
            path = [from_state]
            for i in range(1, num_points):
                t = i / (num_points - 1)
                x = from_state[0] + t * dx
                y = from_state[1] + t * dy
                
                # Linearly interpolate heading
                heading_diff = normalize_angle(to_state[2] - from_state[2])
                theta = normalize_angle(from_state[2] + t * heading_diff)
                
                path.append((x, y, theta))
            
            # Add final state if needed
            if path[-1] != to_state:
                path.append(to_state)
                
            # Calculate cost
            direct_cost = distance
            return path[-1], path, direct_cost
        
        # If direct path not possible, use Dubins path
        # Generate Dubins path (forward motion only)
        path = generate_dubins_path(
            from_state, to_state, 
            turning_radius, 
            step_size=step_size
        )
        
        if path is None:
            return None
            
        # Verify forward motion by checking each segment
        for i in range(1, len(path)):
            prev_x, prev_y, prev_theta = path[i-1]
            curr_x, curr_y, curr_theta = path[i]
            
            # Calculate direction of motion
            dx = curr_x - prev_x
            dy = curr_y - prev_y
            
            if dx == 0 and dy == 0:
                continue  # Skip zero-length segments
                
            # Direction of motion
            motion_angle = math.atan2(dy, dx)
            
            # Check if motion is aligned with heading (forward motion)
            # Allow small deviation due to numerical issues
            heading_diff = abs(normalize_angle(prev_theta - motion_angle))
            if heading_diff > math.radians(90):  # If more than 90 degrees, it's backward
                return None
        
        # Calculate path cost
        cost = 0
        for i in range(1, len(path)):
            dx = path[i][0] - path[i-1][0]
            dy = path[i][1] - path[i-1][1]
            cost += math.sqrt(dx*dx + dy*dy)
        
        # Return the result
        return path[-1], path, cost
    
    def _shortcut_path(self, path):
        """
        Apply path shortcutting to remove unnecessary curves while preserving
        kinematic constraints.
        """
        if not path or not self.shortcut_path:
            return path
            
        # Parameters for shortcutting
        min_segment_length = self.turning_radius * 2  # Minimum length to consider shortcutting
        heading_threshold = math.radians(20)  # Maximum heading deviation
        
        i = 0
        shortened_path = [path[0]]
        
        while i < len(path) - 1:
            # Try to find the furthest point we can connect directly
            # while maintaining kinematic feasibility
            max_feasible_j = i + 1
            
            # Current point
            current = path[i]
            
            # Try connecting to further points
            for j in range(i + 2, len(path)):
                # Skip points that are too close
                distance = math.sqrt((path[j][0] - current[0])**2 + 
                                    (path[j][1] - current[1])**2)
                if distance < min_segment_length:
                    continue
                    
                # Check if headings at endpoints are compatible with direct path
                direct_angle = math.atan2(path[j][1] - current[1], 
                                         path[j][0] - current[0])
                
                # Check heading alignment at start and end points
                start_heading_diff = abs(normalize_angle(current[2] - direct_angle))
                end_heading_diff = abs(normalize_angle(path[j][2] - direct_angle))
                
                # If headings aren't well-aligned, direct connection isn't feasible
                if start_heading_diff > heading_threshold or end_heading_diff > heading_threshold:
                    continue
                
                # Generate straight-line path for collision checking
                direct_path = []
                num_points = max(5, int(distance / self.step_size))
                
                direct_path = [current]
                for k in range(1, num_points):
                    t = k / (num_points - 1)
                    x = current[0] + t * (path[j][0] - current[0])
                    y = current[1] + t * (path[j][1] - current[1])
                    
                    # Smoothly interpolate heading
                    heading_diff = normalize_angle(path[j][2] - current[2])
                    theta = normalize_angle(current[2] + t * heading_diff)
                    
                    direct_path.append((x, y, theta))
                
                # Add end point if needed
                if direct_path[-1] != path[j]:
                    direct_path.append(path[j])
                
                # Check collision-free
                if self._check_collision_free(direct_path):
                    max_feasible_j = j
            
            # Add the furthest feasible point
            if max_feasible_j > i + 1:  # If we can skip points
                # If we're skipping more than one point, add additional intermediate points 
                # for very long segments to maintain path density
                if max_feasible_j - i > 5:
                    # Calculate direct path between furthest points
                    start_pt = path[i]
                    end_pt = path[max_feasible_j]
                    dist = math.sqrt((end_pt[0] - start_pt[0])**2 + 
                                    (end_pt[1] - start_pt[1])**2)
                    
                    # For long segments, add intermediate points
                    if dist > self.turning_radius * 5:
                        num_intermediates = min(5, max(1, int(dist / (self.turning_radius * 2))))
                        for k in range(1, num_intermediates):
                            t = k / num_intermediates
                            x = start_pt[0] + t * (end_pt[0] - start_pt[0])
                            y = start_pt[1] + t * (end_pt[1] - start_pt[1])
                            
                            # Smoothly interpolate heading
                            heading_diff = normalize_angle(end_pt[2] - start_pt[2])
                            theta = normalize_angle(start_pt[2] + t * heading_diff)
                            
                            shortened_path.append((x, y, theta))
                
                # Add the furthest point
                shortened_path.append(path[max_feasible_j])
                i = max_feasible_j
            else:
                # We can't skip, add the next point
                shortened_path.append(path[i + 1])
                i += 1
        
        return shortened_path
    
    def _ensure_minimal_path_density(self, path, min_step_size):
        """Ensure the path has a minimum density of points."""
        if len(path) < 2:
            return path
            
        dense_path = [path[0]]
        
        for i in range(1, len(path)):
            prev = path[i-1]
            curr = path[i]
            
            # Calculate distance between points
            dist = math.sqrt((curr[0] - prev[0])**2 + (curr[1] - prev[1])**2)
            
            # If distance is large, add intermediate points
            if dist > min_step_size * 2:
                num_points = max(2, int(dist / min_step_size))
                
                for j in range(1, num_points):
                    t = j / num_points
                    x = prev[0] + t * (curr[0] - prev[0])
                    y = prev[1] + t * (curr[1] - prev[1])
                    
                    # Interpolate heading
                    heading_diff = normalize_angle(curr[2] - prev[2])
                    theta = normalize_angle(prev[2] + t * heading_diff)
                    
                    dense_path.append((x, y, theta))
            
            # Add current point
            dense_path.append(curr)
        
        return dense_path

    def _visualize_tree(self, title="RRT* Tree", show_goal_path=False):
        """
        Visualize the RRT* tree.
        
        Args:
            title: Plot title
            show_goal_path: Whether to highlight the path to the goal
        """
        import matplotlib.pyplot as plt
        
        plt.figure(figsize=(12, 12))
        plt.imshow(self.grid.T, origin='lower', cmap='gray')
        
        # Plot all nodes and edges
        for node in self.nodes:
            if node.parent:
                # Plot path from parent
                if node.path_from_parent:
                    xs = [p[0] for p in node.path_from_parent]
                    ys = [p[1] for p in node.path_from_parent]
                    
                    # Insert parent position at beginning
                    xs.insert(0, node.parent.x)
                    ys.insert(0, node.parent.y)
                    
                    plt.plot(xs, ys, 'b-', alpha=0.3, linewidth=0.5)
        
        # Plot nodes
        node_xs = [node.x for node in self.nodes]
        node_ys = [node.y for node in self.nodes]
        plt.scatter(node_xs, node_ys, c='blue', s=2, alpha=0.5)
                    
        # Plot start and goal
        plt.scatter(self.start.x, self.start.y, color='green', s=100, marker='o', label='Start')
        plt.scatter(self.goal.x, self.goal.y, color='red', s=100, marker='x', label='Goal')
        
        # Plot goal path if requested
        if show_goal_path and self.goal.parent:
            node = self.goal
            path_nodes = []
            
            while node is not None:
                path_nodes.insert(0, node)
                node = node.parent
                
            # Plot paths between consecutive nodes
            for i in range(1, len(path_nodes)):
                curr_node = path_nodes[i]
                if curr_node.path_from_parent:
                    xs = [p[0] for p in curr_node.path_from_parent]
                    ys = [p[1] for p in curr_node.path_from_parent]
                    
                    # Insert parent position at beginning
                    xs.insert(0, curr_node.parent.x)
                    ys.insert(0, curr_node.parent.y)
                    
                    plt.plot(xs, ys, 'g-', linewidth=2)
        
        plt.title(title)
        plt.legend()
        plt.grid(False)
        plt.tight_layout()
        plt.show()

    def plan(self, visualize_steps=False, visualize_final=True, quick_viz=False):
        """
        Run the Kinodynamic RRT* algorithm.
        
        Args:
            visualize_steps: Whether to visualize intermediate steps
            visualize_final: Whether to visualize the final tree
            quick_viz: If True, skip full tree visualization and only show path
        
        Returns:
            List of (x, y, theta) poses if path found, None otherwise
        """
        print(f"Starting Kinodynamic RRT* with {self.max_iter} iterations")
        start_time = time.time()
        
        # Reset path found flag
        self.path_found = False
        
        # Main loop
        i = 0
        while i < self.max_iter and not self.path_found:
            # Progress reporting
            if (i+1) % 100 == 0:
                print(f"Iteration {i+1}/{self.max_iter}, nodes: {len(self.nodes)}")
                if visualize_steps and (i+1) % 500 == 0:
                    self._visualize_tree(title=f"RRT* Tree at iteration {i+1}")
            
            # 1. Sample random state
            rand_state = self._random_state()
            
            # 2. Find nearest node
            nearest_node = self._nearest_node(rand_state)
            
            # 3. Steer from nearest node to random state
            if self.parallel and i % self.num_workers == 0:
                # Batch steering for parallel processing
                batch_size = min(self.num_workers, self.max_iter - i)
                tasks = []
                
                for j in range(batch_size):
                    if i + j < self.max_iter:
                        batch_state = self._random_state()
                        batch_nearest = self._nearest_node(batch_state)
                        tasks.append((batch_nearest, batch_state))
                
                # Process in parallel (early termination if path found)
                results = self._parallel_steering(tasks)
                
                # Process results
                for new_node, path in results:
                    # Stop if we already found a path
                    if self.path_found:
                        break
                    
                    # Check if path is collision-free
                    if self._check_collision_free(path):
                        # Add node to the tree
                        self.nodes.append(new_node)
                        
                        # Update KD-Tree
                        self.kdtree = scipy.spatial.KDTree(np.array([(node.x, node.y) for node in self.nodes]))
                        
                        # Rewire tree
                        near_nodes = self._near_nodes(new_node)
                        self._rewire(new_node, near_nodes)
                        
                        # Try to connect to goal
                        if self._try_connect_to_goal(new_node):
                            print(f"Found path to goal at iteration {i}")
                            break
                
                # Exit main loop if path found
                if self.path_found:
                    break
                
                # Skip processed iterations
                i += batch_size
            else:
                # Sequential processing
                new_node, path, _ = self._steer(nearest_node, rand_state)
                
                # Skip if no path or collision
                if new_node is None or not self._check_collision_free(path):
                    i += 1
                    continue
                
                # Add node to the tree
                self.nodes.append(new_node)
                
                # Update KD-Tree with the new node
                self.kdtree = scipy.spatial.KDTree(np.array([(node.x, node.y) for node in self.nodes]))
                
                # Find nodes for potential rewiring
                near_nodes = self._near_nodes(new_node)
                
                # Rewire tree
                self._rewire(new_node, near_nodes)
                
                # Try to connect to goal
                if self._try_connect_to_goal(new_node):
                    print(f"Found path to goal at iteration {i}")
                    break
                    
                i += 1
        
        # Check if a path was found
        if not self.path_found:
            print(f"No path found after {self.max_iter} iterations")
            if visualize_final:
                self._visualize_tree(title="RRT* Tree (No path found)")
            return None
        
        # Extract path
        path = []
        node = self.goal
        
        while node is not None:
            # Add node's path from parent
            if node.path_from_parent:
                # Add in reverse order
                for point in reversed(node.path_from_parent):
                    if not path or point != path[0]:
                        path.insert(0, point)
            # If no path_from_parent (e.g., start node), add the node itself
            elif not path or node.get_state() != path[0]:
                path.insert(0, node.get_state())
            
            node = node.parent
        
        # Apply path shortcutting if enabled
        if self.shortcut_path and path and len(path) > 2:  # Only shortcut if we have enough points
            original_length = len(path)
            path = self._shortcut_path(path)
            print(f"Path shortcutting: {original_length} points -> {len(path)} points")
            
            # Safeguard: ensure we didn't reduce too much
            if len(path) < 3:
                print("Path shortcutting removed too many points, interpolating points")
                path = self._ensure_minimal_path_density(path, self.step_size)
        
        print(f"Path found with {len(path)} points in {time.time() - start_time:.2f} seconds")
        
        if visualize_final:
            if quick_viz and self.path_found:
                # Only visualize the final path, not the full tree
                self._visualize_path_only()
            else:
                self._visualize_tree(title="RRT* Tree with Path", show_goal_path=True)
            
        return path
        
    def _visualize_tree(self, title="RRT* Tree", show_goal_path=False):
        """Visualize the RRT* tree with performance optimizations"""
        plt.figure(figsize=(12, 12))
        plt.imshow(self.grid.T, origin='lower', cmap='gray')
        
        # Performance optimization: Only plot a subset of edges for large trees
        max_edges_to_plot = 5000
        nodes_to_plot = self.nodes
        
        if len(self.nodes) > max_edges_to_plot:
            # Randomly sample nodes to plot (but always include recent ones)
            nodes_to_plot = random.sample(self.nodes[:-100], max_edges_to_plot - 100) + self.nodes[-100:]
        
        # Plot edges for selected nodes
        for node in nodes_to_plot:
            if node.parent:
                # Plot path from parent
                if node.path_from_parent:
                    xs = [p[0] for p in node.path_from_parent]
                    ys = [p[1] for p in node.path_from_parent]
                    
                    # Insert parent position at beginning
                    xs.insert(0, node.parent.x)
                    ys.insert(0, node.parent.y)
                    
                    plt.plot(xs, ys, 'b-', alpha=0.3, linewidth=0.5)
        
        # Plot nodes
        node_xs = [node.x for node in nodes_to_plot]
        node_ys = [node.y for node in nodes_to_plot]
        plt.scatter(node_xs, node_ys, c='blue', s=2, alpha=0.5)
                    
        # Plot start and goal
        plt.scatter(self.start.x, self.start.y, color='green', s=100, marker='o', label='Start')
        plt.scatter(self.goal.x, self.goal.y, color='red', s=100, marker='x', label='Goal')
        
        # Plot goal path if requested
        if show_goal_path and self.goal.parent:
            node = self.goal
            path_nodes = []
            
            while node is not None:
                path_nodes.insert(0, node)
                node = node.parent
                
            # Plot paths between consecutive nodes
            for i in range(1, len(path_nodes)):
                curr_node = path_nodes[i]
                if curr_node.path_from_parent:
                    xs = [p[0] for p in curr_node.path_from_parent]
                    ys = [p[1] for p in curr_node.path_from_parent]
                    
                    # Insert parent position at beginning
                    xs.insert(0, curr_node.parent.x)
                    ys.insert(0, curr_node.parent.y)
                    
                    plt.plot(xs, ys, 'g-', linewidth=2)
        
        plt.title(title)
        plt.legend()
        plt.grid(False)
        plt.tight_layout()
        plt.show()
        
    def _visualize_path_only(self):
        """Visualize only the final path (much faster than full tree)"""
        plt.figure(figsize=(10, 10))
        plt.imshow(self.grid.T, origin='lower', cmap='gray')
        
        # Extract path
        path = []
        node = self.goal
        
        while node is not None:
            # Add node's path from parent
            if node.path_from_parent:
                # Add in reverse order
                for point in reversed(node.path_from_parent):
                    path.insert(0, point)
            # If no path_from_parent, add the node itself
            elif not path or node.get_state() != path[0]:
                path.insert(0, node.get_state())
            
            node = node.parent
        
        # Plot path
        if path:
            xs = [p[0] for p in path]
            ys = [p[1] for p in path]
            plt.plot(xs, ys, 'g-', linewidth=2)
        
        # Plot start and goal
        plt.scatter(self.start.x, self.start.y, color='green', s=100, marker='o', label='Start')
        plt.scatter(self.goal.x, self.goal.y, color='red', s=100, marker='x', label='Goal')
        
        plt.title("Found Path")
        plt.legend()
        plt.tight_layout()
        plt.show()