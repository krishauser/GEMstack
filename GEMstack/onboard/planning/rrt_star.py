import unittest
import numpy as np
import matplotlib
matplotlib.use('Agg')  # Use non-interactive backend for file saving.
import matplotlib.pyplot as plt
from scipy.spatial import KDTree
import os

def sample_unit_ball(dim=2):
    """Uniformly sample a point from the unit n-ball."""
    x = np.random.randn(dim)
    x /= np.linalg.norm(x)
    r = np.random.rand() ** (1 / dim)
    return r * x

class InformedRRTStar:
    def __init__(self, start, goal, x_bounds, y_bounds, step_size, max_iter,
                 occupancy_grid, neighbor_radius=None, inflation_radius=1):
        """
        Informed RRT* with KD-tree optimization, informed sampling, and 
        vectorized collision checking using precomputed inflation offsets.
        
        :param start: (x, y) start coordinate.
        :param goal: (x, y) goal coordinate.
        :param x_bounds: (min_x, max_x) sampling region in x.
        :param y_bounds: (min_y, max_y) sampling region in y.
        :param step_size: Maximum distance to extend toward a sample.
        :param max_iter: Maximum number of iterations.
        :param occupancy_grid: 2D numpy array (0 free, nonzero obstacle).
        :param neighbor_radius: Radius for neighbor search (default: 5 * step_size).
        :param inflation_radius: Buffer (in grid cells) around obstacles.
        """
        self.start = np.array(start, dtype=float)
        self.goal = np.array(goal, dtype=float)
        self.x_bounds = x_bounds
        self.y_bounds = y_bounds
        self.step_size = step_size
        self.max_iter = max_iter
        self.occupancy_grid = occupancy_grid

        # Tree: map each node (as tuple) to its parent.
        self.tree = {tuple(self.start): None}
        # Cost from start to each node.
        self.cost = {tuple(self.start): 0.0}

        self.neighbor_radius = neighbor_radius if neighbor_radius is not None else 5.0 * step_size
        self.inflation_radius = inflation_radius
        
        # Precompute inflation offsets for vectorized collision checking.
        self.inflation_offsets = np.array([[dx, dy] 
                                            for dx in range(-inflation_radius, inflation_radius + 1)
                                            for dy in range(-inflation_radius, inflation_radius + 1)])
        
        # For KD-tree nearest neighbor search.
        self.nodes_list = [tuple(self.start)]
        self.kdtree = KDTree(np.array(self.nodes_list))

        # Best solution cost so far.
        self.c_best = np.inf
        # Minimum possible cost from start to goal.
        self.c_min = np.linalg.norm(self.goal - self.start)
        # Initial transformation (will be updated once a solution is found).
        self.L = np.diag([self.c_best / 2, 0])
        self.rotation = self._compute_rotation()

        # Record all attempted edges as tuples: (from, to, success_flag)
        self.attempted_edges = []

    def _compute_rotation(self):
        """Compute the rotation matrix to align the x-axis with (goal - start)."""
        theta = np.arctan2(self.goal[1] - self.start[1], self.goal[0] - self.start[0])
        return np.array([[np.cos(theta), -np.sin(theta)],
                         [np.sin(theta),  np.cos(theta)]])

    def update_kdtree(self):
        """Rebuild the KD-tree from the current nodes_list."""
        self.nodes_list = list(self.tree.keys())
        self.kdtree = KDTree(np.array(self.nodes_list))

    def sample_random_point(self):
        """
        If a solution is found, sample from the informed subset (an ellipse);
        otherwise, sample uniformly.
        """
        if self.c_best < np.inf:
            x_ball = sample_unit_ball(2)
            L1 = self.c_best / 2.0
            L2 = np.sqrt(max(self.c_best ** 2 - self.c_min ** 2, 0)) / 2.0
            L_matrix = np.diag([L1, L2])
            x_rand = self.rotation.dot(L_matrix.dot(x_ball)) + (self.start + self.goal) / 2.0
            x_rand[0] = np.clip(x_rand[0], self.x_bounds[0], self.x_bounds[1])
            x_rand[1] = np.clip(x_rand[1], self.y_bounds[0], self.y_bounds[1])
            return x_rand
        else:
            if np.random.rand() < 0.1:
                return self.goal
            x = np.random.uniform(self.x_bounds[0], self.x_bounds[1])
            y = np.random.uniform(self.y_bounds[0], self.y_bounds[1])
            return np.array([x, y])

    def is_point_free(self, point):
        """
        Check if a point is free using the precomputed inflation offsets.
        This is a vectorized collision check.
        """
        x, y = int(round(point[0])), int(round(point[1]))
        grid_h, grid_w = self.occupancy_grid.shape
        # Compute neighbor coordinates.
        coords = self.inflation_offsets + np.array([x, y])
        # Filter only those within grid bounds.
        valid = (coords[:, 0] >= 0) & (coords[:, 0] < grid_h) & (coords[:, 1] >= 0) & (coords[:, 1] < grid_w)
        valid_coords = coords[valid]
        if np.any(self.occupancy_grid[valid_coords[:, 0], valid_coords[:, 1]] != 0):
            return False
        return True

    def is_collision_free(self, node_a, node_b):
        """Check collision along the straight-line segment from node_a to node_b."""
        dist = np.linalg.norm(node_b - node_a)
        num_samples = int(np.ceil(dist)) * 2
        for i in range(num_samples + 1):
            t = i / num_samples
            point = node_a + t * (node_b - node_a)
            if not self.is_point_free(point):
                return False
        return True

    def steer(self, from_node, to_point):
        """Steer from from_node towards to_point by at most step_size."""
        direction = to_point - from_node
        dist = np.linalg.norm(direction)
        if dist < self.step_size:
            return to_point
        return from_node + (direction / dist) * self.step_size

    def nearest_neighbor(self, sample):
        """Use the KD-tree to find the nearest node to sample."""
        if len(self.nodes_list) == 0:
            return self.start
        distance, index = self.kdtree.query(sample)
        return np.array(self.nodes_list[index])

    def find_near_neighbors(self, new_node):
        """Find all nodes within neighbor_radius of new_node using the KD-tree."""
        indices = self.kdtree.query_ball_point(new_node, self.neighbor_radius)
        return [np.array(self.nodes_list[i]) for i in indices]

    def build_path(self):
        """Reconstruct the path from goal to start using parent pointers."""
        path = [tuple(self.goal)]
        while path[-1] is not None:
            path.append(self.tree[path[-1]])
        return path[::-1][1:]  # Reverse and drop the None

    def plan(self):
        """Main planning loop for Informed RRT*."""
        for i in range(self.max_iter):
            sample = self.sample_random_point()
            nearest = self.nearest_neighbor(sample)
            new_node = self.steer(nearest, sample)
            # Record attempted edge: nearest -> new_node.
            if self.is_collision_free(nearest, new_node):
                self.attempted_edges.append((tuple(nearest), tuple(new_node), True))
            else:
                self.attempted_edges.append((tuple(nearest), tuple(new_node), False))
                continue

            neighbors = self.find_near_neighbors(new_node)
            best_parent = nearest
            best_cost = self.cost[tuple(nearest)] + np.linalg.norm(nearest - new_node)

            for nbr in neighbors:
                if self.is_collision_free(nbr, new_node):
                    self.attempted_edges.append((tuple(nbr), tuple(new_node), True))
                    candidate_cost = self.cost[tuple(nbr)] + np.linalg.norm(nbr - new_node)
                    if candidate_cost < best_cost:
                        best_parent = nbr
                        best_cost = candidate_cost
                else:
                    self.attempted_edges.append((tuple(nbr), tuple(new_node), False))
            self.tree[tuple(new_node)] = tuple(best_parent)
            self.cost[tuple(new_node)] = best_cost
            self.nodes_list.append(tuple(new_node))
            self.update_kdtree()

            for nbr in neighbors:
                if np.array_equal(nbr, best_parent):
                    continue
                if self.is_collision_free(new_node, nbr):
                    self.attempted_edges.append((tuple(new_node), tuple(nbr), True))
                    new_cost = self.cost[tuple(new_node)] + np.linalg.norm(new_node - nbr)
                    if new_cost < self.cost[tuple(nbr)]:
                        self.tree[tuple(nbr)] = tuple(new_node)
                        self.cost[tuple(nbr)] = new_cost
                else:
                    self.attempted_edges.append((tuple(new_node), tuple(nbr), False))

            # Attempt connection to goal.
            if np.linalg.norm(new_node - self.goal) < self.step_size:
                if self.is_collision_free(new_node, self.goal):
                    self.attempted_edges.append((tuple(new_node), tuple(self.goal), True))
                    self.tree[tuple(self.goal)] = tuple(new_node)
                    self.cost[tuple(self.goal)] = self.cost[tuple(new_node)] + np.linalg.norm(new_node - self.goal)
                    if self.cost[tuple(self.goal)] < self.c_best:
                        self.c_best = self.cost[tuple(self.goal)]
                        L1 = self.c_best / 2.0
                        L2 = np.sqrt(max(self.c_best ** 2 - self.c_min ** 2, 0)) / 2.0
                        self.L = np.diag([L1, L2])
                        self.rotation = self._compute_rotation()
                        print(f"Iteration {i}: Found a solution with cost {self.c_best:.2f}")
                        return self.build_path()  # Early exit for demonstration.
                else:
                    self.attempted_edges.append((tuple(new_node), tuple(self.goal), False))
        return None

    def visualize_and_save(self, path=None, filename="rrt_visualization.png"):
        """
        Visualize the occupancy grid, all attempted edges, accepted tree edges, and
        the final solution path (if available), then save the figure as an image file.
        """
        plt.figure(figsize=(8, 8))
        plt.imshow(self.occupancy_grid.T, origin="lower", cmap="Greys", interpolation="none")
        plt.scatter(self.start[0], self.start[1], c='green', s=100, label='Start', marker='o')
        plt.scatter(self.goal[0], self.goal[1], c='red', s=100, label='Goal', marker='*')
        
        # Plot all attempted edges.
        for (frm, to, success) in self.attempted_edges:
            frm = np.array(frm)
            to = np.array(to)
            if success:
                plt.plot([frm[0], to[0]], [frm[1], to[1]], "c-", linewidth=0.5, alpha=0.5)
            else:
                plt.plot([frm[0], to[0]], [frm[1], to[1]], "k--", linewidth=0.5, alpha=0.5)
        
        # Overplot accepted tree edges.
        for node, parent in self.tree.items():
            if parent is not None:
                plt.plot([node[0], parent[0]], [node[1], parent[1]], "b-", linewidth=1.5)
        
        # Highlight the solution path if available.
        if path:
            px, py = zip(*path)
            plt.plot(px, py, 'r', linewidth=3, label='Solution Path')
        
        plt.title("Informed RRT* with All Attempted Connections")
        plt.xlabel("X")
        plt.ylabel("Y")
        plt.legend()
        
        full_path = os.path.abspath(filename)
        print(f"Saving visualization to: {full_path}")
        plt.savefig(full_path, bbox_inches='tight', dpi=300)
        plt.close()

# ---------------- Unit Test Code ----------------

def create_free_space_grid(shape=(10, 10)):
    """Return an occupancy grid with no obstacles."""
    return np.zeros(shape, dtype=int)

def create_one_obstacle_grid(shape=(10, 10)):
    """
    Create a grid with one horizontal obstacle.
    For example, block row 5 for columns 0 to 7, leaving a gap at columns 8 and 9.
    """
    grid = np.zeros(shape, dtype=int)
    grid[5, 0:8] = 1
    return grid

def create_complex_obstacle_grid(grid_size=(50, 50)):
    """
    Create a complex occupancy grid with several obstacles.
    """
    grid = np.zeros(grid_size, dtype=int)
    # Vertical wall in the middle with a gap.
    grid[10:40, 25] = 1
    grid[25, 25] = 0
    # Horizontal wall in upper left.
    grid[15, 5:20] = 1
    # Block obstacle in bottom-right.
    grid[35:45, 35:45] = 1
    # Scattered obstacles.
    grid[5:10, 40:42] = 1
    grid[40:43, 10:15] = 1
    return grid

class TestInformedRRTStar(unittest.TestCase):
    # def test_free_space(self):
    #     grid = create_free_space_grid((10, 10))
    #     planner = InformedRRTStar(
    #         start=(0, 0),
    #         goal=(9, 9),
    #         x_bounds=(0, 9),
    #         y_bounds=(0, 9),
    #         step_size=1.0,
    #         max_iter=2000,
    #         occupancy_grid=grid,
    #         inflation_radius=1,
    #         neighbor_radius=3.0
    #     )
    #     path = planner.plan()
    #     self.assertIsNotNone(path, "No path found in free space.")
    #     planner.visualize_and_save(path, filename="/Users/jeffbyju/Documents/GEMstack/GEMstack/onboard/planning/free_space_rrt.png")
    #     print("Free space visualization saved to /Users/jeffbyju/Documents/GEMstack/GEMstack/onboard/planning/free_space_rrt.png")
        
    # def test_one_obstacle(self):
    #     grid = create_one_obstacle_grid((10, 10))
    #     planner = InformedRRTStar(
    #         start=(0, 0),
    #         goal=(9, 9),
    #         x_bounds=(0, 9),
    #         y_bounds=(0, 9),
    #         step_size=1.0,
    #         max_iter=2000,
    #         occupancy_grid=grid,
    #         inflation_radius=1,
    #         neighbor_radius=3.0
    #     )
    #     path = planner.plan()
    #     self.assertIsNotNone(path, "No path found with one obstacle.")
    #     planner.visualize_and_save(path, filename="/Users/jeffbyju/Documents/GEMstack/GEMstack/onboard/planning/one_obstacle_rrt.png")
    #     print("One obstacle visualization saved to /Users/jeffbyju/Documents/GEMstack/GEMstack/onboard/planning/one_obstacle_rrt.png")
        
    def test_complex_obstacle(self):
        grid = create_complex_obstacle_grid((50, 50))
        planner = InformedRRTStar(
            start=(5, 5),
            goal=(45, 45),
            x_bounds=(0, 49),
            y_bounds=(0, 49),
            step_size=1.5,
            max_iter=10000,
            occupancy_grid=grid,
            inflation_radius=1,
            neighbor_radius=4.0
        )
        path = planner.plan()
        if path is None:
            print("No solution found in complex environment; saving visualization of attempted paths.")
            planner.visualize_and_save(path=None, filename="/Users/jeffbyju/Documents/GEMstack/GEMstack/onboard/planning/complex_rrt_attempts.png")
        self.assertIsNotNone(path, "No path found in the complex obstacle environment.")
        if path is not None:
            planner.visualize_and_save(path, filename="/Users/jeffbyju/Documents/GEMstack/GEMstack/onboard/planning/complex_rrt_solution.png")
            print("Complex obstacle solution visualization saved to /Users/jeffbyju/Documents/GEMstack/GEMstack/onboard/planning/complex_rrt_solution.png")

if __name__ == "__main__":
    unittest.main(argv=['first-arg-is-ignored'], exit=False)
