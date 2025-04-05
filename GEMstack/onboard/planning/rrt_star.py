import unittest
import numpy as np
import cv2
import matplotlib.pyplot as plt

class RRT:
    def __init__(self, start, goal, x_bounds, y_bounds, step_size, max_iter, occupancy_grid=None, neighbor_radius=None, 
                 collision_sample_count=50, inflation_radius=1):
        self.start = np.array(start)
        self.goal = np.array(goal)
        self.x_bounds = x_bounds
        self.y_bounds = y_bounds
        self.step_size = step_size
        self.max_iter = max_iter
        self.tree = {tuple(start): None}
        self.occupancy_grid = occupancy_grid
        self.cost = {tuple(start): 0}  # Maintain cost to reach each node
        # Set neighbor radius (if not provided, default to 5 * step_size)
        self.neighbor_radius = neighbor_radius if neighbor_radius is not None else step_size * 5

        # Number of collision samples along a line
        self.collision_sample_count = collision_sample_count 
        
        # Buffer around obstacles in cell units
        self.inflation_radius = inflation_radius

        # OPTIMIZATION: Precompute inflation offsets for vectorized collision checking.
        self.inflation_offsets = np.array([[dx, dy] 
                                           for dx in range(-self.inflation_radius, self.inflation_radius + 1)
                                           for dy in range(-self.inflation_radius, self.inflation_radius + 1)])

    def random_point(self):
        if np.random.rand() < 0.1:
            return self.goal
        x = np.random.uniform(self.x_bounds[0], self.x_bounds[1])
        y = np.random.uniform(self.y_bounds[0], self.y_bounds[1])
        return np.array([x, y])

    def nearest_neighbor(self, point):
        # OPTIMIZATION: Vectorized computation of distances to all tree nodes.
        nodes = np.array(list(self.tree.keys()))
        dists = np.linalg.norm(nodes - point, axis=1)
        idx = np.argmin(dists)
        return tuple(nodes[idx])
    
    def get_near_nodes(self, point, radius):
        # OPTIMIZATION: Vectorized search for nodes within the given radius.
        nodes = np.array(list(self.tree.keys()))
        dists = np.linalg.norm(nodes - point, axis=1)
        near_nodes = nodes[dists < radius]
        return [tuple(node) for node in near_nodes]

    def steer(self, from_node, to_point):
        direction = to_point - np.array(from_node)
        distance = np.linalg.norm(direction)
        if distance < self.step_size:
            return to_point
        return np.array(from_node) + (direction / distance) * self.step_size

    def is_goal_reached(self, node):
        return np.linalg.norm(np.array(node) - self.goal) <= self.step_size

    def is_collision_free(self, node):
        if self.occupancy_grid is None:
            return True
        x, y = int(node[0]), int(node[1])
        h, w = self.occupancy_grid.shape
        if not (0 <= x < h and 0 <= y < w):
            return False
        
        # OPTIMIZATION: Use vectorized check for the neighborhood around the point.
        # Compute the neighbor coordinates by adding precomputed offsets.
        neighbors = self.inflation_offsets + np.array([x, y])
        # Filter neighbors that are within grid bounds.
        valid = (neighbors[:, 0] >= 0) & (neighbors[:, 0] < h) & (neighbors[:, 1] >= 0) & (neighbors[:, 1] < w)
        valid_neighbors = neighbors[valid]
        # Check if any of the valid neighbor cells are obstacles (non-zero)
        if np.any(self.occupancy_grid[valid_neighbors[:, 0], valid_neighbors[:, 1]] != 0):
            return False
        return True

    def check_line_collision(self, from_node, to_node):
        x1, y1 = int(from_node[0]), int(from_node[1])
        x2, y2 = int(to_node[0]), int(to_node[1])
        # Use a configurable number of sample points for finer collision checking
        points = np.linspace((x1, y1), (x2, y2), num=self.collision_sample_count)
        # Check each sampled point using the vectorized collision check in is_collision_free
        return all(self.is_collision_free(pt) for pt in points)

    def plan(self):
        for attempt in range(self.max_iter):
            rand_pt = self.random_point()
            closest = self.nearest_neighbor(rand_pt)
            new_node = self.steer(closest, rand_pt)

            if self.is_collision_free(new_node) and self.check_line_collision(closest, new_node):
                # RRT* "Choose Parent" step:
                near_nodes = self.get_near_nodes(new_node, self.neighbor_radius)
                best_parent = closest
                min_cost = self.cost[tuple(closest)] + np.linalg.norm(new_node - np.array(closest))
                for near in near_nodes:
                    if self.check_line_collision(near, new_node):
                        cost = self.cost[near] + np.linalg.norm(new_node - np.array(near))
                        if cost < min_cost:
                            best_parent = near
                            min_cost = cost

                # Add new_node with best parent and cost update
                self.tree[tuple(new_node)] = best_parent
                self.cost[tuple(new_node)] = min_cost

                # Rewire step
                for near in near_nodes:
                    if near == best_parent:
                        continue
                    if self.check_line_collision(new_node, near):
                        new_cost = self.cost[tuple(new_node)] + np.linalg.norm(np.array(near) - new_node)
                        if new_cost < self.cost[near]:
                            self.tree[near] = tuple(new_node)
                            self.cost[near] = new_cost

                if self.is_goal_reached(new_node):
                    self.tree[tuple(self.goal)] = tuple(new_node)
                    self.cost[tuple(self.goal)] = self.cost[tuple(new_node)] + np.linalg.norm(self.goal - new_node)
                    return self.construct_path()
        return None

    def construct_path(self):
        path = [tuple(self.goal)]
        while path[-1] is not None:
            path.append(self.tree[path[-1]])
        return path[::-1][1:]
    
    def visualize(self, path=None):
        plt.imshow(self.occupancy_grid.T, origin='lower', cmap='hot')
        plt.scatter(*self.start, color='green', label='Start')
        plt.scatter(*self.goal, color='red', label='Goal')
        for node, parent in self.tree.items():
            if parent is not None:
                plt.plot([node[0], parent[0]], [node[1], parent[1]], 'blue')
        if path:
            path_x, path_y = zip(*path)
            plt.plot(path_x, path_y, 'r', linewidth=2, label='Path')
        plt.legend()
        plt.show()
        
class TestRRT(unittest.TestCase):
    def setUp(self):
        self.occupancy_grid = np.zeros((100, 100), dtype=int)
        self.rrt = RRT(start=(0, 0), goal=(99, 25), x_bounds=(0, 100), y_bounds=(0, 100),
                       step_size=1.0, max_iter=1000, occupancy_grid=self.occupancy_grid)
    
    def test_random_point_within_bounds(self):
        for _ in range(100):
            point = self.rrt.random_point()
            self.assertTrue(self.rrt.x_bounds[0] <= point[0] <= self.rrt.x_bounds[1])
            self.assertTrue(self.rrt.y_bounds[0] <= point[1] <= self.rrt.y_bounds[1])
    
    def test_nearest_neighbor(self):
        self.rrt.tree = {(0, 0): None, (5, 5): (0, 0), (8, 8): (5, 5)}
        self.rrt.cost = {(0, 0): 0,
                         (5, 5): np.linalg.norm(np.array((5,5)) - np.array((0,0))),
                         (8, 8): np.linalg.norm(np.array((8,8)) - np.array((5,5)))}
        nearest = self.rrt.nearest_neighbor((6, 6))
        self.assertEqual(nearest, (5, 5))
    
    def test_steer(self):
        new_point = self.rrt.steer((0, 0), np.array([2, 2]))
        expected = np.array([0.707, 0.707]) * self.rrt.step_size  # Normalized step
        np.testing.assert_almost_equal(new_point, expected, decimal=2)
    
    def test_is_goal_reached(self):
        self.assertFalse(self.rrt.is_goal_reached((90, 90)))
        self.assertTrue(self.rrt.is_goal_reached((99, 25)))
    
    def test_plan(self):
        path = self.rrt.plan()
        self.assertIsNotNone(path, "RRT failed to find a path")
        self.assertEqual(path[0], (0, 0))  # Start point
        self.assertEqual(path[-1], (99, 25))  # Goal point
        self.rrt.visualize(path)
    
    def test_plan_with_obstacles(self):
        print("TESTING OBSTACLES")
        self.occupancy_grid[10, :30] = 1 
        self.occupancy_grid[45, 15:75] = 1
        self.occupancy_grid[80, :90] = 1
        self.rrt = RRT(start=(0, 0), goal=(99, 25), x_bounds=(0, 100), y_bounds=(0, 100),
                       step_size=1.0, max_iter=100000, occupancy_grid=self.occupancy_grid, neighbor_radius=3, inflation_radius=5)
        path = self.rrt.plan()
        self.assertIsNotNone(path, "RRT failed to find a path with obstacles")
        for node in path:
            self.assertTrue(self.rrt.is_collision_free(node), f"Path contains collision at {node}")
        self.rrt.visualize(path)

if __name__ == "__main__":
    unittest.main()

