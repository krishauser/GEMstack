import unittest
import numpy as np
import cv2
import matplotlib.pyplot as plt

class RRT:
    def __init__(self, start, goal, x_bounds, y_bounds, step_size, max_iter, occupancy_grid=None):
        self.start = np.array(start)
        self.goal = np.array(goal)
        self.x_bounds =x_bounds
        self.y_bounds= y_bounds
        self.step_size= step_size
        self.max_iter = max_iter
        self.tree = {tuple(start): None}
        self.occupancy_grid =occupancy_grid

    def random_point(self):
        if np.random.rand() < 0.1:
            return self.goal
            
        x = np.random.uniform(self.x_bounds[0],self.x_bounds[1])
        y = np.random.uniform(self.y_bounds[0],self.y_bounds[1])
        
        return np.array([x, y])

    def nearest_neighbor(self, point):
        return min(self.tree.keys(), key=lambda node:np.linalg.norm(np.array(node) -point))

    def steer(self, from_node, to_point):
        direction = to_point-np.array(from_node)
        distance=np.linalg.norm(direction)
        
        if distance < self.step_size:
            return to_point
            
        return np.array(from_node)+ (direction/distance)*self.step_size

    def is_goal_reached(self, node):
        return np.linalg.norm(np.array(node)- self.goal)<= self.step_size

    def is_collision_free(self, node):
        if self.occupancy_grid is None:
            return True
            
        x, y= int(node[0]),int(node[1])
        
        ok_coords = (0<= x < self.occupancy_grid.shape[0] and 
                   0 <=y < self.occupancy_grid.shape[1])
                    
        return ok_coords and self.occupancy_grid[x,y] == 0

    def check_line_collision(self, from_node, to_node):
        x1,y1 = int(from_node[0]), int(from_node[1])
        x2,y2 = int(to_node[0]),int(to_node[1])
        
        points = np.linspace((x1,y1), (x2,y2), num = 20)
        
        return all(self.is_collision_free(pt)for pt in points)

    def plan(self):
        for attempt in range(self.max_iter):
            rand_pt=self.random_point()
            
            closest = self.nearest_neighbor(rand_pt)
            
            new_node=self.steer(closest, rand_pt)
            
            if(self.is_collision_free(new_node)and 
                self.check_line_collision(closest,new_node)):
                
                self.tree[tuple(new_node)]=closest
                
                if self.is_goal_reached(new_node):
                    self.tree[tuple(self.goal)]=tuple(new_node)
                    return self.construct_path()
                    
        return None

    def construct_path(self):
        path = [tuple(self.goal)]
        
        while path[-1]is not None:
            path.append(self.tree[path[-1]])
            
        return path[::-1][1:]
    
    def visualize(self, path=None):
        plt.imshow(self.occupancy_grid.T,origin='lower',cmap='hot')
        
        plt.scatter(*self.start, color='green',label='Start')
        plt.scatter(*self.goal,color='red', label='Goal')
        
        for node,parent in self.tree.items():
            if parent is not None:
                plt.plot([node[0],parent[0]],[node[1],parent[1]],'blue')
                
        if path:
            path_x,path_y = zip(*path)
            plt.plot(path_x,path_y,'r',linewidth=2,label='Path')
            
        plt.legend()
        plt.show()
        
class TestRRT(unittest.TestCase):
    def setUp(self):
        self.occupancy_grid = np.zeros((10, 10), dtype=int)
        self.rrt = RRT(start=(0, 0), goal=(9, 9), x_bounds=(0, 10), y_bounds=(0, 10), step_size=1.0, max_iter=1000, occupancy_grid=self.occupancy_grid)
    
    def test_random_point_within_bounds(self):
        for _ in range(100):
            point = self.rrt.random_point()
            self.assertTrue(self.rrt.x_bounds[0] <= point[0] <= self.rrt.x_bounds[1])
            self.assertTrue(self.rrt.y_bounds[0] <= point[1] <= self.rrt.y_bounds[1])
    
    def test_nearest_neighbor(self):
        self.rrt.tree = {(0, 0): None, (5, 5): (0, 0), (8, 8): (5, 5)}
        nearest = self.rrt.nearest_neighbor((6, 6))
        self.assertEqual(nearest, (5, 5))
    
    def test_steer(self):
        new_point = self.rrt.steer((0, 0), np.array([2, 2]))
        expected = np.array([0.707, 0.707]) * self.rrt.step_size  # Normalized step
        np.testing.assert_almost_equal(new_point, expected, decimal=2)
    
    def test_is_goal_reached(self):
        self.assertFalse(self.rrt.is_goal_reached((5, 5)))
        self.assertTrue(self.rrt.is_goal_reached((9, 9)))
    
    def test_plan(self):
        path = self.rrt.plan()
        self.assertIsNotNone(path, "RRT failed to find a path")
        self.assertEqual(path[0], (0, 0))  # Start point
        self.assertEqual(path[-1], (9, 9))  # Goal point
        self.rrt.visualize(path)
    
    def test_plan_with_obstacles(self):
        self.occupancy_grid[5, :3] = 1  # Add an obstacle in the middle
        self.occupancy_grid[5, 7:10] = 1
        self.occupancy_grid[:3, 5] = 1
        self.occupancy_grid[7:, 5] = 1
        self.rrt = RRT(start=(0, 0), goal=(9, 9), x_bounds=(0, 10), y_bounds=(0, 10), step_size=1.0, max_iter=1000, occupancy_grid=self.occupancy_grid)
        path = self.rrt.plan()
        self.assertIsNotNone(path, "RRT failed to find a path with obstacles")
        for node in path:
            self.assertTrue(self.rrt.is_collision_free(node), f"Path contains collision at {node}")
        self.rrt.visualize(path)

if __name__ == "__main__":
    unittest.main()