#needed to import GEMstack from top level directory
import sys
import os
sys.path.append(os.getcwd())

from GEMstack.state import Path, ObjectFrameEnum
from GEMstack.onboard.planning.hybrid_astar import Astar, AstarHybrid
from GEMstack.onboard.planning.astar import AStar
from GEMstack.mathutils.dubins import DubinsCar, DubinsCarIntegrator
from GEMstack.state.physical_object import PhysicalObject, ObjectPose, ObjectFrameEnum
import GEMstack.mathutils.collisions as collisions

import matplotlib.pyplot as plt
import numpy as np
import math
import unittest

def gen_obstacle(num_obstacles = 1, xrange = 5, yrange=5):
    obstacles = []
    for _ in range(num_obstacles):
        pose = ObjectPose(frame=ObjectFrameEnum(3),
                        t=0, x = np.random.uniform(0,xrange), y=np.random.uniform(0,yrange))
        dimensions = (1,1,1)
        obstacles.append(PhysicalObject(pose, dimensions, None))

    return obstacles

class ParkingSolver(AStar):
    """sample use of the astar algorithm. In this exemple we work on a maze made of ascii characters,
    and a 'node' is just a (x,y) tuple that represents a reachable position"""

    def __init__(self, obstacles):
        self.obstacles = obstacles

        self.vehicle = DubinsCar() #x = (tx,ty,theta) and u = (fwd_velocity,turnRate).
        self.vehicle_sim = DubinsCarIntegrator(self.vehicle, 1, 0.1)
        self.actions = [(1, -1), (1, -0.5), (1,0), (1, 0.5), (1,1)]

    def is_goal_reached(self, current, goal):
        return np.linalg.norm(np.array(current) - np.array(goal)) < .5
    
    def heuristic_cost_estimate(self, n1, n2):
        """computes the 'direct' distance between two (x,y) tuples"""
        (x1, y1, theta_1) = n1
        (x2, y2, theta_2) = n2
        return math.hypot(x2 - x1, y2 - y1, theta_2 - theta_1)

    def distance_between(self, n1, n2):
        """this method always returns 1, as two 'neighbors' are always adajcent"""
        return 1

    def neighbors(self, node):
        """ for a given configuration of the car in the maze, returns up to 4 adjacent(north,east,south,west)
            nodes that can be reached (=any adjacent coordinate that is not a wall)
        """
        neighbors = []
        for control in self.actions:
            next_state = self.vehicle_sim.nextState(node, control)
            next_state = np.round(next_state, 3)
            if self.is_valid_neighbor([next_state]):
                neighbors.append(tuple(next_state))
        return neighbors
    
    def is_valid_neighbor(self, path):
        """check if any points along the path are in collision
        with any of the known obstacles

        Args:
            path (_type_): _description_
        """
        for obstacle in self.obstacles:
            for point in path:
                # print(point)
                # print(obstacle.polygon_parent())
                if collisions.circle_intersects_polygon_2d(point[:-1], 1, obstacle.polygon_parent()):
                    return False
        return True
    
def solve():
    # generate obstacle
    obstacles = gen_obstacle(3)

    start = (0, 0, 0)  # we choose to start at the upper left corner
    goal = (5, 5, 0)  # we want to reach the lower right corner

    # let's solve it
    foundPath = list(ParkingSolver(obstacles).astar(start, goal))

    plot_path(obstacles, foundPath)

    return list(foundPath)

def plot_path(obstacles, path):
    x = [point[0] for point in path]
    y = [point[1] for point in path]
    plt.plot(x,y)

    for obstacle in obstacles:
        l, w, h = obstacle.dimensions
        center = [obstacle.pose.x, obstacle.pose.y]

        vertices = np.array([
        [center[0] - w / 2, center[1] - l / 2],  # Bottom-left
        [center[0] + w / 2, center[1] - l / 2],  # Bottom-right
        [center[0] + w / 2, center[1] + l / 2],  # Top-right
        [center[0] - w / 2, center[1] + l / 2],  # Top-left
        [center[0] - w / 2, center[1] - l / 2]   # Close the polygon
    ])

        # Plot the polygon
        plt.plot(vertices[:, 0], vertices[:, 1], 'b-', linewidth=2)
        plt.fill(vertices[:, 0], vertices[:, 1], 'b', alpha=0.3)  # Optional fill
        #plt.scatter(*center, color='red', label="Center")  # Mark the center
        
    plt.axis('equal')
    #plt.legend()
    plt.grid(True)
    plt.show()

if __name__ == '__main__':
    solution = solve()
    print(solution)