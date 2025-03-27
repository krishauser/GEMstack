#Imports
from typing import List, Tuple, Union
from ..component import Component
from ...state import AllState, VehicleState, EntityRelation, EntityRelationEnum, Path, Trajectory, Route, ObjectFrameEnum, AgentState
from ...utils import serialization, settings
from ...mathutils.transforms import vector_madd
from ...mathutils.quadratic_equation import quad_root

import numpy as np
import queue
import math

class Astar(Component):
    def __init__(self, start: Tuple[float, float, float], goal: Tuple[float, float, float], heuristic):
        # A star specific variables
        self.start = start
        self.goal = goal
        self.heuristic = heuristic

        # graph based planner variables
        self.open_set = set()
        self.came_from = {} # this will be used for reconstructing the path

        self.cost_to_node = {} # essentially g(n) in A* algorithm
        self.cost_to_go = queue.PriorityQueue() # f(n) in A* algorithm

    def get_neighbors(self, current: List[float]) -> List[float]:
        """
        This should be defined somewhere else but I am defining it here for now to test the A* algorithm

        Get the neighbor node from the current node
        :param current: Current node [x,y,theta]
        :param delta: Change in x, y, and theta
        :return: Neighbor node [x,y,theta]
        """
        neighbors = []
        deltas = [[1,0,0],[-1,0,0], [0,1,0], [0,-1,0]]
        for delta in deltas:
            print(f"current: {current}")
            print(f"delta: {delta}")
            neighbors.append((current[0] + delta[0], current[1] + delta[1], current[2] + delta[2]))
        return neighbors

    def reconstruct_path(self, current):
        print(f"RECONSTRUCT PATH========================")
        print(f"came_from: {self.came_from}")
        print(f"current: {current}")
        total_path = [current[1]]
        while True:
            print(f"current: {current}")
            current = self.came_from[current[1]]
            if current is None:
                break
            total_path.append(current[1])
        print(f"FOUND PATH: {total_path[::-1]}")
        return total_path[::-1]

    def __call__(self,start_config: List[float], goal_config: List[float]) -> List[List[float]]:
        """
        Hybrid A* path planning algorithm
        :param start: Start position [x,y, theta]. theta is the start heading in radians
        :param goal: Goal position [x,y, theta]. theta is the goal heading in radians
        :return: List of waypoints [x,y]
        """
        self.open_set.add(start_config)

        self.came_from[start_config] = None
        self.cost_to_node[start_config] = 0
        self.cost_to_go.put((self.heuristic(start_config, goal_config), start_config))

        # Start the A* search
        while self.open_set:
            current = self.cost_to_go.get()
            # print(f"current {current}")
            # print(f"goal config: {goal_config}")
            if current[1] == goal_config:
                return self.reconstruct_path(current)
            
            print(self.open_set)
            self.open_set.remove(current[1])
            for neighbor in self.get_neighbors(current[1]):
                tentative_cost = self.cost_to_node[current[1]] + self.heuristic(current[1], neighbor)
                # print(tentative_cost)
                # print(cost_to_node)
                # print(neighbor)
                # print(cost_to_node.keys())
                if not neighbor in self.cost_to_node.keys() or tentative_cost < self.cost_to_node[neighbor]:
                    self.came_from[neighbor] = current
                    self.cost_to_node[neighbor] = tentative_cost
                    self.cost_to_go.put((tentative_cost + self.heuristic(neighbor, goal_config), neighbor))
                    self.open_set.add(neighbor)

                    if neighbor not in self.open_set:
                        self.open_set.put(neighbor)
        return False
            



