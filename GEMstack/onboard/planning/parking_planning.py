from typing import List, Tuple, Union, Dict
from ..component import Component
from ...state import AllState, VehicleState, Path, Trajectory, Route, ObjectFrameEnum, AgentState, Obstacle, ObjectPose
from ...utils import serialization, settings
from ...mathutils.transforms import vector_madd
from ...mathutils.quadratic_equation import quad_root
from ...mathutils.dubins import SecondOrderDubinsCar
from ...mathutils.dynamics import IntegratorControlSpace
from ...mathutils import collisions
from .astar import AStar
from .longitudinal_planning import longitudinal_plan


import numpy as np
import math

# @TODO Need to change the functions here to use VehicleState
class ParkingSolverSecondOrderDubins(AStar):
    """sample use of the astar algorithm. In this exemple we work on a maze made of ascii characters,
    and a 'node' is just a (x,y) tuple that represents a reachable position"""

    def __init__(self, vehicle=None, obstacles=None, actions=None):
        self._obstacles = obstacles

        # Vehicle model
        self._vehicle = None

        # SecondOrderDubinsCar() #x = (tx,ty,theta,v,dtheta) and u = (fwd_accel,wheel_angle_rate)
        self.vehicle_sim = IntegratorControlSpace(SecondOrderDubinsCar(), 1, 0.1)
        #@TODO create a more standardized way to define the actions
        self._actions = [(1, -1), (1, -0.5), (1,0), (1, 0.5), (1,1),
                        (-1, -1), (-1, -0.5), (-1,0), (-1, 0.5), (-1,1)]

    @property
    def vehicle(self):
        return self._vehicle
    
    @vehicle.setter
    def vehicle(self, vehicle):
        self._vehicle = vehicle

    @property
    def obstacles(self):
        return self._obstacles
    
    @obstacles.setter
    def obstacles(self, obstacles):
        self._obstacles = obstacles

    @property
    def actions(self):
        return self._actions
    
    @actions.setter
    def actions(self, actions):
        # @TODO Add a validity checker for the actions with respect to the vehicle constraints
        self._actions = actions

    def is_goal_reached(self, current: List[float], goal: List[float]):
        # @TODO Currently, the threshold is just a random number, get rid of magic constants
        print(f"Current Pose: {current}")
        print(f"Goal Pose: {goal}")
        if np.abs(current[3]) > 0: return False # car must be stopped, this equality will only work in simulation  
        return np.linalg.norm(np.array([current[0], current[1]]) - np.array([goal[0], goal[1]])) < 1
    vehicle
    def heuristic_cost_estimate(self, vehicle_state_1, vehicle_state_2):
        # @TODO Consider creating a more sophisticated heuristic
        """computes the 'direct' distance between two (x,y) tuples"""

        return math.hypot(vehicle_state_2[0] - vehicle_state_1[0], vehicle_state_2[1] - vehicle_state_1[1])
        #return math.hypot(x2 - x1, y2 - y1, theta2 - theta1, v2-v1, dtheta2-dtheta1)

    def distance_between(self, n1, n2):
        """this method always returns 1, as two 'neighbors' are always adajcent"""
        return 1

    def neighbors(self, node):
        """ for a given configuration of the car in the maze, returns up to 4 adjacent(north,east,south,west)
            nodes that can be reached (=any adjacent coordinate that is not a wall)
        """
        neighbors = []
        print(f"Node: {node}")
        for control in self.actions:
            next_state = self.vehicle_sim.nextState(node, control)
            next_state = np.round(next_state, 3)
            if self.is_valid_neighbor([next_state]):
                neighbors.append(tuple(next_state))
        return neighbors
    
    def is_valid_neighbor(self, path):
        """check if any points along the path are in collision
        with any of the known obstacles
        @TODO We are not currently using the geometry of the vehicle,
        define a geometry for the vehicle and then use polygon_itersects_polygon
        @TODO Consider performing a linear search on the trajectory 
        and checking for collission on each of these configurations

        Args:
            path (_type_): _description_
        """
        for obstacle in self.obstacles:
            for point in path:
                print(point)
                print(obstacle.polygon_parent())
                if collisions.circle_intersects_polygon_2d(point[:-1], 1, obstacle.polygon_parent()):
                    #polygon_intersects_polygon_2d when we have the acutal car geometry
                    return False
        return True
   

class ParkingPlanner(Component):
    """_summary_

    Args:
        Component (_type_): _description_
    """
    def __init__(self):
        # self.route_progress = None
        # self.t_last = None

        # self.mode = settings.get("planning.longitudinal_plan.mode")
        # self.planner = settings.get("planning.longitudinal_plan.planner")
        # self.acceleration = settings.get("planning.longitudinal_plan.acceleration")
        # self.deceleration = settings.get("planning.longitudinal_plan.deceleration")
        # self.desired_speed = settings.get("planning.longitudinal_plan.desired_speed")
        # self.yield_deceleration = settings.get("planning.longitudinal_plan.yield_deceleration")

        # Get the model of the car we are going to be using 
        # Get the obstacles 
        # Define the functions we need 
        # Create the Astar object 
        self.planner = ParkingSolverSecondOrderDubins()

    def state_inputs(self):
        return ['all']

    def state_outputs(self) -> List[str]:
        return ['trajectory']

    def rate(self):
        return 10.0
    
    def vehicle_state_to_dynamics(self, vehicle_state: VehicleState) -> Tuple[float, float]:
        """Takes a vehicle state and outputs the state of a second order dubins car

        Args:
            vehicle_state (VehicleState): _description_

        Returns:
            Tuple[float, float]: _description_
        """
        x = vehicle_state.pose.x
        y = vehicle_state.pose.y
        theta = vehicle_state.pose.yaw # check that this is correct
        v = vehicle_state.v
        dtheta = vehicle_state.heading_rate

        return (x,y,theta,v,dtheta)

    def update(self, state : AllState) -> Trajectory:
        """_summary_

        Args:
            state (AllState): _description_

        Returns:
            Trajectory: _description_
        """
        vehicle = state.vehicle # type: VehicleState
        obstacles = state.obstacles # type: Dict[str, Obstacle]
        route = state.route
        goal_point = route.points[-1] # I might need to change this to the same frame as the car?
        goal_pose = ObjectPose(frame=ObjectFrameEnum.START,t=15, x=goal_point[0],y=goal_point[1],z=0)
        goal = VehicleState.zero()
        goal.pose = goal_pose
        goal.v = 0

        # Need to parse and create second order dubin car states
        start_state = self.vehicle_state_to_dynamics(vehicle)
        goal_state = self.vehicle_state_to_dynamics(goal)


        # Update the planner
        self.planner.obstacles = list(obstacles.values())
        self.planner.vehicle = vehicle

        # Compute the new trajectory and return it 
        res = list(self.planner.astar(start_state, goal_state))
        points = [state[:2] for state in res]
        print("===========================")
        print(f"Points: {points}")
        route = Path(frame=vehicle.pose.frame, points=points)
        traj = longitudinal_plan(route, 2, -2, 10, vehicle.v, "milestone")
        print(traj)
        return traj 
