from typing import List, Tuple, Union, Dict
from ..component import Component
from ...state import AllState, VehicleState, Path, Trajectory, Route, ObjectFrameEnum, AgentState, Obstacle, ObjectPose, PhysicalObject
from ...utils import serialization, settings
from ...mathutils.transforms import vector_madd
from ...mathutils.quadratic_equation import quad_root
from GEMstack.mathutils.dubins import DubinsCar, SecondOrderDubinsCar, DubinsCarIntegrator
from ...mathutils.dynamics import IntegratorControlSpace
from ...mathutils import collisions
from .astar import AStar
from .longitudinal_planning import longitudinal_plan
from testing.reeds_shepp_path import path_length


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
        # self.vehicle_sim = IntegratorControlSpace(SecondOrderDubinsCar(), T=2, dt=0.1)
        self.vehicle_sim = IntegratorControlSpace(SecondOrderDubinsCar(), T=0.5, dt=0.2)
        # #@TODO create a more standardized way to define the actions
        # self._actions = [(2, -0.5), (2, -0.25), (2,0), (2, 0.25), (2,0.5),
        #                  (0,0), (-1, -0.5), (-1,0), (-1, 0.5)]
        self._actions = generate_action_set()

        @staticmethod
        def generate_action_set():
            return [
                (1.0, -0.3), (1.0, 0.0), (1.0, 0.3),
                (-1.0, -0.3), (-1.0, 0.0), (-1.0, 0.3)
            ]

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
        # print(f"Current Pose: {current}")
        # print(f"Goal Pose: {goal}")
        if np.abs(current[3]) > 1: return False # car must be stopped, this equality will only work in simulation  
        return np.linalg.norm(np.array([current[0], current[1]]) - np.array([goal[0], goal[1]])) < 0.5
    
    def heuristic_cost_estimate(self, state_1, state_2):
        """
        Computes the Reeds-Shepp path length between two configurations as the heuristic cost.
        The states here are (x,y,theta,v,dtheta,t)
        """
        (x1, y1, theta1, v1, dtheta1, t1) = state_1
        (x2, y2, theta2, v2, dtheta2, t2) = state_2
        
        # Create start and goal configurations for Reeds-Shepp
        start = (x1, y1, theta1)
        goal = (x2, y2, theta2)
        
        # Calculate Reeds-Shepp path length
        path_length_cost = path_length(start, goal, 0.5)  # Using turning radius of 1.0
        
        # Add penalties for velocity and time differences
        velocity_penalty = abs(v2 - v1) * 0.1
        time_penalty = abs(t2 - t1) * 0.01
        
        return path_length_cost + velocity_penalty + time_penalty

    def terminal_cost_estimate(self, state_1, state_2):
        """
        Computes the terminal cost estimate between current state and goal state.
        The states here are (x,y,theta,v,dtheta,t)
        """
        (x1, y1, theta1, v1, dtheta1, t1) = state_1
        (x2, y2, theta2, v2, dtheta2, t2) = state_2
        
        # Create start and goal configurations for Reeds-Shepp
        start = (x1, y1, theta1)
        goal = (x2, y2, theta2)
        
        # Calculate Reeds-Shepp path length
        path_length_cost = path_length(start, goal, 0.5)  # Using turning radius of 1.0
        
        # Add penalties for velocity, angular velocity, and time differences
        velocity_penalty = abs(v2 - v1) * 0.1
        angular_velocity_penalty = abs(dtheta2 - dtheta1) * 0.1
        time_penalty = abs(t2 - t1) * 0.01
        
        return path_length_cost + velocity_penalty + angular_velocity_penalty + time_penalty
    
    def distance_between(self, state_1, state_2):
        """
        The states here are (x,y,theta,v,dtheta,t)
        """
        (x1, y1, theta1, v1, dtheta1,t1) = state_1
        (x2, y2, theta2, v2, dtheta2, t2) = state_2

        return math.hypot(x2 - x1, y2 - y1)

    def neighbors(self, node):
        """ for a given configuration of the car in the maze, returns up to 4 adjacent(north,east,south,west)
            nodes that can be reached (=any adjacent coordinate that is not a wall)
        """
        neighbors = []
        # print(f"Node: {node}")
        for control in self.actions:
            next_state = self.vehicle_sim.nextState(node[:5], control)
            # print(f"next state: {next_state}")
            next_state = np.append(next_state, node[5] + self.vehicle_sim.T)
            next_state = np.round(next_state, 3)
            if self.is_valid_neighbor([next_state]):
                print(f"Accepted: {next_state}")
                neighbors.append(tuple(next_state))
            else:
                print(f"Rejected (collision): {next_state}")
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
            # print(f"Obstacle: {obstacle}")
            for point in path:
                vehicle_polygon = self.state_to_polygon(point)
                # print(f"Vehicle Polygon: {vehicle_polygon}")
                # print(point)
                # print(obstacle.polygon_parent())
                # print("====================================")
                # print(f"Vehicle Polygon: {vehicle_polygon}")
                # print(f"Obstacle Polygon: {obstacle.polygon_parent()}")
                # print(f"Point: {point}")
                if collisions.polygon_intersects_polygon_2d(vehicle_polygon, obstacle.polygon_parent()):
                    #polygon_intersects_polygon_2d when we have the acutal car geometry
                    return False
        return True

    def state_to_polygon(self, state):
        """Convert a state to a polygon. The state is of the form (x,y,theta,v,dtheta,t)

        Args:
            state (_type_): _description_

        Returns:
            _type_: _description_
        """
        x = state[0]
        y = state[1]
        theta = state[2]
        v = state[3]
        dtheta = state[4]
        t = state[5]

        pose = ObjectPose(frame=ObjectFrameEnum.START,t=t, x=x,y=y,z=0,yaw=theta)
        
        temp_obj = PhysicalObject(pose=pose,
                               dimensions=self.vehicle.to_object().dimensions,
                               outline=self.vehicle.to_object().outline)

        return temp_obj.polygon()

def generate_action_set():
    return [
        (1.0, -0.3), (1.0, 0.0), (1.0, 0.3),
        (-1.0, -0.3), (-1.0, 0.0), (-1.0, 0.3)
    ]

# @TODO Need to change the functions here to use VehicleState
class ParkingSolverFirstOrderDubins(AStar):
    """sample use of the astar algorithm. In this exemple we work on a maze made of ascii characters,
    and a 'node' is just a (x,y) tuple that represents a reachable position"""

    def __init__(self, vehicle=None, obstacles=None, actions=None):
        self._obstacles = obstacles

        # Vehicle model
        self._vehicle = None
        
        self.vehicle = DubinsCar() #x = (tx,ty,theta) and u = (fwd_velocity,turnRate).
        self.vehicle_sim = DubinsCarIntegrator(self.vehicle, 2, 0.25)
        #@TODO create a more standardized way to define the actions
        self._actions = generate_action_set()


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
        # print(f"Current Pose: {current}")
        # print(f"Goal Pose: {goal}")
        # if np.abs(current[3]) > 1: return False # car must be stopped, this equality will only work in simulation  
        # return np.linalg.norm(np.array([current[0], current[1]]) - np.array([goal[0], goal[1]])) < 0.5
        pos_dist = np.linalg.norm([current[0] - goal[0], current[1] - goal[1]])
        yaw_dist = abs((current[2] - goal[2] + np.pi) % (2 * np.pi) - np.pi)
        return pos_dist < 0.5 and yaw_dist < 0.3
    
    def heuristic_cost_estimate(self, state_1, state_2):
        """computes the 'direct' distance between two (x,y) tuples"""
        # Extract position and orientation from states
        (x1, y1, theta1, t) = state_1
        (x2, y2, theta2, t) = state_2

        return self.approx_reeds_shepp(state_1, state_2) # Using turning radius of 1.0
        return math.hypot(x2 - x1, y2 - y1, theta2 - theta1)
    
    def euclidian_cost_esimate(self, state_1, state_2):
        """computes the 'direct' distance between two (x,y) tuples"""
        # Extract position and orientation from states
        (x1, y1, theta1, t) = state_1
        (x2, y2, theta2, t) = state_2

        return math.hypot(x2 - x1, y2 - y1, theta2 - theta1)
    
    def approx_reeds_shepp(self, state_1, state_2):
        # Extract position and orientation from states
        (x1, y1, theta1, t1) = state_1
        (x2, y2, theta2, t2) = state_2

        # Create start and goal configurations for Reeds-Shepp
        start = (x1, y1, theta1)
        goal = (x2, y2, theta2)
        
        # Calculate Reeds-Shepp path length
        path_length_cost = path_length(start, goal, 1.0)  # Using turning radius of 1.0
        
        # # Add penalties for velocity and time differences
        # velocity_penalty = abs(v2 - v1) * 0.1
        # time_penalty = abs(t2 - t1) * 0.01
        
        return path_length_cost # + velocity_penalty + time_penalty
    
    def terminal_cost_estimate(self, state_1, state_2):
        """computes the 'direct' distance between two (x,y) tuples"""
        # Extract position and orientation from states
        return self.approx_reeds_shepp(state_1, state_2)

    def distance_between(self, state_1, state_2):
        """
        The states here are (x,y,theta,v,dtheta,t)
        """
        (x1, y1, theta1, t) = state_1
        (x2, y2, theta2, t) = state_2

        return math.hypot(x2 - x1, y2 - y1)

    def neighbors(self, node):
        """ for a given configuration of the car in the maze, returns up to 4 adjacent(north,east,south,west)
            nodes that can be reached (=any adjacent coordinate that is not a wall)
        """
        print(f"[DEBUG] neighbors() called on node {node}")
        neighbors = []
        # print(f"Node: {node}")
        for control in self.actions:
            next_state = self.vehicle_sim.nextState(node[:3], control)
            # print(f"next state: {next_state}")
            next_state = np.append(next_state, node[3] + self.vehicle_sim.T)
            next_state = np.round(next_state, 3)
            if self.is_valid_neighbor([next_state]):
                print(f"Accepted: {next_state}")
                neighbors.append(tuple(next_state))
            else:
                print(f"Rejected first order (collision): {next_state}")
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
            # print(f"Obstacle: {obstacle}")
            for point in path:
                vehicle_object = self.state_to_object(point)
                # vehicle_polygon = vehicle_object.polygon_parent()
                # print(f"Vehicle Polygon: {vehicle_polygon}")
                # print(point)
                # print(obstacle.polygon_parent())
                # print("====================================")
                # print(f"Vehicle Object: {vehicle_object}")
                # print(f"Vehicle Polygon: {vehicle_polygon}")
                # print(f"Obstacle Polygon: {obstacle.polygon_parent()}")
                # print(f"Obstacle: {obstacle}")
                # print(f"Point: {point}")
                # print(f"Obstacle Applied at current point: {obstacle.to_frame(frame=ObjectFrameEnum.CURRENT, current_pose=vehicle_object.pose)}")
                # print(f"Obstacle Polygon at current point: {obstacle.to_frame(frame=ObjectFrameEnum.CURRENT, current_pose=vehicle_object.pose).polygon_parent()}")
                #!!!!! the obstacle is in the world frame!
                if collisions.polygon_intersects_polygon_2d(vehicle_object.polygon_parent(), obstacle.polygon_parent()):
                # if collisions.polygon_intersects_polygon_2d(vehicle_object.polygon_parent(), 
                #     obstacle.to_frame(frame=ObjectFrameEnum.CURRENT, current_pose=vehicle_object.pose).polygon_parent()):
                    #polygon_intersects_polygon_2d when we have the acutal car geometry
                    # raise Exception("Collision detected")
                    print("Collision detected")
                    return False
        return True

    def state_to_object(self, state):
        """Convert a state to a polygon. The state is of the form (x,y,theta,v,dtheta,t)
        Adds a buffer to the vehicle's dimensions for safer collision checking.

        Args:
            state (_type_): _description_

        Returns:
            _type_: _description_
        """
        x = state[0]
        y = state[1]
        theta = state[2]
        t = state[3]

        pose = ObjectPose(frame=ObjectFrameEnum.ABSOLUTE_CARTESIAN,t=t, x=x,y=y,z=0,yaw=theta)
        
        # Get original vehicle dimensions
        original_dimensions = self.vehicle.to_object().dimensions
        
        # Add buffer to dimensions (increase length and width by buffer amount)
        buffer = 0.5  # 0.5 meters buffer on each side
        buffered_dimensions = (
            original_dimensions[0] + 2 * buffer,  # length
            original_dimensions[1] + 2 * buffer,  # width
            original_dimensions[2]  # height remains the same
        )
        
        temp_obj = PhysicalObject(pose=pose,
                               dimensions=buffered_dimensions,
                               outline=self.vehicle.to_object().outline)

        return temp_obj
   

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
        # self.planner = ParkingSolverSecondOrderDubins()
        self.planner = ParkingSolverFirstOrderDubins()

    def state_inputs(self):
        return ['all']

    def state_outputs(self) -> List[str]:
        return ['trajectory']

    def rate(self):
        return 1.0
    
    def vehicle_state_to_second_order(self, vehicle_state: VehicleState) -> Tuple[float, float]:
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
        t = 0

        return (x,y,theta,v,dtheta,t)
    
    def vehicle_state_to_first_order(self, vehicle_state: VehicleState) -> Tuple[float, float]:
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
        t = 0

        return (x,y,theta,t)

    def update(self, state : AllState) -> Trajectory:
        """_summary_

        Args:
            state (AllState): _description_

        Returns:
            Trajectory: _description_
        """
        vehicle = state.vehicle # type: VehicleState
        print(f"Vehicle {vehicle}")
        obstacles = state.obstacles # type: Dict[str, Obstacle]
        agents = state.agents # type: Dict[str, AgentState]
        # print(f"Obstacles {obstacles}")
        print(f"Agents {agents}")
        route = state.route
        goal_point = route.points[-1] # I might need to change this to the same frame as the car?
        goal_pose = ObjectPose(frame=ObjectFrameEnum.ABSOLUTE_CARTESIAN, t=15, x=goal_point[0],y=goal_point[1],z=0,yaw=0)
        goal = VehicleState.zero()
        goal.pose = goal_pose
        goal.v = 0

        # # Need to parse and create second order dubin car states
        # start_state = self.vehicle_state_to_dynamics(vehicle)
        # goal_state = self.vehicle_state_to_dynamics(goal)

        # Need to parse and create second order dubin car states
        start_state = self.vehicle_state_to_first_order(vehicle)
        goal_state = self.vehicle_state_to_first_order(goal)

        # Update the planner
        # self.planner.obstacles = list(obstacles.values())
        self.planner.obstacles = list(agents.values())
        self.planner.vehicle = vehicle

        # Compute the new trajectory and return it 
        res = list(self.planner.astar(start_state, goal_state, reversePath=False, iterations=200))
        # points = [state[:2] for state in res] # change here to return the theta as well
        points = []
        for state in res:
            points.append((state[0], state[1]))
        times = [state[3] for state in res]
        path = Path(frame=vehicle.pose.frame, points=points)
        traj = Trajectory(path.frame,points,times)
        print("===========================")
        print(f"Points: {points}")
        print(f"Times: {times}")
        # route = Path(frame=vehicle.pose.frame, points=points)
        # traj = longitudinal_plan(route, 2, -2, 10, vehicle.v, "milestone")
        print(traj)
        return traj 