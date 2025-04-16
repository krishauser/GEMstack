from typing import List, Tuple, Union, Dict
from ..component import Component
from ...state import AllState, VehicleState, Path, Trajectory, Route, ObjectFrameEnum, AgentState, Obstacle, ObjectPose, PhysicalObject
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
        self.vehicle_sim = IntegratorControlSpace(SecondOrderDubinsCar(), T=1, dt=0.1)
        #@TODO create a more standardized way to define the actions
        self._actions = [(2, -0.5), (2, -0.25), (2,0), (2, 0.25), (2,0.5),
                         (0,0), (-1, -0.5), (-1,0), (-1, 0.5)]

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
    vehicle
    def heuristic_cost_estimate(self, state_1, state_2):
        # @TODO Consider creating a more sophisticated heuristic
        """computes the 'direct' distance between two (x,y) tuples.
        The states here are (x,y,theta,v,dtheta,t)
        """
        (x1, y1, theta1, v1, dtheta1,t1) = state_1
        (x2, y2, theta2, v2, dtheta2, t2) = state_2

        return math.hypot(x2 - x1, y2 - y1, 2*(v2-v1))
        #return math.hypot(x2 - x1, y2 - y1, theta2 - theta1, v2-v1, dtheta2-dtheta1)

    def terminal_cost_estimate(self, state_1, state_2):
        # @TODO Consider creating a more sophisticated heuristic
        """computes the 'direct' distance between two (x,y) tuples.
        The states here are (x,y,theta,v,dtheta,t)
        """
        (x1, y1, theta1, v1, dtheta1,t1) = state_1
        (x2, y2, theta2, v2, dtheta2, t2) = state_2

        return math.hypot(x2 - x1, y2 - y1)
    
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
        t = 0

        return (x,y,theta,v,dtheta,t)

    def update(self, state : AllState) -> Trajectory:
        """_summary_

        Args:
            state (AllState): _description_

        Returns:
            Trajectory: _description_
        """
        vehicle = state.vehicle # type: VehicleState
        obstacles = state.obstacles # type: Dict[str, Obstacle]
        agents = state.agents # type: Dict[str, AgentState]
        print(f"Obstacles {obstacles}")
        print(f"Agents {agents}")
        route = state.route
        goal_point = route.points[-1] # I might need to change this to the same frame as the car?
        goal_pose = ObjectPose(frame=ObjectFrameEnum.START,t=15, x=goal_point[0],y=goal_point[1],z=0,yaw=0)
        goal = VehicleState.zero()
        goal.pose = goal_pose
        goal.v = 0

        # Need to parse and create second order dubin car states
        start_state = self.vehicle_state_to_dynamics(vehicle)
        goal_state = self.vehicle_state_to_dynamics(goal)


        # Update the planner
        # self.planner.obstacles = list(obstacles.values())
        self.planner.obstacles = list(agents.values())
        self.planner.vehicle = vehicle

        # Compute the new trajectory and return it 
        res = list(self.planner.astar(start_state, goal_state, 100))
        points = [state[:2] for state in res]
        times = [state[5] for state in res]
        path = Path(frame=vehicle.pose.frame, points=points)
        traj = Trajectory(path.frame,points,times)
        # print("===========================")
        # print(f"Points: {points}")
        # print(f"Times: {times}")
        # route = Path(frame=vehicle.pose.frame, points=points)
        # traj = longitudinal_plan(route, 2, -2, 10, vehicle.v, "milestone")
        # print(traj)
        return traj 

class UnparkingPlanner:
    def __init__(self):
        pass

    def plan(self, start_state, goal_state, map_data=None):
        """
        Generate an unparking maneuver: reverse out, turn, then drive forward.
        
        Parameters:
            start_state: tuple (x, y, theta)
            goal_state: tuple (x, y, theta)
            map_data: placeholder for future obstacle checking

        Returns:
            path: list of (x, y, theta) tuples
        """
        x, y, theta = start_state

        # 1. Reverse a little (backward in -theta direction)
        reverse_dist = -2.0  # meters
        x1 = x + reverse_dist * np.cos(theta)
        y1 = y + reverse_dist * np.sin(theta)
        
        # 2. Curve to align with forward direction
        turn_theta = theta - np.pi / 4  # quick left turn out
        x2 = x1 + 1.5 * np.cos(turn_theta)
        y2 = y1 + 1.5 * np.sin(turn_theta)

        # 3. Continue straight toward goal
        xg, yg, thetag = goal_state
        forward_dist = np.hypot(xg - x2, yg - y2)
        num_points = int(forward_dist / 0.5)
        path = []

        for i in range(num_points + 1):
            alpha = i / num_points
            xf = x2 + alpha * (xg - x2)
            yf = y2 + alpha * (yg - y2)
            path.append((xf, yf, thetag))

        # Full path: start -> reverse -> turn -> straight
        full_path = [
            (x, y, theta),
            (x1, y1, theta),
            (x2, y2, turn_theta)
        ] + path

        return full_path

class UnparkingPlanner(Component):
    def __init__(self):
        self.planner = ParkingSolverSecondOrderDubins()

    def state_inputs(self):
        return ['all']

    def state_outputs(self) -> List[str]:
        return ['trajectory']

    def rate(self):
        return 10.0

    def vehicle_state_to_dynamics(self, vehicle_state: VehicleState) -> Tuple[float, float]:
        x = vehicle_state.pose.x
        y = vehicle_state.pose.y
        theta = vehicle_state.pose.yaw
        v = vehicle_state.v
        dtheta = vehicle_state.heading_rate
        t = 0
        return (x, y, theta, v, dtheta, t)

    def update(self, state: AllState) -> Trajectory:
        vehicle = state.vehicle
        agents = state.agents
        route = state.route

        # Original pose
        start_state = self.vehicle_state_to_dynamics(vehicle)

        # Modify start to back up slightly
        reverse_dist = -2.0
        theta = start_state[2]
        x_rev = start_state[0] + reverse_dist * np.cos(theta)
        y_rev = start_state[1] + reverse_dist * np.sin(theta)
        start_unpark = (x_rev, y_rev, theta, 0, 0, 0)

        # Goal is just forward 10 meters from original position
        forward_dist = 10.0
        x_goal = start_state[0] + forward_dist * np.cos(theta)
        y_goal = start_state[1] + forward_dist * np.sin(theta)
        goal = VehicleState.zero()
        goal.pose = ObjectPose(frame=ObjectFrameEnum.START, x=x_goal, y=y_goal, z=0, yaw=theta)
        goal.v = 0
        goal_state = self.vehicle_state_to_dynamics(goal)

        # Set planner context
        self.planner.obstacles = list(agents.values())
        self.planner.vehicle = vehicle

        # Run A*
        res = list(self.planner.astar(start_unpark, goal_state, 100))
        points = [state[:2] for state in res]
        times = [state[5] for state in res]
        path = Path(frame=vehicle.pose.frame, points=points)
        traj = Trajectory(path.frame, points, times)

        return traj