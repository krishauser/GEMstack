from typing import List, Tuple, Union, Dict
from ..component import Component
from ...state import AllState, VehicleState, Path, Trajectory, Route, ObjectFrameEnum, AgentState, Obstacle, ObjectPose, PhysicalObject
from ...utils import serialization, settings
from ...mathutils.transforms import vector_madd
from GEMstack.mathutils.dubins import DubinsCar, SecondOrderDubinsCar, DubinsCarIntegrator
from ...mathutils.dynamics import IntegratorControlSpace
from ...mathutils import collisions
from .astar import AStar
from .longitudinal_planning import longitudinal_plan
from . import reed_shepp
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import rospy
import time



import numpy as np
import math


def generate_action_set(max_vel, max_turn_rate):
            return [
                (max_vel, -max_turn_rate), (max_vel, 0.0), (max_vel, max_turn_rate),
                (-max_vel, -max_turn_rate), (-max_vel, 0.0), (-max_vel, max_turn_rate)
            ]
class ParkingSolverFirstOrderDubins(AStar):
    """sample use of the astar algorithm. In this exemple we work on a maze made of ascii characters,
    and a 'node' is just a (x,y) tuple that represents a reachable position"""

    def __init__(self, vehicle=None, obstacles=None, actions=None):
        self._obstacles = obstacles

        # Vehicle model
        self._vehicle = None

        # settings
        self.T = settings.get("run.drive.planning._route_planner.dubins.integrator.time_step", 1.5)
        self.dt = settings.get("run.drive.planning._route_planner.dubins.integrator.dt", .25)
        self.max_v = settings.get("run.drive.planning._route_planner.dubins.actions.max_velocity", 0.75)
        self.max_turn_rate = settings.get("run.drive.planning._route_planner.dubins.actions.max_turn_rate", 0.3)
        self.tolerance = settings.get("run.drive.planning._route_planner.dubins.tolerance", 0.5)
        self.heuristic_string = settings.get("run.drive.planning._route_planner.astar.heuristic", "reeds_shepp")
        
        self.vehicle = DubinsCar() #x = (tx,ty,theta) and u = (fwd_velocity,turnRate).
        self.vehicle_sim = DubinsCarIntegrator(self.vehicle, self.T, self.dt)
        self._actions = generate_action_set(self.max_v, self.max_turn_rate)


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
        return np.linalg.norm(np.array([current[0], current[1]]) - np.array([goal[0], goal[1]])) < 0.5
    
    def heuristic_cost_estimate(self, state_1, state_2):
        """run the correct heuristic function based on the heuristic_string"""

        if self.heuristic_string == "reeds_shepp":
            return self.reed_shepp(state_1, state_2) # Using turning radius of 1.0
    
        elif self.heuristic_string == "approx_reeds_shepp":
            return self.approx_reeds_shepp(state_1, state_2) # Using turning radius of 1.0
        
        elif self.heuristic_string == "euclidian":
            return self.euclidian_cost_esimate(state_1, state_2)
        
        else:
            raise Exception(f"Unknown heuristic function: {self.heuristic_string}")
    
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
    
    def reed_shepp(self, state_1, state_2):
        # Extract position and orientation from states
        (x1, y1, theta1, t1) = state_1
        (x2, y2, theta2, t2) = state_2

        # Create start and goal configurations for Reeds-Shepp
        start = (x1, y1, theta1)
        goal = (x2, y2, theta2)
        
        # Calculate Reeds-Shepp path length
        path = reed_shepp.get_optimal_path(start, goal)

        return reed_shepp.path_length(path)  # Using turning radius of 1.0
    
    def terminal_cost_estimate(self, state_1, state_2):
        """computes the 'direct' distance between two (x,y) tuples"""
        # Extract position and orientation from states
        return self.reed_shepp(state_1, state_2)
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
                # print(f"Accepted: {next_state}")
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
                if collisions.polygon_intersects_polygon_2d(vehicle_object.polygon_parent(), obstacle.polygon_parent()):
                    # print("Collision detected")
                    return False
        return True

    def state_to_object(self, state):
        """Convert a state to a polygon. The state is of the form (x,y,theta,v,dtheta,t)

        Args:
            state (_type_): _description_

        Returns:
            _type_: _description_
        """
        x = state[0]
        y = state[1]
        theta = state[2]
        t = state[3]

        pose = ObjectPose(frame=ObjectFrameEnum.CURRENT,t=t, x=x,y=y,z=0,yaw=theta)
        
        temp_obj = PhysicalObject(pose=pose,
                               dimensions=self.vehicle.to_object().dimensions,
                               outline=self.vehicle.to_object().outline)

        return temp_obj
   

class ParkingPlanner():
    """_summary_

    Args:
        Component (_type_): _description_
    """
    def __init__(self):

        self.planner = ParkingSolverFirstOrderDubins()

        self.iterations = settings.get("run.drive.planning._route_planner.astar.iterations", 200)

    def state_inputs(self):
        return ['all']

    def state_outputs(self) -> List[str]:
        return ['route']

    def rate(self):
        return 1.0
    
    def vehicle_state_to_second_order(self, vehicle_state: VehicleState) -> Tuple[float, float]:
        """Takes a vehicle state and outputs the state of a second order dubins car

        Args:
            vehicle_state (VehicleState): _description_

        Returns:
            Tuple[float, float]: _description_
        """
        vehicle_state.pose = vehicle_state.pose.to_frame()
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
    
    def pose_to_first_order(self, pose: ObjectPose) -> Tuple[float, float]:
        """Takes a vehicle state and outputs the state of a second order dubins car

        Args:
            vehicle_state (VehicleState): _description_

        Returns:
            Tuple[float, float]: _description_
        """
        x = pose.x
        y = pose.y
        theta = pose.yaw # check that this is correct
        t = 0

        return (x,y,theta,t)

    def update(self, state : AllState) -> Route:
        """_summary_

        Args:
            state (AllState): _description_

        Returns:
            Trajectory: _description_
        """
        vehicle = state.vehicle # type: VehicleState
        obstacles = state.obstacles # type: Dict[str, Obstacle]
        agents = state.agents # type: Dict[str, AgentState]
        all_obstacles = {**agents, **obstacles}
        goal_pose = state.parking_goal
        assert goal_pose.frame == ObjectFrameEnum.CURRENT
        print("Routing to goal", goal_pose)
        start_state = state.vehicle.to_frame(ObjectFrameEnum.CURRENT, current_pose = state.vehicle.pose,start_pose_abs = state.start_vehicle_pose)


        start = self.vehicle_state_to_first_order(start_state)
        goal = self.pose_to_first_order(goal_pose)

        # Update the planner
        self.planner.obstacles = list(all_obstacles.values())
        self.planner.vehicle = vehicle

        # Compute the new trajectory and return it 
        print(f"Iteartions ------------------------------ {self.iterations}")
        res = list(self.planner.astar(start, goal, reversePath=False, iterations=self.iterations))
        # points = [state[:2] for state in res] # change here to return the theta as well
        points = []
        for state in res:
            points.append((state[0], state[1]))
        times = [state[3] for state in res]
        path = Path(frame=vehicle.pose.frame, points=points)

        route = Path(frame=vehicle.pose.frame, points=points)
        return route 
    

    from rospy.exceptions import ROSInitException

    def create_trajectory_line_marker(self, traj: Trajectory, frame_id="map", marker_id=0, color=(0.0, 0.0, 1.0, 1.0)) -> Marker:
        marker = Marker()
        marker.header.frame_id = frame_id

        try:
            marker.header.stamp = rospy.Time.now()
        except ROSInitException:
            # fallback to zero time (safe default in RViz)
            marker.header.stamp = rospy.Time(0)

        marker.ns = "trajectory"
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD

        marker.scale.x = 0.1  # Line width
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color

        for pt in traj.points:
            p = Point(x=pt[0], y=pt[1], z=pt[2] if len(pt) > 2 else 0.0)
            marker.points.append(p)

        return marker


    def visualize_trajectory(self, traj: Trajectory, frame_id="vehicle"):
       
        self.marker_pub = rospy.Publisher("trajectory_markers", Marker, queue_size=1, latch=True)
        rospy.sleep(0.5)  # ensure publisher is ready

        line_marker = self.create_trajectory_line_marker(traj, frame_id=frame_id)
        self.marker_pub.publish(line_marker)

