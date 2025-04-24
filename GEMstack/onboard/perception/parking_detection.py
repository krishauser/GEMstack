import rospy
import numpy as np
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import PointCloud2, Image
from typing import Dict
import sensor_msgs.point_cloud2 as pc2
import ros_numpy
from ultralytics import YOLO
from message_filters import Subscriber, ApproximateTimeSynchronizer
from ..component import Component 
from ...state import AgentState, ObjectPose, ObjectFrameEnum, Obstacle, ObstacleMaterialEnum
from ..interface.gem import GEMInterface
from scipy.spatial import ConvexHull
from .parking_utils import *
from .visualization_utils import *


class ParkingSpotsDetector3D(Component):
    def __init__(self, vehicle_interface: GEMInterface):
        # Init Variables
        self.vehicle_interface = vehicle_interface
        self.goal_parking_spot = None
        self.cone_pts_3d = []
        self.visualization = False

    def spin(self):
        rospy.spin()

    def rate(self) -> float:
        return 10.0  # Hz

    def state_inputs(self) -> list:
        return ['agents']

    def state_outputs(self) -> list:
        return ['goal', 'obstacles']
    
    def order_points_convex_hull(self, points_2d):
        points_np = np.array(points_2d)
        hull = ConvexHull(points_np)
        ordered = [points_np[i] for i in hull.vertices]
        return ordered
    
    def detect_parking_spot(self, cone_3d_centers):
        # Initial variables
        goal_parking_spot = None
        parking_obstacles_pose = []
        parking_obstacles_dim = []

        # Get 2D cone centers
        cone_ground_centers = np.array(cone_3d_centers)
        cone_ground_centers_2D = cone_ground_centers[:, :2]
        ordered_cone_ground_centers_2D = self.order_points_convex_hull(cone_ground_centers_2D)
        # print(f"-----cone_ground_centers_2D: {cone_ground_centers_2D}")
        candidates = find_all_candidate_parking_spots(ordered_cone_ground_centers_2D)
        # print(f"-----candidates: {candidates}")
        if len(candidates) > 0:
            parking_obstacles_pose, parking_obstacles_dim = get_parking_obstacles(ordered_cone_ground_centers_2D)
            # print(f"-----parking_obstacles: {self.parking_obstacles_pose}")
            goal_parking_spot = select_best_candidate(candidates, ordered_cone_ground_centers_2D)
            # print(f"-----goal_parking_spot: {self.goal_parking_spot}")
        return goal_parking_spot, parking_obstacles_pose, parking_obstacles_dim, ordered_cone_ground_centers_2D

    def update(self, agents: Dict[str, AgentState]):
        # Initial variables
        goal_parking_spot = None
        parking_obstacles_pose = []
        parking_obstacles_dim = []
        ordered_cone_ground_centers_2D = []

        # Populate cone points
        cone_pts_3D = []
        for cone in agents.values():
            cone_pt_3D = (cone.pose.x, cone.pose.y, 0.0)
            cone_pts_3D.append(cone_pt_3D)

        # Detect parking spot
        if len(cone_pts_3D) >= 4:
            goal_parking_spot, parking_obstacles_pose, parking_obstacles_dim, ordered_cone_ground_centers_2D = self.detect_parking_spot(cone_pts_3D)

        # Return if no goal parking spot is found
        if not goal_parking_spot:
            return None
        
        # Constructing parking obstacles
        current_time = self.vehicle_interface.time()
        obstacle_id = 0
        parking_obstacles = {}
        for o_pose, o_dim in zip(parking_obstacles_pose, parking_obstacles_dim):
            x, y, z, yaw = o_pose
            obstacle_pose = ObjectPose(
                                t=current_time,
                                x=x,
                                y=y,
                                z=z,
                                yaw=yaw,
                                pitch=0.0,
                                roll=0.0,
                                frame=ObjectFrameEnum.CURRENT
                            )
            new_obstacle = Obstacle(
                                pose=obstacle_pose,
                                dimensions=o_dim,
                                outline=None,
                                material=ObstacleMaterialEnum.BARRIER,
                                collidable=True
                            )
            parking_obstacles[obstacle_id] = new_obstacle
            obstacle_id += 1
        
        # Constructing goal pose
        x, y, yaw = goal_parking_spot
        goal_pose = ObjectPose(
                        t=current_time,
                        x=x,
                        y=y,
                        z=0.0,
                        yaw=yaw,
                        pitch=0.0,
                        roll=0.0,
                        frame=ObjectFrameEnum.CURRENT
                    )
        
        new_state = [goal_pose, parking_obstacles]
        return new_state