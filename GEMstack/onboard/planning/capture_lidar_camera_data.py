from ...state import AllState, VehicleState, ObjectPose, ObjectFrameEnum, AgentState, AgentEnum, AgentActivityEnum, MissionEnum
from ..interface.gem import GEMInterface
from ..component import Component
import cv2
from typing import Dict
import numpy as np
import rospy
from sensor_msgs.msg import PointCloud2, Image
import sensor_msgs.point_cloud2 as pc2
import struct, ctypes
from message_filters import Subscriber, ApproximateTimeSynchronizer
from cv_bridge import CvBridge
import time
from ...offboard.log_management.s3 import push_folder_to_s3
import math
import pickle
from datetime import datetime
import os


def pc2_to_numpy(pc2_msg, want_rgb=False):
    """
    Convert a ROS PointCloud2 message into a numpy array with basic filtering.

    This function extracts the x, y, z coordinates from the point cloud and
    filters out points with x <= 0 and z >= 2.5.

    Args:
        pc2_msg: The ROS PointCloud2 message.
        want_rgb (bool): Flag to indicate if RGB data is desired (not used here).

    Returns:
        np.ndarray: Filtered point cloud array of shape (N, 3).
    """
    gen = pc2.read_points(pc2_msg, skip_nans=True)
    pts = np.array(list(gen), dtype=np.float32)
    pts = pts[:, :3]  # Only x, y, z coordinates
    mask = (pts[:, 0] > 0) & (pts[:, 2] < 2.5)
    return pts[mask]


def rad_to_deg(rad):
    """Convert radians to degrees"""
    return rad * 180.0 / math.pi


class SaveInspectionData(Component):
    """The component helps in saving lidar, camera and gnss data when the vehicle is in inspection mode."""
    def __init__(self, vehicle_interface: GEMInterface):
        self.vehicle_interface = vehicle_interface
        self.current_agents = {}
        self.tracked_agents = {}
        self.pedestrian_counter = 0

        self.latest_fr_image = None
        self.latest_rr_image = None
        self.latest_lidar = None
        self.latitude = None
        self.longitude = None
        self.altitude = None
        self.bridge = CvBridge()

        self.index = 0
        timestamp = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
        self.folder_path = f'./inspection_data_{timestamp}'
        os.makedirs(self.folder_path, exist_ok=True)

    def rate(self) -> float:
        return 8.0

    def state_inputs(self) -> list:
        return ['all']

    def state_outputs(self) -> list:
        return []

    def initialize(self):
        # Instead of individual subscriptions, use message_filters to synchronize
        self.fr_image = Subscriber('/camera_fr/arena_camera_node/image_raw', Image)
        self.rr_image = Subscriber('/camera_rr/arena_camera_node/image_raw', Image)
        self.lidar_sub = Subscriber('/ouster/points', PointCloud2)
        self.gnss_sub = Subscriber('/septentrio_gnss/insnavgeod', PointCloud2)

        self.sync = ApproximateTimeSynchronizer([self.fr_image,
                                                 self.rr_image,
                                                 self.lidar_sub,
                                                 self.gnss_sub],
                                                queue_size=10, slop=0.1)
        self.sync.registerCallback(self.synchronized_callback)

    def synchronized_callback(self, fr_image_msg, rr_image_msg, lidar_msg, gnss_msg):
        """
        This callback is triggered when both an image and a LiDAR message arrive within the slop.
        It stores the latest synchronized sensor data for processing in update().
        """
        # Convert the image message to an OpenCV image (assuming it is already in cv2.Mat format or convert as needed)
        try:
            # Convert the ROS Image message to an OpenCV image (BGR format)
            self.latest_fr_image = self.bridge.imgmsg_to_cv2(fr_image_msg, "bgr8")
            self.latest_rr_image = self.bridge.imgmsg_to_cv2(rr_image_msg, "bgr8")
        except Exception as e:
            rospy.logerr("Failed to convert image: {}".format(e))
            self.latest_fr_image = None
            self.latest_rr_image = None
        # Convert the LiDAR message to a numpy array
        self.latest_lidar = pc2_to_numpy(lidar_msg, want_rgb=False)

        # Get gnss data
        self.latitude = rad_to_deg(gnss_msg.latitude)
        self.longitude = rad_to_deg(gnss_msg.longitude)
        self.altitude = gnss_msg.height
        self.yaw = gnss_msg.heading
        self.pitch = gnss_msg.pitch
        self.roll = gnss_msg.roll

    def update(self, state) -> Dict[str, AgentState]:
        # Process only if synchronized sensor data is available
        if self.latest_fr_image is None and self.latest_rr_image is None or self.latest_lidar is None:
            return

        if state.mission.type == MissionEnum.INSPECT:
            lidar_pc = self.latest_lidar.copy()
            camera_fr = self.latest_fr_image.copy()
            camera_rr = self.latest_rr_image.copy()
            cv2.imwrite(os.path.join(self.folder_path, f'fr_{self.index}.png'), camera_fr)
            cv2.imwrite(os.path.join(self.folder_path, f'rr_{self.index}.png'), camera_rr)

            # Save GNSS data
            with open(os.path.join(self.folder_path, 'gnss.txt'), 'a') as fh:
                fh.write(f'{self.latitude},{self.longitude},{self.altitude},{self.yaw}, {self.roll}, {self.pitch}\n')

            # Save LIDAR point cloud
            with open(os.path.join(self.folder_path, f'pc_{self.index}.b'), 'wb') as fh:
                pickle.dump(lidar_pc, fh)

            self.index += 1

        elif state.mission.type == MissionEnum.INSPECT_UPLOAD:
            # upload lidar and camera to s3
            # Currently this is a sync function. Can be converted into async to avoid blocking call
            push_folder_to_s3(self.folder_path, "cs588", "inspect_results")
