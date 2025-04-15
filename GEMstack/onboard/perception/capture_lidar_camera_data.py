from ...state import AllState, VehicleState, ObjectPose, ObjectFrameEnum, AgentState, AgentEnum, AgentActivityEnum
from ..interface.gem import GEMInterface
from ..component import Component
from ultralytics import YOLO
import cv2
from typing import Dict
import open3d as o3d
import numpy as np
from sklearn.cluster import DBSCAN
from scipy.spatial.transform import Rotation as R
import rospy
from sensor_msgs.msg import PointCloud2, Image
import sensor_msgs.point_cloud2 as pc2
import struct, ctypes
from message_filters import Subscriber, ApproximateTimeSynchronizer
from cv_bridge import CvBridge
import time     


class SaveInspectionData(Component):

    def __init__(self, vehicle_interface: GEMInterface):
        self.vehicle_interface = vehicle_interface
        self.current_agents = {}
        self.tracked_agents = {}
        self.pedestrian_counter = 0
        # Variables to store synchronized sensor data:
        self.latest_image = None
        self.latest_lidar = None
        self.bridge = CvBridge()

    def rate(self) -> float:
        return 4.0

    def state_inputs(self) -> list:
        return ['vehicle', 'mission']

    def state_outputs(self) -> list:
        return ['agents']

    def initialize(self):
        # Instead of individual subscriptions, use message_filters to synchronize
        self.fr_image = Subscriber('/oak/rgb/image_raw', Image)
        self.rr_image = Subscriber('/oak/rgb/image_raw', Image)
        self.lidar_sub = Subscriber('/ouster/points', PointCloud2)
        self.sync = ApproximateTimeSynchronizer([self.fr_image, self.rr_image, self.lidar_sub],
                                                queue_size=10, slop=0.1)
        self.sync.registerCallback(self.synchronized_callback)
        # Initialize YOLO detector

        # Set up camera intrinsics and LiDAR-to-camera transformation.
        self.T_l2v = np.array([[ 0.99939639,  0.02547917,  0.023615,    1.1       ],
                                [-0.02530848,  0.99965156, -0.00749882,  0.03773583],
                                [-0.02379784,  0.00689664,  0.999693,    1.95320223],
                                [ 0.,          0.,          0.,          1.        ]])
        self.K = np.array([[684.83331299, 0., 573.37109375],
                           [0., 684.60968018, 363.70092773],
                           [0., 0., 1.]])
        self.T_l2c = np.array([
            [0.001090, -0.999489, -0.031941,  0.149698],
            [-0.007664,  0.031932, -0.999461, -0.397813],
            [0.999970,  0.001334, -0.007625, -0.691405],
            [0.000000,  0.000000,  0.000000,  1.000000]
        ])
        self.T_c2l = np.linalg.inv(self.T_l2c)
        self.R_c2l = self.T_c2l[:3, :3]
        self.camera_origin_in_lidar = self.T_c2l[:3, 3]
        

    def synchronized_callback(self, fr_image_msg, rr_image_msg, lidar_msg):
        """
        This callback is triggered when both an image and a LiDAR message arrive within the slop.
        It stores the latest synchronized sensor data for processing in update().
        """
        # Convert the image message to an OpenCV image (assuming it is already in cv2.Mat format or convert as needed)
            # Convert the ROS Image message to an OpenCV image (BGR format)
        try:
            # Convert the ROS Image message to an OpenCV image (BGR format)
            self.latest_image_fr = self.bridge.imgmsg_to_cv2(fr_image_msg, "bgr8")
            self.latest_image_rr = self.bridge.imgmsg_to_cv2(rr_image_msg, "bgr8")
        except Exception as e:
            rospy.logerr("Failed to convert image: {}".format(e))
            self.latest_image = None
        # Convert the LiDAR message to a numpy array
        self.latest_lidar = pc2_to_numpy(lidar_msg, want_rgb=False)

    def update(self, vehicle: VehicleState, mission) -> Dict[str, AgentState]:
        # Process only if synchronized sensor data is available
        if self.latest_image_fr is None self.latest_image_rr is None or self.latest_lidar is None:
            return {}

        current_time = self.vehicle_interface.time()
        
        elif mission == "INSPECT":
            lidar_pc = self.latest_lidar.copy()
            camera_fr = self.latest_image_fr.copy()
            camera_rr = self.latest_image_rr.copy()
            # save_camera_lidar
        elif mission == "FINISH":
            # upload lidar and camera to s3
