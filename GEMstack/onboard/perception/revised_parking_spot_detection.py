from ...state import AllState, VehicleState, ObjectPose, ObjectFrameEnum, AgentState, AgentEnum, AgentActivityEnum
from sensor_msgs.msg import Image
from ..interface.gem import GEMInterface
from ..component import Component
from typing import Dict
import threading
import copy
import cv2
import numpy as np
import rospy
from ultralytics import YOLO  # Assuming appropriate relative path or installed package
from pixelwise_3D_lidar_coord_handler import PixelWise3DLidarCoordHandler
import sensor_msgs.point_cloud2 as pc2

sobel_kernel_size = 3
sobel_min_threshold = 90
conf_thresh = 0.835
MODEL_WEIGHT_PATH = '../../knowledge/detection/parking_spot_detection.pt'

# Template for a parking spot detector
class ParkingSpotDetector(Component):
    """Obtains agent detections from a simulator"""
    def __init__(self, vehicle_interface: GEMInterface, model_weight_path: str):
        self.pub = rospy.Publisher("/annotated_lines", Image, queue_size=10)
        self.vehicle_interface = vehicle_interface
        self.model = YOLO(model_weight_path)
        self.handler = PixelWise3DLidarCoordHandler()
        self.front_image = None
        self.parking_spot = None

    def rate(self):
        return 0.25

    def state_outputs(self):
        return ['parking_spot']

    def initialize(self):
        # Subscribe to necessary sensors
        self.vehicle_interface.subscribe_sensor('front_camera', self.image_callback, cv2.Mat)
        self.vehicle_interface.subscribe_sensor('lidar', self.lidar_callback, 'PointCloud2')

    def image_callback(self, image: cv2.Mat):
        self.front_image = image

    def lidar_callback(self, point_cloud):
        self.point_cloud = point_cloud

    def parking_spot_detection(self):
        if self.front_image is None or self.point_cloud is None:
            return None  # Wait until both image and point cloud are available
        
        bbox_info = self.detect_empty(self.front_image)
        if bbox_info:
            canvas = self.front_image.copy()
            goal_pose = self.get_parking_spot(canvas, bbox_info)

        x,y,yaw = goal_pose

        return ObjectPose(t=0, x=x, y=y, yaw=yaw, frame=ObjectFrameEnum.CURRENT)

    def empty_detect(self, img: cv2.Mat, empty_spot=0, conf_threshold=conf_thresh):
        global model
        results = model(img)
        for box, conf in zip(results[0].obb, results[0].obb.conf):
            class_id = int(box.cls[0].item())
            confidence = float(conf.item())
            if class_id == empty_spot and confidence >= conf_threshold:
                x, y, w, h, r = box.xywhr[0].tolist()
                return (x, y, w, h, r)  
        return None

    def get_parking_spot(self, canvas, bbox_info):
        # Additional image processing and visualization logic here
        pass

    def update(self):
        self.parking_spot = self.parking_spot_detection()
        return copy.deepcopy(self.parking_spot)
        # Make sure that the goal pose frame doesnt have to be altered