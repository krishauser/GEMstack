"""
Demo script to run YOLO object detection on a Gazebo simulation.

This code subscribes to the front camera feed from the GEM e2/e4 model and applies YOLO-based object detection to the incoming images.

Visualization:
- Use RViz or rqt to monitor the topics.

ROS Topics:
- Raw camera feed: /gem/debug
- YOLO detection output: /gem/image_detection
"""


# Python 
import os
from typing import List, Dict
from collections import defaultdict
from datetime import datetime
import copy
# ROS, CV
import rospy
import message_filters
import cv2
import numpy as np
import tf
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, PointCloud2
from visualization_msgs.msg import MarkerArray
# GEMStack
from ...state import AllState,VehicleState,ObjectPose,ObjectFrameEnum,AgentState,AgentEnum,AgentActivityEnum,ObjectFrameEnum
from ..interface.gem import GEMInterface, GNSSReading
from ..component import Component
from scipy.spatial.transform import Rotation as R



class SensorCheck(Component):

    def __init__(self, vehicle_interface : GEMInterface) -> None:
        """
        Initializes essential functions required to run the YOLO model.
        """

        # vehicle interface
        self.vehicle_interface = vehicle_interface
        # cv2 to ros converter
        self.bridge = CvBridge()


   
    def initialize(self):
        """
        Defines callback functions for subscribing to the front camera image stream and sets up publishers for debugging purposes.
        """

        super().initialize()
        self.vehicle_interface.subscribe_sensor('front_camera',self.front_camera_callback, type = cv2.Mat)
        self.vehicle_interface.subscribe_sensor('front_left_camera',self.front_left_camera_callback, type = cv2.Mat)
        self.vehicle_interface.subscribe_sensor('front_right_camera',self.front_right_camera_callback, type = cv2.Mat)
        self.vehicle_interface.subscribe_sensor('rear_left_camera',self.rear_left_camera_callback, type = cv2.Mat)
        self.vehicle_interface.subscribe_sensor('rear_right_camera',self.rear_right_camera_callback, type = cv2.Mat)

        self.pub_front_camera_image = rospy.Publisher("/gem/debug/front_camera", Image, queue_size=1)
        self.pub_front_left_camera_image = rospy.Publisher("/gem/debug/front_left_camera", Image, queue_size=1)
        self.pub_front_right_camera_image = rospy.Publisher("/gem/debug/front_right_camera", Image, queue_size=1)
        self.pub_rear_left_camera_image = rospy.Publisher("/gem/debug/rear_left_camera", Image, queue_size=1)
        self.pub_rear_right_camera_image = rospy.Publisher("/gem/debug/rear_right_camera", Image, queue_size=1)
        


    def update(self, vehicle : VehicleState) -> Dict[str,AgentState]:

        """
        Displays vehicle statistics in the GLOBAL reference frame.

        This function also allows switching between coordinate frames using the `VehicleState -> pose` method.

        Returning an `AgentState` will automatically log detected objects.
        """

        return {}



    def front_camera_callback(self, image: cv2.Mat):
        
        """
        A simple callback function that re published the topic to validate image coming through gazebo
        """

        ros_img = self.bridge.cv2_to_imgmsg(image, 'bgr8')
        self.pub_front_camera_image.publish(ros_img)

    def front_left_camera_callback(self, image: cv2.Mat):
        
        """
        A simple callback function that re published the topic to validate image coming through gazebo
        """

        ros_img = self.bridge.cv2_to_imgmsg(image, 'bgr8')
        self.pub_front_left_camera_image.publish(ros_img)

    def front_right_camera_callback(self, image: cv2.Mat):
        
        """
        A simple callback function that re published the topic to validate image coming through gazebo
        """

        ros_img = self.bridge.cv2_to_imgmsg(image, 'bgr8')
        self.pub_front_right_camera_image.publish(ros_img)

    def rear_left_camera_callback(self, image: cv2.Mat):
        
        """
        A simple callback function that re published the topic to validate image coming through gazebo
        """

        ros_img = self.bridge.cv2_to_imgmsg(image, 'bgr8')
        self.pub_rear_left_camera_image.publish(ros_img)

    def rear_right_camera_callback(self, image: cv2.Mat):
        
        """
        A simple callback function that re published the topic to validate image coming through gazebo
        """

        ros_img = self.bridge.cv2_to_imgmsg(image, 'bgr8')
        self.pub_rear_right_camera_image.publish(ros_img)


    def rate(self):
        return 4.0
    
    def state_inputs(self):
        return ['vehicle']
    
    def state_outputs(self):
        return ['agents']

