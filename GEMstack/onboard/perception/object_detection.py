
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
# YOLO
from ultralytics import YOLO
from ultralytics.engine.results import Results, Boxes
# GEMStack
from ...state import AllState,VehicleState,ObjectPose,ObjectFrameEnum,AgentState,AgentEnum,AgentActivityEnum,ObjectFrameEnum
from ..interface.gem import GEMInterface, GNSSReading
from ..component import Component
from scipy.spatial.transform import Rotation as R



class ObjectDetection(Component):
    # TODO: Pull params into a JSON/yaml
    # TODO: Convert some lists into np.arrays, vectorize calculations
    # TODO: Implement logging instead of print, cleanup comments
    # TODO: Cleanup funcs + split into separate classes
    # TODO: Decide if we want to name dets "peds" or "objs"/"agents"
    #       Maybe peds for now and Agents in agent_detection.py?
    def __init__(self, vehicle_interface : GEMInterface) -> None:


        # vehicle interface
        self.vehicle_interface = vehicle_interface


        # cv2 to ros converter
        self.bridge = CvBridge()


        # yolo model
        self.detector = YOLO(os.getcwd()+'/GEMstack/knowledge/detection/yolov8n.pt')
        self.confidence = 0.1
        self.classes_to_detect = 0

   
    def initialize(self):
        super().initialize()
        self.vehicle_interface.subscribe_sensor('front_camera',self.front_camera_callback, type = cv2.Mat)
        # self.pub_pedestrians_pc2 = rospy.Publisher("/point_cloud/pedestrians", PointCloud2, queue_size=10)
        # self.pub_obj_centers_pc2 = rospy.Publisher("/point_cloud/obj_centers", PointCloud2, queue_size=10)
        # self.pub_bboxes_markers = rospy.Publisher("/markers/bboxes", MarkerArray, queue_size=10)
        self.pub_image = rospy.Publisher("/gem/debug", Image, queue_size=1)
        self.pub_detimage = rospy.Publisher("/gem/image_detection", Image, queue_size=1)


    def update(self, vehicle : VehicleState) -> Dict[str,AgentState]:

        print(f"VEHICLE State at time: {vehicle.pose.t}")

        print(f"x: {vehicle.pose.x}")
        print(f"y: {vehicle.pose.y}")

        print(f"z: {vehicle.pose.z}")
        print(f"roll: {vehicle.pose.roll}")
        print(f"pitch: {vehicle.pose.pitch}")
        print(f"yaw: {vehicle.pose.yaw}")
        print(f"speed: {vehicle.v}")


        return {}



    def front_camera_callback(self, image: cv2.Mat):
        

        ros_img = self.bridge.cv2_to_imgmsg(image, 'bgr8')
        self.pub_image.publish(ros_img)

        track_result = self.detector.track(source=image, persist=True, conf=self.confidence)

        class_names = self.detector.names
        label_text = "Object "
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.5
        font_color = (255, 255, 255)  # White text
        outline_color = (0, 0, 0)  # Black outline
        line_type = 2
        text_thickness = 1 # Text thickness
        outline_thickness = 1  # Thickness of the text outline

        boxes = track_result[0].boxes
        for box in boxes:

            
            class_id = int(box.cls.item())
            label_text = class_names[class_id]
            xywh = box.xywh[0].tolist()
            x, y, w, h = xywh
            id = box.id.item()

            # Draw bounding box
            cv2.rectangle(image, (int(x - w / 2), int(y - h / 2)), (int(x + w / 2), int(y + h / 2)), (255, 0, 255), 3)

            # Define text label
            x = int(x - w / 2)
            y = int(y - h / 2)
            label = label_text + str(id) + " : " + str(round(box.conf.item(), 2))

            # Get text size
            text_size, baseline = cv2.getTextSize(label, font, font_scale, line_type)
            text_w, text_h = text_size

            # Position text above the bounding box
            text_x = x
            text_y = y - 10 if y - 10 > 10 else y + h + text_h

            # Draw text outline for better visibility
            for dx, dy in [(-1, -1), (-1, 1), (1, -1), (1, 1)]:  
                cv2.putText(image, label, (text_x + dx, text_y - baseline + dy), font, font_scale, outline_color, outline_thickness)

            # Draw main text on top of the outline
            cv2.putText(image, label, (text_x, text_y - baseline), font, font_scale, font_color, text_thickness)


            
        ros_img = self.bridge.cv2_to_imgmsg(image, 'bgr8')
        self.pub_detimage.publish(ros_img)
        

        



    def rate(self):
        return 4.0
    
    def state_inputs(self):
        return ['vehicle']
    
    def state_outputs(self):
        return ['agents']

