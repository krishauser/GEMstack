import os
import rospy
import cv2
import numpy as np
import struct
import ctypes
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge, CvBridgeError
from ultralytics import YOLO
from pixelwise_3D_lidar_coord_handler import PixelWise3DLidarCoordHandler
import sensor_msgs.point_cloud2 as pc2

conf_val = 0.835
MODEL_WEIGHT_PATH = '../../knowledge/detection/parking_spot_detection.pt'

class ImageProcessorNode:
    def __init__(self):
        self.bridge = CvBridge()
        self.model = YOLO(MODEL_WEIGHT_PATH)
        self.handler = PixelWise3DLidarCoordHandler()
        self.image_sub = rospy.Subscriber("/oak/rgb/image_raw", Image, self.image_callback)
        self.lidar_sub = rospy.Subscriber("/ouster/points", PointCloud2, self.lidar_callback)
        self.point_cloud = None

    def lidar_callback(self, point_cloud):
        self.point_cloud = point_cloud

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print("CvBridge Error:", e)
            return
        if self.point_cloud is None:
            return
        self.process_image(cv_image)

    def process_image(self, cv_image):
        bbox_info = self.detect_empty(cv_image)
        if bbox_info:
            x, y, yaw = self.calculate_pose(cv_image, bbox_info)
            print(f"3D coordinates and yaw: [{x}, {y}, {yaw}]")

    def detect_empty(self, img):
        results = self.model(img)
        for box, conf in zip(results[0].obb, results[0].obb.conf):
            class_id, confidence = int(box.cls[0].item()), float(conf.item())
            if class_id == 0 and confidence >= conf_val:
                x, y, w, h, r = box.xywhr[0].tolist()
                return (x, y, w, h, r)
        return None

    def calculate_pose(self, img, bbox_info):
        x, y, w, h, r = bbox_info
        angle = np.pi/2 - r
        mid_x, mid_y = x + w / 2, y + h / 2
        if self.point_cloud:
            numpy_point_cloud = self.handler.ros_PointCloud2_to_numpy(self.point_cloud)
            coord_3d_map = self.handler.get3DCoord(img, numpy_point_cloud)
            if 0 <= mid_x < coord_3d_map.shape[1] and 0 <= mid_y < coord_3d_map.shape[0]:
                coord_3d = coord_3d_map[int(mid_y), int(mid_x)]
                return coord_3d[0], coord_3d[1], angle

if __name__ == '__main__':
    rospy.init_node('image_processor', anonymous=True)
    processor_node = ImageProcessorNode()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")