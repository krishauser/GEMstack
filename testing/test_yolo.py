""" 
    Usage:
        python test_yolo.py -t <target_task>
"""

import time
import math
import numpy as np
import cv2
import rospy
import os
import pathlib
from ultralytics import YOLO
import cv2
import rospy
import numpy as np
from typing import Dict,Tuple, List
import time
from numpy.linalg import inv
# from sklearn.cluster import DBSCAN
import sys
import os
import cv2
import random
sys.path.append(os.getcwd())
abs_path = os.path.abspath(os.path.dirname(__file__))
from sensor_msgs.msg import Image,PointCloud2
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import struct
import ctypes
import sensor_msgs.point_cloud2 as pc2
import argparse

from cv_bridge import CvBridge
import numpy as np
import os

lidar_points = None
camera_image = None
depth_image = None
bridge = CvBridge()

parser = argparse.ArgumentParser()
parser.add_argument('--output_dir', '-o', type=str, default='output')
parser.add_argument('--test_target', '-t', type=str, required=True, 
                    choices=['yolo_ros', 'yolo_only'])
parser.add_argument('--vis', '-v', default=False, action='store_true')
args = parser.parse_args()

OUTPUT_DIR = args.output_dir

TOPICS = {    
    'e2': {    
        'lidar': "/lidar1/velodyne_points", # 10Hz, shape: (25281, 3)
        'camera': "/zed2/zed_node/rgb/image_rect_color" # 30Hz, shape: (720, 1280, 3)
    },
    'e4': {    
        'lidar': "/ouster/points", # 10Hz, shape: (131072, 3)
        'camera': "/oak/rgb/image_raw" # 10Hz, shape: (720, 1152, 3)
    },
}
topic = TOPICS
print ('topic subscribe:', topic)

def ros_PointCloud2_to_numpy(pc2_msg, want_rgb = False):
    if pc2 is None:
        raise ImportError("ROS is not installed")
    start = time.time()

    # rospy.loginfo('pc2_msg fields: %s', str(pc2_msg.fields))
    gen = pc2.read_points(pc2_msg, skip_nans=True, field_names=['x', 'y', 'z'])
    
    if want_rgb:
        xyzpack = np.array(list(gen),dtype=np.float32)
        if xyzpack.shape[1] != 4:
            raise ValueError("PointCloud2 does not have points with color data.")
        xyzrgb = np.empty((xyzpack.shape[0],6))
        xyzrgb[:,:3] = xyzpack[:,:3]
        for i,x in enumerate(xyzpack):
            rgb = x[3] 
            # cast float32 to int so that bitwise operations are possible
            s = struct.pack('>f' ,rgb)
            i = struct.unpack('>l',s)[0]
            # you can get back the float value by the inverse operations
            pack = ctypes.c_uint32(i).value
            r = (pack & 0x00FF0000)>> 16
            g = (pack & 0x0000FF00)>> 8
            b = (pack & 0x000000FF)
            #r,g,b values in the 0-255 range
            xyzrgb[i,3:] = (r,g,b)
        return xyzrgb
    else:
        start = time.time()
        res = np.array(list(gen),dtype=np.float32)[:,:3]
        rospy.loginfo('to array time: %s', str(time.time() - start))
        return res

class PedestrianDetector():
    """Detects and tracks pedestrians."""
    def __init__(self):
        yolo_path = os.path.join(abs_path, '../GEMstack/knowledge/detection/yolov8n.pt')


        self.detector = YOLO(yolo_path)

        self.pedestrian_counter = 0
        self.last_agent_states = {}
        self.previous_agents = {} 
        
        # init transformation parameters
        extrinsic = np.loadtxt("../GEMstack/knowledge/calibration/gem_e4_lidar2oak.txt")
                
        self.extrinsic = np.array(extrinsic)
        intrinsic = np.loadtxt("../GEMstack/knowledge/calibration/gem_e4_intrinsic.txt")
        self.intrinsic = np.concatenate([intrinsic, np.zeros((3, 1))], axis=1)

        lidar2vehicle_path = os.path.join(abs_path, '../GEMstack/knowledge/calibration/gem_e4_lidar2vehicle.txt')
        T_lidar2_Gem = np.loadtxt(lidar2vehicle_path)
        self.T_lidar2_Gem = np.asarray(T_lidar2_Gem)

        # obtained by GEMstack/offboard/calibration/check_target_lidar_range.py
        # Hardcode the roi area for agents
        self.xrange = (0, 100)
        self.yrange = (-200.0247698, 100.0374074)
        
        # subscribe
        self.bridge = CvBridge()
        self.e2_lidar_sub = rospy.Subscriber(topic['e2']['lidar'], PointCloud2, self.lidar_callback)
        self.e2_camera_sub = rospy.Subscriber(topic['e2']['camera'], Image, self.camera_callback)
        self.e4_lidar_sub = rospy.Subscriber(topic['e4']['lidar'], PointCloud2, self.lidar_callback)
        self.e4_camera_sub = rospy.Subscriber(topic['e4']['camera'], Image, self.camera_callback)
        
        self.zed_image = None
        self.point_cloud = None
        self.index = 0
        self.start_camera_t = time.time()
        self.start_lidar_t = time.time()
    
    def camera_callback(self, image: Image):
        end_camera_t = time.time()
        print ('camera arrive window:', end_camera_t - self.start_camera_t)
        self.start_camera_t = end_camera_t
        
        try:
            # Convert a ROS image message into an OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(image, "bgr8")
        except CvBridgeError as e:
            print(e)

        self.zed_image = cv_image

    def lidar_callback(self, point_cloud):
        end_lidar_t = time.time()
        rospy.loginfo('lidar arrive window: %s', str(end_lidar_t - self.start_lidar_t))
        self.start_lidar_t = end_lidar_t
    
        self.point_cloud = ros_PointCloud2_to_numpy(point_cloud, want_rgb=False)
        # self.point_cloud = alternative_ros_PointCloud2_to_numpy(point_cloud)
        
    def detect_agents(self):
        if self.zed_image is None:
            return
        # if self.zed_image is None or self.point_cloud is None:
        #     return 
      

        vis = self.zed_image.copy()
        # vis_pc = self.point_cloud.copy()
        
        t1 = time.time()
        detection_result = self.detector(vis,verbose=False)
        print ('detection time:', time.time() - t1)
        
        #TODO: create boxes from detection result
        boxes = []
        target_ids = [0, 2, 11]
        bbox_ids = []
        for box in detection_result[0].boxes: # only one image, so use index 0 of result
            class_id = int(box.cls[0].item())
            if class_id in target_ids: # class 0 stands for pedestrian
                bbox_ids.append(class_id)
                bbox = box.xywh[0].tolist()
                boxes.append(bbox)
            
        for i in range(len(boxes)):
            box = boxes[i]
            id = bbox_ids[i]
            # print ("id:", id)
            
            x,y,w,h = box
            xmin, xmax = x - w/2, x + w/2
            ymin, ymax = y - h/2, y + h/2
            
            # draw bbox
            color =  (255, 0, 255)
            left_up = (int(x-w/2), int(y-h/2))
            right_bottom = (int(x+w/2), int(y+h/2))
            cv2.rectangle(vis, left_up, right_bottom, color, thickness=2)
            
        if args.vis:
            cv2.imshow('frame', vis)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                exit(1)
        
def main():
    args = parser.parse_args()
    print ('======= Initial arguments =======')
    params = []
    for key, val in vars(args).items():
        param = f"--{key} {val}"
        print(f"{key} => {val}")
        params.append(param)

    if args.test_target == 'yolo_ros':
        rospy.init_node('rgb_track_node', anonymous=True)
        rate = rospy.Rate(30)  # Hz

        ped = PedestrianDetector()

        try:
            print ('\nStart detection...')
            while not rospy.is_shutdown():
                rate.sleep()  # Wait a while before trying to get a new waypoints
                ped.detect_agents()
        except rospy.ROSInterruptException:
            pass
    
    elif args.test_target == 'yolo_only':
        img = cv2.imread('test.png')
        yolo_path = os.path.join(abs_path, '../GEMstack/knowledge/detection/yolov8n.pt')
        detector = YOLO(yolo_path)

        for i in range(10): 
            t1 = time.time()
            detection_result = detector(img,verbose=False)
            print (f'{i} detection time:', time.time() - t1)


if __name__ == '__main__':
    main()