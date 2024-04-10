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
from sklearn.cluster import DBSCAN
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

colors = []
colors += [(i, 0, 255) for i in range(100, 256, 25)]
colors += [(i, 255, 0) for i in range(100, 256, 25)]
colors += [(0, i, 255) for i in range(100, 256, 25)]
colors += [(255, i, 0) for i in range(100, 256, 25)]
colors += [(255, 0, i) for i in range(100, 256, 25)]
colors += [(0, 255, i) for i in range(100, 256, 25)]
seed = 19
random.seed(seed)
random.shuffle(colors)

id2str = {
    0: "pedestrian",
    2: "car",
    11: "stop sign"
}

parser = argparse.ArgumentParser()
parser.add_argument('--output_dir', '-o', type=str, default='save')
args = parser.parse_args()

OUTPUT_DIR = args.output_dir


def ros_PointCloud2_to_numpy(pc2_msg, want_rgb = False):
    if pc2 is None:
        raise ImportError("ROS is not installed")
    gen = pc2.read_points(pc2_msg, skip_nans=True)
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
        return np.array(list(gen),dtype=np.float32)[:,:3]
    
def lidar_to_image(point_cloud_lidar: np.ndarray, extrinsic : np.ndarray, intrinsic : np.ndarray):
    
    homo_point_cloud_lidar = np.hstack((point_cloud_lidar, np.ones((point_cloud_lidar.shape[0], 1)))) # (N, 4)
    pointcloud_pixel = (intrinsic @ extrinsic @ (homo_point_cloud_lidar).T) # (3, N)
    pointcloud_pixel = pointcloud_pixel.T # (N, 3)

    # normalize
    pointcloud_pixel[:, 0] /= pointcloud_pixel[:, 2] # normalize
    pointcloud_pixel[:, 1] /= pointcloud_pixel[:, 2] # normalize
    point_cloud_image =  pointcloud_pixel[:,:2] # (N, 2)
    return point_cloud_image

def lidar_to_vehicle(point_cloud_lidar: np.ndarray, T_lidar2_Gem: np.ndarray):
    ones = np.ones((point_cloud_lidar.shape[0], 1))
    pcd_homogeneous = np.hstack((point_cloud_lidar, ones)) # (N, 4)
    pointcloud_trans = np.dot(T_lidar2_Gem, pcd_homogeneous.T) # (4, N)
    pointcloud_trans = pointcloud_trans.T # (N, 4)
    point_cloud_image_world = pointcloud_trans[:, :3] # (N, 3)
    return point_cloud_image_world

def filter_lidar_by_range(point_cloud, xrange: Tuple[float, float], yrange: Tuple[float, float]):
    xmin, xmax = xrange
    ymin, ymax = yrange
    idxs = np.where((point_cloud[:, 0] > xmin) & (point_cloud[:, 0] < xmax) &
                    (point_cloud[:, 1] > ymin) & (point_cloud[:, 1] < ymax) )
    return point_cloud[idxs]

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
        self.camera_info_sub = rospy.Subscriber("/ouster/points",  PointCloud2, self.lidar_callback)
        self.front_camera_sub = rospy.Subscriber("/oak/rgb/image_raw", Image, self.camera_callback)
        self.zed_image = None
        self.point_cloud = None
        self.index = 0
    
    def camera_callback(self, image: Image):
        try:
            # Convert a ROS image message into an OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(image, "bgr8")
        except CvBridgeError as e:
            print(e)

        self.zed_image = cv_image

    def lidar_callback(self, point_cloud):
        # self.point_cloud = point_cloud
        self.point_cloud = ros_PointCloud2_to_numpy(point_cloud, want_rgb=False)
    
    def detect_agents(self):
        if self.zed_image is None:
            return
        # if self.zed_image is None or self.point_cloud is None:
        #     return 
        
        vis = self.zed_image.copy()
        # vis_pc = self.point_cloud.copy()
        
        t1 = time.time()
        detection_result = self.detector(self.zed_image,verbose=False)
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
            print ("id:", id)
            
            x,y,w,h = box
            xmin, xmax = x - w/2, x + w/2
            ymin, ymax = y - h/2, y + h/2
            
            # draw bbox
            color =  (255, 0, 255)
            left_up = (int(x-w/2), int(y-h/2))
            right_bottom = (int(x+w/2), int(y+h/2))
            cv2.rectangle(vis, left_up, right_bottom, color, thickness=2)
            
        cv2.imshow('frame', vis)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            exit(1)
        
        
        # pathlib.Path(OUTPUT_DIR).mkdir(parents=True, exist_ok=True)
        
        # color_fn = os.path.join(OUTPUT_DIR,'color{}.png'.format(self.index))
        # lidar_fn = os.path.join(OUTPUT_DIR,'lidar{}.npz'.format(self.index))
        
        # print ('Output result:', color_fn)
        # cv2.imwrite(color_fn, vis)
        # np.savez(lidar_fn, vis_pc)
    
def main():
    # args = parser.parse_args()
    # print ('======= Initial arguments =======')
    # params = []
    # for key, val in vars(args).items():
    #     param = f"--{key} {val}"
    #     print(f"{key} => {val}")
    #     params.append(param)

    rospy.init_node('rgb_track_node', anonymous=True)
    rate = rospy.Rate(30)  # Hz

    ped = PedestrianDetector()

    try:
        print ('\nStart navigation...')
        while not rospy.is_shutdown():
            rate.sleep()  # Wait a while before trying to get a new waypoints
            ped.detect_agents()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()