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
from message_filters import Subscriber, TimeSynchronizer
import message_filters

parser = argparse.ArgumentParser()
parser.add_argument('--output_dir', '-o', type=str, default='save')
parser.add_argument('--vis', '-v', default=False, action='store_true')
args = parser.parse_args()

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
# topic = TOPICS[args.vehicle]
topic = TOPICS
print ('topic subscribe:', topic)
        
def ros_PointCloud2_to_numpy(pc2_msg, want_rgb = False):
    if pc2 is None:
        raise ImportError("ROS is not installed")
    # gen = pc2.read_points(pc2_msg, skip_nans=True)
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
        self.xrange = (0, 20)
        self.yrange = (-10.0247698, 10.0374074)
        
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
        rospy.loginfo('camera_callback:', end_camera_t - self.start_camera_t)
        self.start_camera_t = end_camera_t
        # try:
        #     # Convert a ROS image message into an OpenCV image
        #     cv_image = self.bridge.imgmsg_to_cv2(image, "bgr8")
        # except CvBridgeError as e:
        #     print(rospy.loginfo

        self.zed_image = image

    def lidar_callback(self, point_cloud):
        end_lidar_t = time.time()
        rospy.loginfo('lidar_callback:', end_lidar_t - self.start_lidar_t)
        self.start_lidar_t = end_lidar_t
        
        self.point_cloud = point_cloud

        # self.point_cloud = ros_PointCloud2_to_numpy(point_cloud, want_rgb=False)
    
    def vis_lidar_by_clusters(self, vis, point_cloud_image, clusters):
        for proj_pt, cluster in zip(point_cloud_image, clusters):
            color = colors[cluster % len(colors)]
            radius = 1
            center = int(proj_pt[0]), int(proj_pt[1])
            vis = cv2.circle(vis, center, radius, color, cv2.FILLED)
        return vis    
    
    def detect_agents(self, img):
        if self.zed_image is None:
            return
        # if self.zed_image is None or self.point_cloud is None:
        #     return 
        
        vis = img.copy()
        # vis_pc = self.point_cloud.copy()
        
        t1 = time.time()
        detection_result = self.detector(img,verbose=False)
        rospy.loginfo('detection time: %s', str(time.time() - t1))
        
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
        
        return vis
    
    def test_lidar_to_image_dbscan(self):
        if self.zed_image is None or self.point_cloud is None:
            return
        
        vis = self.bridge.imgmsg_to_cv2(self.zed_image, "bgr8")
        
        start_t = time.time()
        pc = ros_PointCloud2_to_numpy(self.point_cloud, want_rgb=False)
        rospy.loginfo('ros_PointCloud2_to_numpy time: %s', str(time.time() - start_t))
        
        start_t = time.time()
        filtered_point_cloud = filter_lidar_by_range(pc, self.xrange,self.yrange)
        rospy.loginfo('filtered_point_cloud time: %s', str(time.time() - start_t))
        
        
        start_t = time.time()
        # Perform DBSCAN clustering
        epsilon = 0.1  # Epsilon parameter for DBSCAN
        min_samples = 5  # Minimum number of samples in a cluster
        dbscan = DBSCAN(eps=epsilon, min_samples=min_samples)
        clusters = dbscan.fit_predict(filtered_point_cloud)
        rospy.loginfo('dbscan time: %s', str(time.time() - start_t))

        start_t = time.time()
        point_cloud_image = lidar_to_image(filtered_point_cloud, 
                                        self.extrinsic, 
                                        self.intrinsic)
        rospy.loginfo('lidar_to_image time: %s', str(time.time() - start_t))
        
        start_t = time.time()
        vis = self.detect_agents(vis)
        rospy.loginfo('detect_agents time: %s', str(time.time() - start_t))
        
        start_t = time.time()
        vis = self.vis_lidar_by_clusters(vis, point_cloud_image, clusters)
        rospy.loginfo('vis_lidar time: %s', str(time.time() - start_t))
            
        cv2.imshow('frame', vis)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            exit(1)


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
        print ('\nStart detection...')
        while not rospy.is_shutdown():
            rate.sleep()  # Wait a while before trying to get a new waypoints
            # ped.detect_agents()
            ped.test_lidar_to_image_dbscan()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()