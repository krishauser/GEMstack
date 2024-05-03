import time
import rospy
import pathlib
from ultralytics import YOLO
import cv2
import rospy
import numpy as np
from typing import Dict, Tuple, List
import time
from numpy.linalg import inv
from sklearn.cluster import DBSCAN
import sys
import os
import random
sys.path.append(os.getcwd())
abs_path = os.path.abspath(os.path.dirname(__file__))
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge, CvBridgeError
import struct
import ctypes
import sensor_msgs.point_cloud2 as pc2
import argparse
import message_filters
from ultralytics.utils.plotting import Annotator, colors
from collections import defaultdict
from GEMstack.onboard.perception.agent_detection import MultiObjectDetector, MultiObjectTracker
from GEMstack.onboard.perception.pixelwise_3D_lidar_coord_handler import PixelWise3DLidarCoordHandler
from GEMstack.onboard.interface.gem import GEMInterface
from GEMstack.state.vehicle import VehicleState
from GEMstack.state.physical_object import ObjectFrameEnum, ObjectPose
from GEMstack.state.agent import AgentActivityEnum, AgentEnum, AgentState
from GEMstack.state.sign import SignEnum, SignState

parser = argparse.ArgumentParser()
parser.add_argument('--test_target', '-t', type=str, default='detection',
                    choices=['detection', 'track'])
parser.add_argument('--output_dir', '-o', type=str, default='save')
parser.add_argument('--vis', '-v', default=False, action='store_true')
parser.add_argument('--use_message_filter', '-m', default=False, action='store_true')
parser.add_argument('--do_cluster', '-c', default=False, action='store_true')
args = parser.parse_args()

pc_colors = []
pc_colors += [(i, 0, 255) for i in range(100, 256, 25)]
pc_colors += [(i, 255, 0) for i in range(100, 256, 25)]
pc_colors += [(0, i, 255) for i in range(100, 256, 25)]
pc_colors += [(255, i, 0) for i in range(100, 256, 25)]
pc_colors += [(255, 0, i) for i in range(100, 256, 25)]
pc_colors += [(0, 255, i) for i in range(100, 256, 25)]
seed = 19
random.seed(seed)
random.shuffle(pc_colors)

id2str = {
    0: "pedestrian",
    2: "car",
    11: "stop sign"
}

OUTPUT_DIR = args.output_dir
TOPICS = {
    'e2': {
        'lidar': "/lidar1/velodyne_points",  # 10Hz, shape: (25281, 3)
        'camera': "/zed2/zed_node/rgb/image_rect_color"  # 30Hz, shape: (720, 1280, 3)
    },
    'e4': {
        'lidar': "/ouster/points",  # 10Hz, shape: (131072, 3)
        'camera': "/oak/rgb/image_raw"  # 10Hz, shape: (720, 1152, 3)
    },
}
# topic = TOPICS[args.vehicle]
topic = TOPICS

def ros_PointCloud2_to_numpy(pc2_msg, want_rgb=False):
    if pc2 is None:
        raise ImportError("ROS is not installed")
    # gen = pc2.read_points(pc2_msg, skip_nans=True)
    gen = pc2.read_points(pc2_msg, skip_nans=True, field_names=['x', 'y', 'z'])

    if want_rgb:
        xyzpack = np.array(list(gen), dtype=np.float32)
        if xyzpack.shape[1] != 4:
            raise ValueError("PointCloud2 does not have points with color data.")
        xyzrgb = np.empty((xyzpack.shape[0], 6))
        xyzrgb[:,:3] = xyzpack[:,:3]
        for i, x in enumerate(xyzpack):
            rgb = x[3]
            # cast float32 to int so that bitwise operations are possible
            s = struct.pack('>f', rgb)
            i = struct.unpack('>l', s)[0]
            # you can get back the float value by the inverse operations
            pack = ctypes.c_uint32(i).value
            r = (pack & 0x00FF0000) >> 16
            g = (pack & 0x0000FF00) >> 8
            b = (pack & 0x000000FF)
            # r,g,b values in the 0-255 range
            xyzrgb[i,3:] = (r, g, b)
        return xyzrgb
    else:
        return np.array(list(gen), dtype=np.float32)[:,:3]

def lidar_to_image(point_cloud_lidar: np.ndarray, extrinsic: np.ndarray, intrinsic: np.ndarray):
    homo_point_cloud_lidar = np.hstack((point_cloud_lidar, np.ones((point_cloud_lidar.shape[0], 1))))  # (N, 4)
    pointcloud_pixel = (intrinsic @ extrinsic @ (homo_point_cloud_lidar).T)  # (3, N)
    pointcloud_pixel = pointcloud_pixel.T  # (N, 3)

    # normalize
    pointcloud_pixel[:, 0] /= pointcloud_pixel[:, 2]  # normalize
    pointcloud_pixel[:, 1] /= pointcloud_pixel[:, 2]  # normalize
    point_cloud_image =  pointcloud_pixel[:,:2]  # (N, 2)
    return point_cloud_image

def lidar_to_vehicle(point_cloud_lidar: np.ndarray, T_lidar2_Gem: np.ndarray):
    ones = np.ones((point_cloud_lidar.shape[0], 1))
    pcd_homogeneous = np.hstack((point_cloud_lidar, ones))  # (N, 4)
    pointcloud_trans = np.dot(T_lidar2_Gem, pcd_homogeneous.T)  # (4, N)
    pointcloud_trans = pointcloud_trans.T  # (N, 4)
    point_cloud_image_world = pointcloud_trans[:, :3]  # (N, 3)
    return point_cloud_image_world

def filter_lidar_by_range(point_cloud, xrange: Tuple[float, float], yrange: Tuple[float, float], zrange: Tuple[float, float]):
    xmin, xmax = xrange
    ymin, ymax = yrange
    zmin, zmax = zrange
    idxs = np.where((point_cloud[:, 0] > xmin) & (point_cloud[:, 0] < xmax) &
                    (point_cloud[:, 1] > ymin) & (point_cloud[:, 1] < ymax) &
                     (point_cloud[:, 2] > zmin) & (point_cloud[:, 2] < zmax))
    return point_cloud[idxs]

class PedestrianDetector():
    """Detects and tracks pedestrians."""
    def __init__(self):
        yolo_path = os.path.join(abs_path, '../GEMstack/knowledge/detection/yolov8s.pt')
        self.detector = YOLO(yolo_path)


        self.pedestrian_counter = 0
        self.last_agent_states = {}
        self.previous_agents = {}

        # init transformation parameters
        extrinsic = np.loadtxt("../GEMstack/GEMstack/knowledge/calibration/gem_e4_lidar2oak.txt")

        self.extrinsic = np.array(extrinsic)
        intrinsic = np.loadtxt("../GEMstack/GEMstack/knowledge/calibration/gem_e4_intrinsic.txt")
        self.intrinsic = np.concatenate([intrinsic, np.zeros((3, 1))], axis=1)

        lidar2vehicle_path = os.path.join(abs_path, '../GEMstack/knowledge/calibration/gem_e4_lidar2vehicle.txt')
        T_lidar2_Gem = np.loadtxt(lidar2vehicle_path)
        self.T_lidar2_Gem = np.asarray(T_lidar2_Gem)

        gem_interface = GEMInterface()
        self.MOD = MultiObjectDetector(gem_interface, extrinsic)
        self.MOT = MultiObjectTracker(
            test=True,
            write_all=0,
            detection_file_name='GEMstack/onboard/prediction/written_frames.txt'
        )

        # obtained by GEMstack/offboard/calibration/check_target_lidar_range.py
        # Hardcode the roi area for agents
        self.xrange = (0, 20)
        self.yrange = (-10, 10)
        self.zrange = (-3, 1)
        # subscribe

        self.bridge = CvBridge()

        if args.use_message_filter:
            image_sub = message_filters.Subscriber(topic['e4']['lidar'], Image)
            lidar_sub = message_filters.Subscriber(topic['e4']['camera'], PointCloud2)
            ts = message_filters.ApproximateTimeSynchronizer([image_sub, lidar_sub], 5, 1)
            ts.registerCallback(self.sync_callback)
        else:
            self.e4_lidar_sub = rospy.Subscriber(topic['e4']['lidar'], PointCloud2, self.lidar_callback)
            self.e4_camera_sub = rospy.Subscriber(topic['e4']['camera'], Image, self.camera_callback)

        self.zed_image = None
        self.point_cloud = None
        self.track_history = defaultdict(lambda: [])
        self.coord_3d_handler = PixelWise3DLidarCoordHandler()

    def sync_callback(self, image: Image, point_cloud: PointCloud2):
        self.zed_image = image
        self.point_cloud = point_cloud
        self.test_kalman_tracking()

    def camera_callback(self, image: Image):
        self.zed_image = image

    def lidar_callback(self, point_cloud):
        self.point_cloud = point_cloud
        # self.point_cloud = ros_PointCloud2_to_numpy(point_cloud, want_rgb=False)

    def vis_lidar_by_clusters(self, vis, point_cloud_image, clusters):
        for proj_pt, cluster in zip(point_cloud_image, clusters):
            color = pc_colors[cluster % len(pc_colors)]
            radius = 1
            center = int(proj_pt[0]), int(proj_pt[1])
            cv2.circle(vis, center, radius, color, cv2.FILLED)
        return vis

    def detect_agents(self, img, pc_3D):
        if self.zed_image is None:
            return

        rospy.loginfo("Detecting agents...")

        vis = img.copy()

        t1 = time.time()
        detection_result = self.detector(img, verbose=False)

        rospy.loginfo('Detection time: %s', str(time.time() - t1))

        target_ids = [0, 2, 11]  # Target class IDs
        class_names = {0: "pedestrian", 2: "car", 11: "stop sign"}

        start_t = time.time()
        coord_3d_map = self.coord_3d_handler.get3DCoord(img, pc_3D)
        rospy.loginfo('PixelWise3DLidarCoordHandler time: %s', str(time.time() - start_t))

        for box in detection_result[0].boxes:
            class_id = int(box.cls[0].item())
            if class_id in target_ids:
                bbox = box.xywh[0].tolist()
                x, y, w, h = bbox
                xmin, xmax = x - w / 2, x + w / 2
                ymin, ymax = y - h / 2, y + h / 2
                
                text = class_names[class_id]
                if class_id == 0:
                    print (f'pedestrian x: {x}, y: {y}')
                    x_3d, y_3d, z_3d = coord_3d_map[int(y)][int(x)]
                    text = text + f' x={x_3d:.2f}, y={y_3d:.2f}, z={z_3d:.2f}'
                    
                # draw bbox
                color = (255, 0, 255)
                left_up = (int(x - w / 2), int(y - h / 2))
                right_bottom = (int(x + w / 2), int(y + h / 2))
                cv2.rectangle(vis, left_up, right_bottom, color, thickness=2)
                cv2.putText(vis, text, (left_up[0], left_up[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                            color, 2)

        rospy.loginfo("Detection complete.")
        return vis

    def vis_depth_estimation(self, vis, box, class_name, point_cloud_image, pc_3D, clusters):
        x,y,w,h = box
        xmin, xmax = x - w/2, x + w/2
        ymin, ymax = y - h/2, y + h/2
        
        # Filter. Get the idxs of point cloud that belongs to the agent
        idxs = np.where((point_cloud_image[:, 0] > xmin) & (point_cloud_image[:, 0] < xmax) &
                        (point_cloud_image[:, 1] > ymin) & (point_cloud_image[:, 1] < ymax) )
        agent_image_pc = point_cloud_image[idxs]
        agent_pc_3D = pc_3D[idxs]
        agent_clusters = clusters[idxs]
        
        # Get unique elements and their counts
        unique_elements, counts = np.unique(agent_clusters, return_counts=True)
        max_freq = np.max(counts)
        label_cluster = unique_elements[counts == max_freq]
        
        # filter again
        idxs = agent_clusters == label_cluster
        agent_image_pc = agent_image_pc[idxs]
        agent_clusters = agent_clusters[idxs]
        agent_pc_3D = agent_pc_3D[idxs]
        
        # calulate depth
        depth = np.mean( (agent_pc_3D[:, 0] ** 2 + agent_pc_3D[:, 1] ** 2) ** 0.5 ) # euclidean dist
        print ('depth:', depth)
        
        # draw point cloud
        for proj_pt, cluster in zip(agent_image_pc, agent_clusters):
            if cluster != label_cluster:
                continue
            pc_color = pc_colors[cluster % len(pc_colors)]
            radius = 1
            center = int(proj_pt[0]), int(proj_pt[1])
            cv2.circle(vis, center, radius, pc_color, cv2.FILLED)
        
        return depth
    
    def track_agents(self, frame, point_cloud_image, pc_3D, clusters=None):
        print("============================================================")
        rospy.loginfo("Tracking agents...")

        results = self.detector.track(frame, persist=True, verbose=False)
        boxes = results[0].boxes.xyxy.cpu()

        detected_ped = []
        detected_car = []
        detected_sign = []
        start_t = time.time()
        coord_3d_map = self.coord_3d_handler.get3DCoord(frame, pc_3D)
        rospy.loginfo('PixelWise3DLidarCoordHandler time: %s', str(time.time() - start_t))

        names = self.detector.model.names
        if results[0].boxes.id is not None:
            clss = results[0].boxes.cls.cpu().tolist()
            track_ids = results[0].boxes.id.int().cpu().tolist()

            annotator = Annotator(frame, line_width=2)

            # Initialize dictionaries to keep track of counts for each class
            class_counts = defaultdict(int)
            class_annotations = defaultdict(int)
            

            for box, cls, track_id in zip(boxes, clss, track_ids):
                if int(cls) in [0, 2, 11]:  # Check if class is pedestrian, car, or stop sign
                    class_name = names[int(cls)]
                    box = box.numpy()
                    label = f"{class_name} {class_counts[class_name]}"

                    # Increment count for the class
                    class_counts[class_name] += 1
                
                    x1,y1,x2,y2 = box

                    # Annotate with the class name and count
                    if clusters is not None:
                        depth = self.vis_depth_estimation(frame, box, class_name, point_cloud_image, pc_3D, clusters)
                        label = label + f" {depth:.2}"
                    else:
                        center_x = (x1 + x2) / 2
                        center_y = (y1 + y2) / 2
                        if int(cls) == 0:
                            print (f'pedestrian {class_counts[class_name]} x: {center_x}, y: {center_y}')
                        x_3d, y_3d, z_3d = coord_3d_map[int(center_y)][int(center_x)]
                        depth = x_3d
                        label = label + f' x={x_3d:.2f}, y={y_3d:.2f}, z={z_3d:.2f}'

                    annotator.box_label(box, color=colors(int(cls), True), label=label)

                    # Store the annotation count for the class
                    class_annotations[class_name] = class_counts[class_name]

                    track = self.track_history[track_id]
                    track.append((int((box[0] + box[2]) / 2), int((box[1] + box[3]) / 2)))

                    if cls == 0:
                        color = (255, 0, 255)
                        agent_type = AgentEnum.PEDESTRIAN
                        print(f"Pedestrian {class_counts[class_name]} Depth:", depth)
                        agent = self.MOD.box_to_agent(box, agent_type, depth)
                        if agent is not None:
                            detected_ped.append(agent) # Pedestrian tracking info => type:AgentState
                        vel = self.MOT.track_agents(detected_ped)
                        print(f"{class_name}{class_counts[class_name]} Velocity:", vel)  
                        print("============================================================") 

                    if cls == 2:
                        color = (0, 255, 255)
                        agent_type = AgentEnum.CAR
                        print("Vehicle Depth:", depth)
                        agent = self.MOD.box_to_agent(box, agent_type, depth)
                        if agent is not None:
                            detected_car.append(agent) # Vehicle tracking info => type:AgentState
                        vel = self.MOT.track_agents(detected_car)
                        print(f"{class_name}{class_counts[class_name]} Velocity:", vel)
                        print("============================================================") 

                    if cls == 11:
                        color = (150, 50, 255)
                        agent_type = SignEnum.STOP_SIGN
                        print("Stop Sign Depth:", depth)
                        agent = self.MOD.box_to_agent(box, agent_type, depth)
                        if agent is not None:
                            detected_sign.append(agent) # Stop sign tracking info => type:AgentState
                        vel = self.MOT.track_agents(detected_sign)
                        print(f"{class_name}{class_counts[class_name]} Velocity:", vel)
                        print("============================================================") 

                    if len(track) > 30:
                        track.pop(0)
                    points = np.array(track, dtype=np.int32).reshape((-1, 1, 2))
                    cv2.circle(frame, (track[-1]), 7, colors(int(cls), True), -1)
                    cv2.polylines(frame, [points], isClosed=False, color=colors(int(cls), True), thickness=2)

        rospy.loginfo("Tracking complete.")
        return frame

    def test_kalman_tracking(self):
        if self.zed_image is None or self.point_cloud is None:
            return

        vis = self.bridge.imgmsg_to_cv2(self.zed_image, "bgr8")

        start_t = time.time()
        raw_point_cloud = ros_PointCloud2_to_numpy(self.point_cloud, want_rgb=False)
        rospy.loginfo('ros_PointCloud2_to_numpy time: %s', str(time.time() - start_t))
        print('ROS PointCloud2 to numpy time:', str(time.time() - start_t))

        point_cloud_image = None # dummy
        clusters = None # dummy

        start_t = time.time()
        if args.test_target == 'detection':
            vis = self.detect_agents(vis, raw_point_cloud)
        elif args.test_target == 'track':
            vis = self.track_agents(vis, point_cloud_image, raw_point_cloud, clusters)
        rospy.loginfo('detect_agents time: %s', str(time.time() - start_t))
        print('Detection time:', str(time.time() - start_t))        
        
        cv2.imshow('frame', vis)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            exit(1)


def main():
    rospy.init_node('rgb_track_node', anonymous=True)
    rate = rospy.Rate(30)  # Hz

    ped = PedestrianDetector()

    try:
        print('\nStart detection...')
        while not rospy.is_shutdown():
            rate.sleep()  # Wait a while before trying to get a new waypoints
            ped.test_kalman_tracking()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()