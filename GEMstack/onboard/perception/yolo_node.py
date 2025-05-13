# from ...state import AllState, VehicleState, ObjectPose, ObjectFrameEnum, AgentState, AgentEnum, AgentActivityEnum
# from ..interface.gem import GEMInterface
# from ..component import Component
# from perception_utils import *
from ultralytics import YOLO
import cv2
from typing import Dict
import open3d as o3d
import numpy as np
from scipy.spatial.transform import Rotation as R
import rospy
from sensor_msgs.msg import PointCloud2, Image
from message_filters import Subscriber, ApproximateTimeSynchronizer
from cv_bridge import CvBridge
import time
import os
import yaml
import ros_numpy


from sklearn.cluster import DBSCAN
import sensor_msgs.point_cloud2 as pc2

# Publisher imports:
from jsk_recognition_msgs.msg import BoundingBox, BoundingBoxArray
from geometry_msgs.msg import Pose, Vector3


def extract_roi_box(lidar_pc, center, half_extents):
    """
    Extract a region of interest (ROI) from the LiDAR point cloud defined by an axis-aligned bounding box.
    """
    lower = center - half_extents
    upper = center + half_extents
    mask = np.all((lidar_pc >= lower) & (lidar_pc <= upper), axis=1)
    return lidar_pc[mask]

def pc2_to_numpy(pc2_msg, want_rgb=False):
    """
    Convert a ROS PointCloud2 message into a numpy array quickly using ros_numpy.
    This function extracts the x, y, z coordinates from the point cloud.
    """
    # Convert the ROS message to a numpy structured array
    pc = ros_numpy.point_cloud2.pointcloud2_to_array(pc2_msg)
    # Convert each field to a 1D array and stack along axis 1 to get (N, 3)
    pts = np.stack((np.array(pc['x']).ravel(),
                    np.array(pc['y']).ravel(),
                    np.array(pc['z']).ravel()), axis=1)
    # Apply filtering (for example, x > 0 and z < 2.5)
    mask = (pts[:, 0] > 0) & (pts[:, 2] < 2.5)
    return pts[mask]


def backproject_pixel(u, v, K):
    """
    Backprojects a pixel coordinate (u, v) into a normalized 3D ray in the camera coordinate system.
    """
    cx, cy = K[0, 2], K[1, 2]
    fx, fy = K[0, 0], K[1, 1]
    x = (u - cx) / fx
    y = (v - cy) / fy
    ray_dir = np.array([x, y, 1.0])
    return ray_dir / np.linalg.norm(ray_dir)


def find_human_center_on_ray(lidar_pc, ray_origin, ray_direction,
                             t_min, t_max, t_step,
                             distance_threshold, min_points, ransac_threshold):
    """
    Identify the center of a human along a projected ray.
    (This function is no longer used in the new approach.)
    """
    return None, None, None


def extract_roi(pc, center, roi_radius):
    """
    Extract points from a point cloud that lie within a specified radius of a center point.
    """
    distances = np.linalg.norm(pc - center, axis=1)
    return pc[distances < roi_radius]


def refine_cluster(roi_points, center, eps=0.2, min_samples=10):
    """
    Refine a point cluster by applying DBSCAN and return the cluster closest to 'center'.
    """
    if roi_points.shape[0] < min_samples:
        return roi_points
    clustering = DBSCAN(eps=eps, min_samples=min_samples).fit(roi_points)
    labels = clustering.labels_
    valid_clusters = [roi_points[labels == l] for l in set(labels) if l != -1]
    if not valid_clusters:
        return roi_points
    best_cluster = min(valid_clusters, key=lambda c: np.linalg.norm(np.mean(c, axis=0) - center))
    return best_cluster


def remove_ground_by_min_range(cluster, z_range=0.05):
    """
    Remove points within z_range of the minimum z (assumed to be ground).
    """
    if cluster is None or cluster.shape[0] == 0:
        return cluster
    min_z = np.min(cluster[:, 2])
    filtered = cluster[cluster[:, 2] > (min_z + z_range)]
    return filtered


def get_bounding_box_center_and_dimensions(points):
    """
    Calculate the axis-aligned bounding box's center and dimensions for a set of 3D points.
    """
    if points.shape[0] == 0:
        return None, None
    min_vals = np.min(points, axis=0)
    max_vals = np.max(points, axis=0)
    center = (min_vals + max_vals) / 2
    dimensions = max_vals - min_vals
    return center, dimensions


def create_ray_line_set(start, end):
    """
    Create an Open3D LineSet object representing a ray between two 3D points.
    The line is colored yellow.
    """
    points = [start, end]
    lines = [[0, 1]]
    line_set = o3d.geometry.LineSet()
    line_set.points = o3d.utility.Vector3dVector(points)
    line_set.lines = o3d.utility.Vector2iVector(lines)
    line_set.colors = o3d.utility.Vector3dVector([[1, 1, 0]])
    return line_set

def downsample_points(lidar_points, voxel_size=0.15):
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(lidar_points)
    down_pcd = pcd.voxel_down_sample(voxel_size=voxel_size)
    return np.asarray(down_pcd.points)

def filter_depth_points(lidar_points, max_human_depth=0.9):

    if lidar_points.shape[0] == 0:
        return lidar_points
    lidar_points_dist = lidar_points[:, 0]
    min_dist = np.min(lidar_points_dist)
    max_possible_dist = min_dist + max_human_depth
    filtered_array = lidar_points[lidar_points_dist < max_possible_dist]
    return filtered_array

def visualize_geometries(geometries, window_name="Open3D", width=800, height=600, point_size=5.0):
    """
    Visualize a list of Open3D geometry objects in a dedicated window.
    """
    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name=window_name, width=width, height=height)
    for geom in geometries:
        vis.add_geometry(geom)
    opt = vis.get_render_option()
    opt.point_size = point_size
    vis.run()
    vis.destroy_window()

def pose_to_matrix(pose):
    """
    Compose a 4x4 transformation matrix from a pose state.
    Assumes pose has attributes: x, y, z, yaw, pitch, roll,
    where the angles are given in degrees.
    """
    # Use default values if any are None (e.g. if the car is not moving)
    x = pose.x if pose.x is not None else 0.0
    y = pose.y if pose.y is not None else 0.0
    z = pose.z if pose.z is not None else 0.0
    if pose.yaw is not None and pose.pitch is not None and pose.roll is not None:
        yaw = math.radians(pose.yaw)
        pitch = math.radians(pose.pitch)
        roll = math.radians(pose.roll)
    else:
        yaw = 0.0
        pitch = 0.0
        roll = 0.0
    R_mat = R.from_euler('zyx', [yaw, pitch, roll]).as_matrix()
    T = np.eye(4)
    T[:3, :3] = R_mat
    T[:3, 3] = np.array([x, y, z])
    return T


def transform_points_l2c(lidar_points, T_l2c):
    N = lidar_points.shape[0]
    pts_hom = np.hstack((lidar_points, np.ones((N, 1))))  # (N,4)
    pts_cam = (T_l2c @ pts_hom.T).T  # (N,4)
    return pts_cam[:, :3]

# ----- New: Vectorized projection function -----
def project_points(pts_cam, K, original_lidar_points):
    """
    Vectorized version.
    pts_cam: (N,3) array of points in camera coordinates.
    original_lidar_points: (N,3) array of points in LiDAR coordinates.
    Returns a (M,5) array: [u, v, X_lidar, Y_lidar, Z_lidar] for all points with Z>0.
    """
    mask = pts_cam[:, 2] > 0
    pts_cam_valid = pts_cam[mask]
    lidar_valid = original_lidar_points[mask]
    Xc = pts_cam_valid[:, 0]
    Yc = pts_cam_valid[:, 1]
    Zc = pts_cam_valid[:, 2]
    u = (K[0, 0] * (Xc / Zc) + K[0, 2]).astype(np.int32)
    v = (K[1, 1] * (Yc / Zc) + K[1, 2]).astype(np.int32)
    proj = np.column_stack((u, v, lidar_valid))
    return proj


class YoloNode():
    """
    Detects Pedestrians by fusing YOLO 2D detections with LiDAR point cloud 
    data by painting the points. The painted data is converted to vehicle 
    frame and then published as a list of bounding boxes.

    Tracking is optional: set `enable_tracking=False` to disable persistent tracking
    and return only detections from the current frame.
    """

    def __init__(
        self,
    ):
        self.latest_image        = None
        self.latest_lidar        = None
        self.bridge              = CvBridge()
        self.camera_name = 'front'
        self.camera_front = (self.camera_name=='front')
        self.score_threshold = 0.4
        self.debug = True
        self.initialize()

    def initialize(self):
        # --- Determine the correct RGB topic for this camera ---
        rgb_topic_map = {
            'front': '/oak/rgb/image_raw',
            'front_right': '/camera_fr/arena_camera_node/image_raw',
            # add additional camera mappings here if needed
        }
        rgb_topic = rgb_topic_map.get(
            self.camera_name,
            f'/{self.camera_name}/rgb/image_raw'
        )

        # Initialize YOLO node
        rospy.init_node('yolo_box_publisher')
        # Create bounding box publisher
        self.pub = rospy.Publisher('/yolo_boxes', BoundingBoxArray, queue_size=10)
        rospy.loginfo("YOLO node initialized and waiting for messages.")

        if self.camera_front:
            self.K = np.array([[684.83331299, 0., 573.37109375],
                               [0., 684.60968018, 363.70092773],
                               [0., 0., 1.]])
        else:
            self.K = np.array([[1.17625545e+03, 0.00000000e+00, 9.66432645e+02],
                               [0.00000000e+00, 1.17514569e+03, 6.08580326e+02],
                               [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])

        if self.camera_front:
            self.D = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
        else:
            self.D = np.array([-2.70136325e-01, 1.64393255e-01, -1.60720782e-03, -7.41246708e-05,
                               -6.19939758e-02])

        self.T_l2v = np.array([[0.99939639, 0.02547917, 0.023615, 1.1],
                               [-0.02530848, 0.99965156, -0.00749882, 0.03773583],
                               [-0.02379784, 0.00689664, 0.999693, 1.95320223],
                               [0., 0., 0., 1.]])
        if self.camera_front:
            self.T_l2c = np.array([
                [0.001090, -0.999489, -0.031941, 0.149698],
                [-0.007664, 0.031932, -0.999461, -0.397813],
                [0.999970, 0.001334, -0.007625, -0.691405],
                [0., 0., 0., 1.000000]
            ])
        else:
            self.T_l2c = np.array([[-0.71836368, -0.69527204, -0.02346088, 0.05718003],
                                   [-0.09720448, 0.13371206, -0.98624154, -0.1598301],
                                   [0.68884317, -0.7061996, -0.16363744, -1.04767285],
                                   [0., 0., 0., 1.]]
                                  )
        self.T_c2l = np.linalg.inv(self.T_l2c)
        self.R_c2l = self.T_c2l[:3, :3]
        self.camera_origin_in_lidar = self.T_c2l[:3, 3]

        # Initialize the YOLO detector
        self.detector = YOLO('yolov8n.pt') # 'GEMstack/knowledge/detection/cone.pt')
        self.detector.to('cuda')

        # Subscribe to the RGB and LiDAR streams
        self.rgb_sub = Subscriber(rgb_topic, Image)
        self.lidar_sub = Subscriber('/ouster/points', PointCloud2)
        self.sync = ApproximateTimeSynchronizer([
            self.rgb_sub, self.lidar_sub
        ], queue_size=10, slop=0.1)
        self.sync.registerCallback(self.synchronized_callback)

    def synchronized_callback(self, image_msg, lidar_msg):
        rospy.loginfo("Received synchronized RGB and LiDAR messages")
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
        except Exception as e:
            rospy.logerr("Failed to convert image: {}".format(e))
            self.latest_image = None
        self.latest_lidar = pc2_to_numpy(lidar_msg, want_rgb=False)

        # Gate guards against data not being present for both sensors:
        if self.latest_image is None or self.latest_lidar is None:
            return {}
        lastest_image = self.latest_image.copy()

        downsample = False
        if downsample:
            lidar_down = downsample_points(self.latest_lidar, voxel_size=0.1)
        else:
            lidar_down = self.latest_lidar.copy()

        # if self.start_time is None:
        #     self.start_time = current_time
        # time_elapsed = current_time - self.start_time
        
        if self.camera_front == False:
            start = time.time()
            undistorted_img, current_K = self.undistort_image(lastest_image, self.K, self.D)
            end = time.time()
            # print('-------processing time undistort_image---', end -start)
            self.current_K = current_K
            orig_H, orig_W = undistorted_img.shape[:2]

            # --- Begin modifications for three-angle detection ---
            img_normal = undistorted_img
        else:
            img_normal = lastest_image.copy()
            undistorted_img = lastest_image.copy()
            orig_H, orig_W = lastest_image.shape[:2]
            self.current_K = self.K
        results_normal = self.detector(img_normal, conf=0.4, classes=[0])
        combined_boxes = []

        boxes_normal = np.array(results_normal[0].boxes.xywh.cpu()) if len(results_normal) > 0 else []
        # for box in boxes_normal:
        #     cx, cy, w, h = box
        #     combined_boxes.append((cx, cy, w, h, AgentActivityEnum.STANDING))

        start = time.time()
        # Transform the lidar points from lidar frame of reference to camera EXTRINSIC frame of reference.
        # Then project the pixels onto the lidar points to "paint them" (essentially determine which points are associated with detected objects)
        pts_cam = transform_points_l2c(lidar_down, self.T_l2c)
        projected_pts = project_points(pts_cam, self.current_K, lidar_down)
        # What is returned:
        # projected_pts[:, 0]: u-coordinate in the image (horizontal pixel position)
        # projected_pts[:, 1]: v-coordinate in the image (vertical pixel position)
        # projected_pts[:, 2:5]: original X, Y, Z coordinates in the LiDAR frame

        # Create empty list of bounding boxes to fill and publish later
        boxes = BoundingBoxArray()
        boxes.header.frame_id = 'currentVehicleFrame'
        boxes.header.stamp = lidar_msg.header.stamp

        # Process YOLO detections
        boxes_normal = np.array(results_normal[0].boxes.xywh.cpu()) if len(results_normal) > 0 else []
        conf_scores = np.array(results_normal[0].boxes.conf.cpu()) if len(results_normal) > 0 else []
    
        for i, box in enumerate(boxes_normal):
            # Skip low confidence detections
            if conf_scores[i] < self.score_threshold:
                continue
                
            # Calculate the 2D bounding box in the image
            cx, cy, w, h = box
            left = int(cx - w / 2)
            right = int(cx + w / 2)
            top = int(cy - h / 2)
            bottom = int(cy + h / 2)

            # Find LiDAR points that project to this box
            mask = (projected_pts[:, 0] >= left) & (projected_pts[:, 0] <= right) & \
                (projected_pts[:, 1] >= top) & (projected_pts[:, 1] <= bottom)
            roi_pts = projected_pts[mask]
            
            # Ignore regions with too few points
            if roi_pts.shape[0] < 5:
                continue

            # Get the 3D points corresponding to the box
            points_3d = roi_pts[:, 2:5]
            points_3d = filter_depth_points(points_3d, max_human_depth=0.8)
            refined_cluster = refine_cluster(points_3d, np.mean(points_3d, axis=0), eps=0.15, min_samples=10)
            refined_cluster = remove_ground_by_min_range(refined_cluster, z_range=0.03)

            if refined_cluster.shape[0] < 5:
                continue
                
            # Create a point cloud from the filtered points
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(refined_cluster)
            
            # Get an oriented bounding box
            obb = pcd.get_oriented_bounding_box()
            refined_center = obb.center
            dims = tuple(obb.extent)
            R_lidar = obb.R.copy()
            
            # We are assuming that dims[0] is height and dims[2] is length of obb.extent

            # Transform from LiDAR to vehicle coordinates
            refined_center_hom = np.append(refined_center, 1)
            refined_center_vehicle_hom = self.T_l2v @ refined_center_hom
            refined_center_vehicle = refined_center_vehicle_hom[:3]
            
            # Calculate rotation in vehicle frame
            R_vehicle = self.T_l2v[:3, :3] @ R_lidar
            # yaw, pitch, roll = R.from_matrix(R_vehicle).as_euler('zyx', degrees=False)
            yaw = np.arctan2(R_vehicle[1, 0], R_vehicle[0, 0])

            refined_center = refined_center_vehicle
            
            # Create a ROS BoundingBox message
            box_msg = BoundingBox()
            box_msg.header.frame_id = 'currentVehicleFrame'
            box_msg.header.stamp = lidar_msg.header.stamp
            
            # Set the pose
            box_msg.pose.position.x = float(refined_center_vehicle[0])
            box_msg.pose.position.y = float(refined_center_vehicle[1])
            box_msg.pose.position.z = float(refined_center_vehicle[2])
            
            # Convert yaw to quaternion
            quat = R.from_euler('z', yaw).as_quat()
            box_msg.pose.orientation.x = float(quat[0])
            box_msg.pose.orientation.y = float(quat[1])
            box_msg.pose.orientation.z = float(quat[2])
            box_msg.pose.orientation.w = float(quat[3])
            
            # Set the dimensions
            # Swapped dims[2] and dims[0]
            box_msg.dimensions.x = float(dims[2])  # length
            box_msg.dimensions.y = float(dims[1])  # width
            box_msg.dimensions.z = float(dims[0])  # height

            # if self.debug:
            #     print("X")
            #     print(refined_center_vehicle[0])
            #     print("L")
            #     print(dims[0])
            #     print("Y")
            #     print(refined_center_vehicle[1])
            #     print("W")
            #     print(dims[1])
            #     print("Z")
            #     print(refined_center_vehicle[2])
            #     print("H")
            #     print(dims[2])
            
            # Add confidence score and label
            box_msg.value = float(conf_scores[i])
            box_msg.label = 0  # person/pedestrian class
            
            boxes.boxes.append(box_msg)
            
            rospy.loginfo(f"Person detected at ({refined_center_vehicle[0]:.2f}, {refined_center_vehicle[1]:.2f}, {refined_center_vehicle[2]:.2f}) with score {conf_scores[i]:.2f}")
        
        # Publish the bounding boxes
        rospy.loginfo(f"Publishing {len(boxes.boxes)} person bounding boxes")
        self.pub.publish(boxes)

    def undistort_image(self, image, K, D):
        h, w = image.shape[:2]
        newK, _ = cv2.getOptimalNewCameraMatrix(K, D, (w, h), 1, (w, h))
        if self.undistort_map1 is None or self.undistort_map2 is None:
            self.undistort_map1, self.undistort_map2 = cv2.initUndistortRectifyMap(K, D, R=None,
                                                                                   newCameraMatrix=newK, size=(w, h),
                                                                                   m1type=cv2.CV_32FC1)

        start = time.time()
        undistorted = cv2.remap(image, self.undistort_map1, self.undistort_map2, interpolation=cv2.INTER_NEAREST)
        end = time.time()
        # print('--------undistort', end-start)
        return undistorted, newK


if __name__ == '__main__':
    try:
        node = YoloNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass