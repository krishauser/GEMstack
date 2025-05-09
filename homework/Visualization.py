#!/usr/bin/env python3
import os
import glob
import cv2
import numpy as np
import open3d as o3d
from ultralytics import YOLO
from sklearn.cluster import DBSCAN
from scipy.spatial.transform import Rotation as R
import time
import matplotlib.pyplot as plt

# -----------------------------
# 1) Minimal Stub Classes
# -----------------------------
class AgentActivityEnum():
    # Activity enumerations (cones or pedestrians, etc.)
    STOPPED = 0         # e.g., stationary objects (pedestrians standing, parked cars, etc.)
    MOVING = 1          # normally moving objects (predictions will be used here)
    FAST = 2            # moving faster than usual (e.g., runners)
    UNDETERMINED = 3    # unknown activity
    STANDING = 4        # our label for a cone detected in normal orientation
    LEFT = 5            # label for a cone detected after right rotation (i.e., originally rotated left)
    RIGHT = 6           # label for a cone detected after left rotation (i.e., originally rotated right)

class ObjectPose:
    # Pose representation for detected objects
    def __init__(self, t, x, y, z, yaw, pitch, roll, frame):
        self.t = t  # time stamp
        self.x = x  # x coordinate
        self.y = y  # y coordinate
        self.z = z  # z coordinate
        self.yaw = yaw      # yaw angle (heading)
        self.pitch = pitch  # pitch angle
        self.roll = roll    # roll angle
        self.frame = frame  # coordinate frame identifier

class AgentState:
    # State representation for an agent (cone, pedestrian, etc.)
    def __init__(self, pose, dimensions, outline, type, activity, velocity, yaw_rate):
        self.pose = pose            # an ObjectPose instance
        self.dimensions = dimensions  # dimensions (width, height, depth)
        self.outline = outline      # contour or outline (if available)
        self.type = type            # type of agent (e.g., cone, pedestrian)
        self.activity = activity    # activity label, see AgentActivityEnum
        self.velocity = velocity    # velocity tuple (vx, vy, vz)
        self.yaw_rate = yaw_rate    # yaw rate (angular velocity)

# -----------------------------
# 2) Helper Functions
# -----------------------------
def undistort_image(image, K, D):
    """
    Undistort an input image using the camera matrix (K) and distortion coefficients (D).
    Returns the undistorted image and the new camera matrix.
    """
    h, w = image.shape[:2]
    newK, _ = cv2.getOptimalNewCameraMatrix(K, D, (w, h), 1, (w, h))
    undistorted = cv2.undistort(image, K, D, None, newK)
    return undistorted, newK

def cylindrical_roi(points, center, radius, height):
    """
    Select points from the point cloud that are within a cylindrical region of interest (ROI).
    The cylinder is defined on the horizontal plane with center[:2] as its center (assume x,y),
    a given radius, and vertically centered at center[2] with a total height of 'height'.
    """
    horizontal_dist = np.linalg.norm(points[:, :2] - center[:2], axis=1)
    vertical_diff = np.abs(points[:, 2] - center[2])
    mask = (horizontal_dist <= radius) & (vertical_diff <= height / 2)
    return points[mask]

def downsample_points(lidar_points, voxel_size=0.15):
    """
    Downsample the LiDAR point cloud using a voxel grid filter.
    """
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(lidar_points)
    down_pcd = pcd.voxel_down_sample(voxel_size=voxel_size)
    return np.asarray(down_pcd.points)

def load_start_pose(vehstate_filepath):
    """
    Read the line starting with 'AFTER_TRANSFORM' from the given file and
    return an ObjectPose in the START frame.
    """
    with open(vehstate_filepath, 'r') as f:
        for line in f:
            if line.startswith("AFTER_TRANSFORM"):
                # line looks like:
                # AFTER_TRANSFORM x=17.551, y=0.164, z=0.016, yaw=0.31, pitch=-0.02, roll=-0.01, frame=START
                parts = line.strip().split()
                kvs = {}
                for token in parts[1:]:
                    key, val = token.rstrip(',').split('=')
                    kvs[key] = val
                return ObjectPose(
                    t=None,
                    x=float(kvs['x']),
                    y=float(kvs['y']),
                    z=float(kvs['z']),
                    yaw=float(kvs['yaw']),
                    pitch=float(kvs['pitch']),
                    roll=float(kvs['roll']),
                    frame=kvs['frame']
                )
    raise RuntimeError(f"No AFTER_TRANSFORM line found in {vehstate_filepath}")


def visualize_z_distribution(points_3d, bins=50):
    """
    Visualize the distribution of Z values in a point cloud.

    Args:
        points_3d (np.ndarray): Array of shape (N,3) with XYZ coordinates.
        bins (int): Number of histogram bins.
    """
    z_vals = points_3d[:, 2]
    plt.figure()
    plt.hist(z_vals, bins=bins)
    plt.xlabel("Z coordinate")
    plt.ylabel("Frequency")
    plt.title("Z Distribution of Points")
    plt.show()


def transform_points_l2c(lidar_points, T_l2c):
    """
    Transform LiDAR points from LiDAR coordinates to camera coordinates using the transformation matrix T_l2c.
    """
    N = lidar_points.shape[0]
    pts_hom = np.hstack((lidar_points, np.ones((N, 1))))  # create homogeneous coordinates (N,4)
    pts_cam = (T_l2c @ pts_hom.T).T
    return pts_cam[:, :3]

def project_points(pts_cam, K, original_lidar_points):
    """
    Project points (in camera coordinates) onto the image plane using camera intrinsics.
    Only points with positive Z (in front of the camera) are projected.
    Returns an array of shape (M, 5): [u, v, X_lidar, Y_lidar, Z_lidar].
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

def project_all_points(points, T_l2c, K):
    """
    For all input points, project them onto the image plane using the transformation matrix T_l2c and camera intrinsic K.
    Returns an (N,2) array with [u, v] coordinates for each point. For points with Z<=0, returns [-1, -1].
    """
    N = points.shape[0]
    pts_hom = np.hstack((points, np.ones((N,1))))
    pts_cam = (T_l2c @ pts_hom.T).T
    u = np.full((N,), -1, dtype=int)
    v = np.full((N,), -1, dtype=int)
    valid = pts_cam[:, 2] > 0
    if np.any(valid):
        Xc = pts_cam[valid, 0]
        Yc = pts_cam[valid, 1]
        Zc = pts_cam[valid, 2]
        u[valid] = (K[0, 0] * (Xc / Zc) + K[0, 2]).astype(int)
        v[valid] = (K[1, 1] * (Yc / Zc) + K[1, 2]).astype(int)
    return np.stack((u, v), axis=1)

def visualize_custom_points(image, points_lidar, T_l2c, K, color=(0, 255, 0)):
    """
    Visualize custom LiDAR points on the given image.
    Projects the LiDAR points (using T_l2c and camera intrinsics K) and draws circles on the image.
    """
    N = points_lidar.shape[0]
    pts_hom = np.hstack((points_lidar, np.ones((N, 1))))
    pts_cam = (T_l2c @ pts_hom.T).T[:, :3]
    for p in pts_cam:
        X, Y, Z = p
        if Z <= 0:
            continue
        u = int(K[0, 0] * (X / Z) + K[0, 2])
        v = int(K[1, 1] * (Y / Z) + K[1, 2])
        cv2.circle(image, (u, v), 5, color, -1)
    return image

def filter_points_within_threshold(points, threshold=15.0):
    """
    Filter out points that are farther than the threshold.
    """
    distances = np.linalg.norm(points, axis=1)
    mask = distances <= threshold
    return points[mask]

def refine_cluster(roi_points, center, eps=0.2, min_samples=10):
    """
    Refine a cluster of points using DBSCAN clustering and return the cluster closest to 'center'.
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
    Remove points in the cluster that are within a specified z-range from the minimum z value (assumed to be ground).
    """
    if cluster is None or cluster.shape[0] == 0:
        return cluster
    min_z = np.min(cluster[:, 2])
    return cluster[cluster[:, 2] > (min_z + z_range)]

def match_existing_pedestrian(new_center, new_dims, existing_agents, distance_threshold=1.0):
    """
    Find the closest already tracked agent (cone/pedestrian) to the new_center, within the distance threshold.
    Returns the agent ID if found.
    """
    best_agent_id = None
    best_dist = float('inf')
    for agent_id, agent_state in existing_agents.items():
        old_center = np.array([agent_state.pose.x, agent_state.pose.y, agent_state.pose.z])
        dist = np.linalg.norm(new_center - old_center)
        if dist < distance_threshold and dist < best_dist:
            best_dist = dist
            best_agent_id = agent_id
    return best_agent_id

def compute_velocity(old_pose, new_pose, dt):
    """
    Compute velocity (vx, vy, vz) based on the difference between the new and old poses divided by dt.
    """
    if dt <= 0:
        return (0, 0, 0)
    vx = (new_pose.x - old_pose.x) / dt
    vy = (new_pose.y - old_pose.y) / dt
    vz = (new_pose.z - old_pose.z) / dt
    return (vx, vy, vz)

def filter_depth_points(lidar_points, max_depth_diff=0.9, use_norm=True):
    """
    Filters LiDAR points based on a maximum allowed depth difference.
    """
    if lidar_points.shape[0] == 0:
        return lidar_points
    if use_norm:
        depths = np.linalg.norm(lidar_points, axis=1)
    else:
        depths = lidar_points[:, 0]
    min_depth = np.min(depths)
    max_possible_depth = min_depth + max_depth_diff
    mask = depths < max_possible_depth
    return lidar_points[mask]

def display_reprojected_cluster(image, refined_cluster, T_l2c, K):
    """
    Reproject 3D points (refined_cluster) into the image and draw small circles for debugging.
    """
    if refined_cluster is None or refined_cluster.shape[0] == 0:
        return image
    N = refined_cluster.shape[0]
    pts_hom = np.hstack((refined_cluster, np.ones((N, 1))))
    pts_cam = (T_l2c @ pts_hom.T).T[:, :3]
    for p in pts_cam:
        X, Y, Z = p
        if Z <= 0:
            continue
        u = int(K[0, 0] * X / Z + K[0, 2])
        v = int(K[1, 1] * Y / Z + K[1, 2])
        cv2.circle(image, (u, v), 2, (255, 0, 0), -1)
    return image

def pose_to_matrix(pose):
    """
    Convert a pose to a 4x4 transformation matrix.
    The pose attributes (x, y, z, yaw, pitch, roll) are used,
    with angles in degrees.
    """
    x = pose.x if pose.x is not None else 0.0
    y = pose.y if pose.y is not None else 0.0
    z = pose.z if pose.z is not None else 0.0
    if pose.yaw is not None and pose.pitch is not None and pose.roll is not None:
        yaw = pose.yaw
        pitch = np.radians(pose.pitch)
        roll = np.radians(pose.roll)
    else:
        yaw = pitch = roll = 0.0
    R_mat = R.from_euler('zyx', [yaw, pitch, roll]).as_matrix()
    T = np.eye(4)
    T[:3, :3] = R_mat
    T[:3, 3] = np.array([x, y, z])
    return T

def transform_points_l2c(lidar_points, T_l2c):
    """
    Transform LiDAR points to camera coordinates.
    """
    N = lidar_points.shape[0]
    pts_hom = np.hstack((lidar_points, np.ones((N, 1))))  # (N,4)
    pts_cam = (T_l2c @ pts_hom.T).T  # (N,4)
    return pts_cam[:, :3]

# ----- New: Vectorized projection function -----
def project_points(pts_cam, K, original_lidar_points):
    """
    Vectorized version of projecting LiDAR points (in camera space) to the image plane.
    Returns an (M, 5) array: [u, v, X_lidar, Y_lidar, Z_lidar] for points with positive Z.
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

def remove_rows(A, B, decimals=3):
    """
    Remove rows that are the same in A and B up to a given decimal precision.
    """
    A_round = np.round(A, decimals=decimals)
    B_round = np.round(B, decimals=decimals)
    A_view = {tuple(row) for row in A_round}
    B_view = {tuple(row) for row in B_round}
    diff = A_view - B_view
    diff = np.array(list(diff))
    return diff

# -----------------------------
# 3) PedestrianDetector2D (New Method)
# -----------------------------
class PedestrianDetector2D:
    """
    New approach:
      1) Downsample LiDAR point cloud.
      2) Transform the point cloud to camera coordinate system.
      3) Project all points onto the image plane.
      4) Use YOLO 2D detection boxes to filter LiDAR points.
      5) Use DBSCAN clustering and ground removal.
      6) Compute oriented bounding boxes and transform them to the vehicle frame.
      7) (Optional) Reproject the refined cluster onto the image (for debugging).
      8) (Optional) Visualize 3D frustum (back-project 2D bounding box into 3D).
    """
    def __init__(self, model_path='yolov8n.pt'):
        self.detector = YOLO(model_path)
        self.camera_front = True
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
            self.D = np.array([-2.70136325e-01,  1.64393255e-01, -1.60720782e-03, -7.41246708e-05,
  -6.19939758e-02])
        self.T_l2v = np.array([[0.99939639, 0.02547917, 0.023615, 1.1],
                                [-0.02530848, 0.99965156, -0.00749882, 0.03773583],
                                [-0.02379784, 0.00689664, 0.999693, 1.95320223],
                                [0., 0., 0., 1.]])
        if self.camera_front:
            self.T_l2c = np.array([[2.89748006e-02, -9.99580136e-01, 3.68439439e-05, -3.07300513e-02],
                [-9.49930618e-03, -3.12215512e-04, -9.99954834e-01, -3.86689354e-01],
                [9.99534999e-01, 2.89731321e-02, -9.50437214e-03, -6.71425124e-01],
                [0.00000000e+00, 0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])
        else:
            self.T_l2c = np.array([[-0.71836368, -0.69527204, -0.02346088,  0.05718003],
     [-0.09720448,  0.13371206, -0.98624154, -0.1598301 ],
     [ 0.68884317, -0.7061996 , -0.16363744, -1.04767285],
     [ 0.        ,  0.        ,  0.        ,  1.        ]])
        self.tracked_agents = {}
        self.pedestrian_counter = 0
        self.current_K = self.K.copy()
        self.debug_frustums = []
        self.cluster_geometries = []
        self.cluster_obbs = []
        self.others_geometry = None
        # Flag: whether to use cylindrical ROI option.
        self.use_cyl_roi = True

    def process_frame(self, image, lidar_points, current_time=0.0, debug_reproj=False, debug_frustum=False,
                      start_pose_abs = None):
        agents = {}
        self.cluster_geometries = []
        self.cluster_obbs = []
        self.debug_frustums = []

        # 1. Undistort the input image and record its original dimensions.
        image, newK = undistort_image(image, self.K, self.D)
        self.current_K = newK
        orig_H, orig_W = image.shape[:2]

        # 2. Create rotated images (for detection only, not for final display):
        #    img_normal: the undistorted normal image.
        #    img_left: left-rotated (counterclockwise 90°).
        #    img_right: right-rotated (clockwise 90°).
        image_left = cv2.rotate(image.copy(), cv2.ROTATE_90_COUNTERCLOCKWISE)
        image_right = cv2.rotate(image.copy(), cv2.ROTATE_90_CLOCKWISE)

        # 3. Run YOLO detections on each image.
        #    Detection box format is [cx, cy, w, h].
        results_normal = self.detector(image, conf=0.4, classes=[0])
        results_left = self.detector(image_left, conf=0.1, classes=[0])
        results_right = self.detector(image_right, conf=0.1, classes=[0])

        boxes_normal = np.array(results_normal[0].boxes.xywh.cpu()) if len(results_normal) > 0 else []
        boxes_left = np.array(results_left[0].boxes.xywh.cpu()) if len(results_left) > 0 else []
        boxes_right = np.array(results_right[0].boxes.xywh.cpu()) if len(results_right) > 0 else []

        # 3.1 Combine boxes from all three angles.
        # For the normal image, no mapping is performed: activity = STANDING.
        # For the left rotated image (counterclockwise 90°):
        #   In the left-rotated image, detection center is (cx, cy).
        #   Inverse mapping to original coordinates is: (x, y) = (orig_W - 1 - c_y, c_x).
        #   Also, swap width and height. Activity is set to RIGHT.
        # For the right rotated image (clockwise 90°):
        #   Inverse mapping: (x, y) = (c_y, orig_H - 1 - c_x). Swap width and height.
        #   Activity is set to LEFT.
        combined_boxes = []
        for box in boxes_normal:
            cx, cy, w, h = box
            combined_boxes.append((cx, cy, w, h, AgentActivityEnum.STANDING))
        for box in boxes_left:
            cx, cy, w, h = box
            new_cx = orig_W - 1 - cy
            new_cy = cx
            new_w = h  # Swap width and height.
            new_h = w
            combined_boxes.append((new_cx, new_cy, new_w, new_h, AgentActivityEnum.RIGHT))
        for box in boxes_right:
            cx, cy, w, h = box
            new_cx = cy
            new_cy = orig_H - 1 - cx
            new_w = h  # Swap width and height.
            new_h = w
            combined_boxes.append((new_cx, new_cy, new_w, new_h, AgentActivityEnum.LEFT))

        # 4. Preprocess LiDAR point cloud.
        lidar_down = lidar_points.copy()
        global_filtered = filter_points_within_threshold(lidar_down, 50)
        pts_cam = transform_points_l2c(lidar_down, self.T_l2c)
        projected_pts = project_points(pts_cam, self.current_K, lidar_down)

        # 5. For each combined 2D bounding box, process LiDAR points to obtain 3D position and orientation.
        refined_all = []
        if lidar_points.shape[0] > 0:
            roi_pcd = o3d.geometry.PointCloud()
            roi_pcd.points = o3d.utility.Vector3dVector(lidar_points)
            # this will pop up a window showing just those points
            o3d.visualization.draw_geometries(
                [roi_pcd],
                window_name="ROI 3D Points",
                width=800,
                height=600
            )
        for box_info in combined_boxes:
            cx, cy, w, h, activity = box_info
            print(cx, cy, w, h)
            left = int(cx - w / 1.5)
            right = int(cx + w / 1.5)
            top = int(cy - h / 2)
            bottom = int(cy + h / 2)

            # --- [Optional] Project 2D bounding box into 3D to create a frustum, if debug_frustum is set ---
            if debug_frustum:
                bbox = [left, top, right, bottom]
                z_near = 0.5  # can be adjusted
                z_far = 30.0  # can be adjusted
                pts_frustum, lines_frustum = self.create_frustum_lines(bbox, self.current_K, z_near, z_far)
                frustum_lineset = o3d.geometry.LineSet()
                frustum_lineset.points = o3d.utility.Vector3dVector(pts_frustum)
                frustum_lineset.lines = o3d.utility.Vector2iVector(lines_frustum)
                # Color all lines red
                frustum_lineset.colors = o3d.utility.Vector3dVector([[1, 0, 0] for _ in range(len(lines_frustum))])
                self.debug_frustums.append(frustum_lineset)
            # -------------------------------------------------------------------------------------

            # Draw the 2D bounding box and label on the (original) image.
            if activity == AgentActivityEnum.STANDING:
                color = (255, 0, 0)  # Blue
                label_text = "STANDING"
            elif activity == AgentActivityEnum.RIGHT:
                color = (0, 255, 0)  # Green
                label_text = "RIGHT"
            elif activity == AgentActivityEnum.LEFT:
                color = (0, 0, 255)  # Red
                label_text = "LEFT"
            else:
                color = (255, 255, 255)
                label_text = "UNKNOWN"
            cv2.rectangle(image, (left, top), (right, bottom), color, 2)
            cv2.putText(image, label_text, (left, max(top - 5, 20)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

            # Extract the LiDAR points that project within the ROI.
            mask = (projected_pts[:, 0] >= left) & (projected_pts[:, 0] <= right) & \
                   (projected_pts[:, 1] >= top) & (projected_pts[:, 1] <= bottom)
            roi_2d_pts = projected_pts[mask]
            points_3d = roi_2d_pts[:, 2:5]  # Extract the original 3D coordinates

            if points_3d.shape[0] > 0:
                roi_pcd = o3d.geometry.PointCloud()
                roi_pcd.points = o3d.utility.Vector3dVector(points_3d)
                # this will pop up a window showing just those points
                o3d.visualization.draw_geometries(
                    [roi_pcd],
                    window_name="ROI 3D Points",
                    width=800,
                    height=600
                )
            if debug_reproj:
                display_reprojected_cluster(image, points_3d, self.T_l2c, self.current_K)
            visualize_z_distribution(points_3d)
            points_3d = filter_points_within_threshold(points_3d, 100)
            # print(points_3d)
            points_3d = remove_ground_by_min_range(points_3d, z_range=0.08)
            points_3d = filter_depth_points(points_3d, max_depth_diff=0.5)

            if points_3d.shape[0] > 0:
                roi_pcd = o3d.geometry.PointCloud()
                roi_pcd.points = o3d.utility.Vector3dVector(points_3d)
                # this will pop up a window showing just those points
                o3d.visualization.draw_geometries(
                    [roi_pcd],
                    window_name="ROI 3D Points",
                    width=800,
                    height=600
                )
            if points_3d.shape[0] < 4:
                continue

            # --- Modification: Optional Cylindrical ROI ---
            # If cylindrical ROI is enabled, use global_filtered points to form a cylindrical ROI.
            center_roi = np.mean(points_3d, axis=0)
            if self.use_cyl_roi:
                roi_cyl = cylindrical_roi(global_filtered, center_roi, radius=0.4, height=1.2)
                refined_cluster = remove_ground_by_min_range(roi_cyl, z_range=0.01)
                refined_cluster = filter_depth_points(refined_cluster, max_depth_diff=0.3)
                center_roi = np.mean(refined_cluster, axis=0)
            else:
                refined_cluster = points_3d.copy()
            if refined_cluster.shape[0] < 4:
                continue
            # -------------------------------------------------

            refined_all.append(refined_cluster)
            # Create an Open3D point cloud for the refined cluster (colored red).
            pcd_cluster = o3d.geometry.PointCloud()
            pcd_cluster.points = o3d.utility.Vector3dVector(refined_cluster)
            pcd_cluster.paint_uniform_color([1, 0, 0])
            self.cluster_geometries.append(pcd_cluster)
            # Compute the oriented bounding box (OBB) for the refined cluster (set to magenta).
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(refined_cluster)
            obb = pcd.get_oriented_bounding_box()
            obb.color = (1, 0, 1)
            self.cluster_obbs.append(obb)

            refined_center = obb.center
            dims = tuple(obb.extent)
            R_lidar = obb.R.copy()
            refined_center_hom = np.append(refined_center, 1)
            refined_center_vehicle_hom = self.T_l2v @ refined_center_hom
            refined_center_vehicle = refined_center_vehicle_hom[:3]

            R_vehicle = self.T_l2v[:3, :3] @ R_lidar
            # Convert rotation matrix to Euler angles (in degrees)
            euler_vehicle = R.from_matrix(R_vehicle).as_euler('zyx', degrees=True)
            yaw, pitch, roll = euler_vehicle

            if start_pose_abs is not None:
                T_vehicle_to_start = pose_to_matrix(start_pose_abs)

                # transform your point (vehicle‐frame) into START frame
                p_start_hom = T_vehicle_to_start @ np.append(refined_center_vehicle, 1.0)
                xp, yp, zp = p_start_hom[:3]

                # overwrite or store for output
                refined_center_vehicle = np.array([xp, yp, zp])

            new_pose = ObjectPose(
                t=current_time,
                x=refined_center_vehicle[0],
                y=refined_center_vehicle[1],
                z=refined_center_vehicle[2],
                yaw=yaw,
                pitch=pitch,
                roll=roll,
                frame=0
            )
            # Match with an existing tracked cone if possible.
            existing_id = match_existing_pedestrian(
                np.array([new_pose.x, new_pose.y, new_pose.z]),
                dims,
                self.tracked_agents,
                distance_threshold=0.5
            )
            if existing_id is not None:
                old_state = self.tracked_agents[existing_id]
                dt = current_time - old_state.pose.t
                vx, vy, vz = compute_velocity(old_state.pose, new_pose, dt)
                updated_agent = AgentState(
                    pose=new_pose,
                    dimensions=dims,
                    outline=None,
                    type=0,
                    activity=activity,
                    velocity=(vx, vy, vz),
                    yaw_rate=0
                )
                agents[existing_id] = updated_agent
                self.tracked_agents[existing_id] = updated_agent
            else:
                agent_id = f"pedestrian{self.pedestrian_counter}"
                self.pedestrian_counter += 1
                new_agent = AgentState(
                    pose=new_pose,
                    dimensions=dims,
                    outline=None,
                    type=0,
                    activity=activity,
                    velocity=(0, 0, 0),
                    yaw_rate=0
                )
                agents[agent_id] = new_agent
                self.tracked_agents[agent_id] = new_agent

        # Optionally, display custom debug points on the image.
        # visualize_custom_points(image, np.array([
        #     [4.730639, 10.195478, -1.941095],
        #     [2.066050, 7.066111, -2.027844],
        #     [1.341566, 7.118239, -1.994539],
        #     [4.160140, 6.539731, -1.895874],
        #     [6.469934, 6.128805, -1.859030],
        # ]), self.T_l2c, self.current_K, color=(0, 0, 255))
        cv2.imshow("Frame with Boxes and Custom Points", image)
        cv2.waitKey(1)

        # Process "other points": from global_filtered, find points that do not fall within any ROI.
        projected_global = project_all_points(global_filtered, self.T_l2c, self.current_K)
        others_mask = np.zeros(len(global_filtered), dtype=bool)
        for box_info in combined_boxes:
            cx, cy, w, h, _ = box_info
            left = int(cx - w / 2)
            right = int(cx + w / 2)
            top = int(cy - h / 2)
            bottom = int(cy + h / 2)
            in_box = (projected_global[:, 0] >= left) & (projected_global[:, 0] <= right) & \
                     (projected_global[:, 1] >= top) & (projected_global[:, 1] <= bottom)
            others_mask = others_mask | in_box
        others_mask = ~others_mask
        others = global_filtered[others_mask]
        others_pc = o3d.geometry.PointCloud()
        others_pc.points = o3d.utility.Vector3dVector(others)
        others_pc.paint_uniform_color([0.5, 0.5, 0.5])
        self.others_geometry = others_pc

        # Return the final detected agents and the image with drawn bounding boxes and labels.
        return agents, image

    def create_frustum_lines(self, box, K, z_near, z_far):
        """
        Create the points and line indices that form a 3D frustum by back-projecting the 2D bounding box.
        'box' is [left, top, right, bottom].
        Returns an array of 3D points and a list of lines (indices into the points array).
        """
        left, top, right, bottom = box
        corners = np.array([
            [left, top],
            [right, top],
            [right, bottom],
            [left, bottom]
        ])
        fx = K[0, 0]
        fy = K[1, 1]
        cx = K[0, 2]
        cy = K[1, 2]
        near_points = []
        far_points = []
        for (u, v) in corners:
            x_near = (u - cx) / fx * z_near
            y_near = (v - cy) / fy * z_near
            near_pt = np.array([x_near, y_near, z_near, 1.0])
            near_points.append(near_pt)
            x_far = (u - cx) / fx * z_far
            y_far = (v - cy) / fy * z_far
            far_pt = np.array([x_far, y_far, z_far, 1.0])
            far_points.append(far_pt)
        near_points = np.array(near_points)
        far_points = np.array(far_points)
        T_c2l = np.linalg.inv(self.T_l2c)
        near_points_lidar = (T_c2l @ near_points.T).T[:, :3]
        far_points_lidar = (T_c2l @ far_points.T).T[:, :3]
        all_points = np.vstack((near_points_lidar, far_points_lidar))
        lines = [[i, i + 4] for i in range(4)]
        lines += [[0, 1], [1, 2], [2, 3], [3, 0]]
        lines += [[4, 5], [5, 6], [6, 7], [7, 4]]
        return all_points, lines

# New helper: Ensure to project all global_filtered points into image coordinates.
def project_all_points(points, T_l2c, K):
    N = points.shape[0]
    pts_hom = np.hstack((points, np.ones((N,1))))
    pts_cam = (T_l2c @ pts_hom.T).T
    u = np.full((N,), -1, dtype=int)
    v = np.full((N,), -1, dtype=int)
    valid = pts_cam[:, 2] > 0
    if np.any(valid):
        Xc = pts_cam[valid, 0]
        Yc = pts_cam[valid, 1]
        Zc = pts_cam[valid, 2]
        u[valid] = (K[0, 0] * (Xc / Zc) + K[0, 2]).astype(int)
        v[valid] = (K[1, 1] * (Yc / Zc) + K[1, 2]).astype(int)
    return np.stack((u, v), axis=1)

# -----------------------------
# 4) Function to load LiDAR data from an NPZ file.
# -----------------------------
def load_lidar_from_npz(file_path):
    data = np.load(file_path)
    print(data.keys())
    return data['lidar']

# -----------------------------
# 5) Main function: runs process_frame and visualizes refined clusters and other point cloud geometries.
#    This part also loads images and LiDAR data (modify paths as needed).
# -----------------------------
def main():
    # Set whether to use the cylindrical ROI option (True or False).
    use_cyl_roi_flag = True
    detector = PedestrianDetector2D(model_path='cone.pt')
    detector.use_cyl_roi = use_cyl_roi_flag

    # Initialize Open3D visualization for LiDAR point cloud geometries.
    vis = o3d.visualization.VisualizerWithKeyCallback()
    vis.create_window(window_name="3D Frame Display", width=800, height=600)
    view_path = "last_view.json"
    if os.path.exists(view_path):
        param = o3d.io.read_pinhole_camera_parameters(view_path)
    else:
        param = None

    # Read image and LiDAR data (please modify the paths as needed).
    # image_files = sorted(glob.glob(os.path.join('../data', 'color*.png')))
    # lidar_files = sorted(glob.glob(os.path.join('../data', 'lidar*.npz')))
    # num_frames = min(len(image_files), len(lidar_files))
    # if num_frames == 0:
    #     print("No matching data found.")
    #     return

    # print(f"Found {num_frames} matching frames.")
    for i in range(1):
        # Use fixed file paths as an example; modify as needed.
        current_time_ms = 1746802907471
        image_path = f'../5.9/{current_time_ms}_image.png'
        lidar_path = f'../5.9/{current_time_ms}_lidar.npz'

        try:
            veh_txt = f"../4.30/{current_time_ms}_vehstate.txt"
            start_pose_abs = load_start_pose(veh_txt)
        except:
            start_pose_abs = None

        image = cv2.imread(image_path)
        if image is None:
            print("Failed to load image:", image_path)
            continue
        lidar_points = load_lidar_from_npz(lidar_path)

        agents, processed_image = detector.process_frame(image, lidar_points, current_time=float(i),
                                                         debug_reproj=True, debug_frustum=True, start_pose_abs = start_pose_abs)
        # Print detected agent information.
        print("Detected Agents:")
        for agent_id, agent_state in agents.items():
            p = agent_state.pose
            orientation = {AgentActivityEnum.STANDING: "STANDING",
                           AgentActivityEnum.LEFT: "LEFT",
                           AgentActivityEnum.RIGHT: "RIGHT"}.get(agent_state.activity, "UNKNOWN")
            print(
                f"Agent {agent_id}: Position=({p.x:.2f}, {p.y:.2f}, {p.z:.2f}), Orientation={orientation}, Dimensions={agent_state.dimensions}")

        # Display the 2D image with bounding boxes and labels.
        cv2.imshow("Detection - Normal2D", processed_image)
        cv2.waitKey(1)

        # Open3D visualization: show refined clusters, OBB, and other points.
        geometries = []
        if detector.cluster_geometries:
            geometries.extend(detector.cluster_geometries)
        if detector.cluster_obbs:
            geometries.extend(detector.cluster_obbs)
        if detector.others_geometry is not None:
            geometries.append(detector.others_geometry)
        if detector.debug_frustums:
            geometries.extend(detector.debug_frustums)

        vis.clear_geometries()
        for geom in geometries:
            vis.add_geometry(geom)
        if param is not None:
            vis.get_view_control().convert_from_pinhole_camera_parameters(param)
        else:
            print("No previous view, using default.")
        vis.run()  # Block until the user closes the visualizer window.
        param = vis.get_view_control().convert_to_pinhole_camera_parameters()
        o3d.io.write_pinhole_camera_parameters(view_path, param)
        vis.clear_geometries()
    vis.destroy_window()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
