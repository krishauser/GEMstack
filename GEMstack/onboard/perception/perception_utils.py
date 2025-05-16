from typing import Dict
import open3d as o3d
import numpy as np
from sklearn.cluster import DBSCAN
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import ros_numpy
import math


# ----- Helper Functions -----

def cylindrical_roi(points, center, radius, height):
    horizontal_dist = np.linalg.norm(points[:, :2] - center[:2], axis=1)
    vertical_diff = np.abs(points[:, 2] - center[2])
    mask = (horizontal_dist <= radius) & (vertical_diff <= height / 2)
    return points[mask]


def filter_points_within_threshold(points, threshold=15.0):
    distances = np.linalg.norm(points, axis=1)
    mask = distances <= threshold
    return points[mask]


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
    # Stack x,y,z fields to a (N,3) array
    pts = np.stack((np.array(pc['x']).ravel(),
                    np.array(pc['y']).ravel(),
                    np.array(pc['z']).ravel()), axis=1)
    # Apply filtering (for example, x > 0 and z in a specified range)
    mask = (pts[:, 0] > -0.5) & (pts[:, 2] < -1) & (pts[:, 2] > -2.7)
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


def filter_depth_points(lidar_points, max_depth_diff=0.9, use_norm=True):
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

def calculate_3d_iou(box1, box2, get_corners_cb, get_volume_cb):
    """
    Calculates the 3D Intersection over Union (IoU) between two bounding boxes.

    Args:
        box1, box2: List or tuple representing a 3D bounding box in the
                    *internal standardized format*: [cx, cy, cz, l, w, h, yaw, ...]
                    where cy is the geometric center y.
        get_corners_cb: Callback function to extract AABB corners from a box
        get_volume_cb: Callback function to calculate volume of a box

    Returns:
        float: The 3D IoU value.

    Doesn't consider yaw
    """

    ######### Simple Axis-Aligned Bounding Box (AABB) IoU#
    min_x1, max_x1, min_y1, max_y1, min_z1, max_z1 = get_corners_cb(box1)
    min_x2, max_x2, min_y2, max_y2, min_z2, max_z2 = get_corners_cb(box2)

    # Calculate intersection volume
    inter_min_x = max(min_x1, min_x2)
    inter_max_x = min(max_x1, max_x2)
    inter_min_y = max(min_y1, min_y2)
    inter_max_y = min(max_y1, max_y2)
    inter_min_z = max(min_z1, min_z2)
    inter_max_z = min(max_z1, max_z2)

    inter_l = max(0, inter_max_x - inter_min_x)
    inter_h = max(0, inter_max_y - inter_min_y)
    inter_w = max(0, inter_max_z - inter_min_z)
    intersection_volume = inter_l * inter_h * inter_w

    # Calculate union volume
    vol1 = get_volume_cb(box1)
    vol2 = get_volume_cb(box2)
    # Ensure volumes are positive and non-zero for stable IoU
    vol1 = max(vol1, 1e-6)
    vol2 = max(vol2, 1e-6)
    union_volume = vol1 + vol2 - intersection_volume
    union_volume = max(union_volume, 1e-6) # Avoid division by zero or very small numbers

    iou = intersection_volume / union_volume
    # Clamp IoU to [0, 1] range
    iou = max(0.0, min(iou, 1.0))
    return iou