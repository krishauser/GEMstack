from sensor_msgs.msg import PointCloud2
import numpy as np
import sensor_msgs.point_cloud2 as pc2
import open3d as o3d
import cv2
import json


def convert_pointcloud2_to_xyz(lidar_pc2_msg: PointCloud2):
    """ Convert 1D PointCloud2 data to x, y, z coords """
    return np.array(list(pc2.read_points(lidar_pc2_msg, skip_nans=True)), dtype=np.float32)[:, :3]


# Credits: The following lines of codes (17 to 159 excluding lines 80 to 115) are adapted from the Calibration Team B
def load_extrinsics(extrinsics_file):
    """
    Load calibrated extrinsics from a .npz file.
    Assumes the file contains keys 'R' and 't'.
    """
    data = np.load(extrinsics_file)
    return data


def load_intrinsics(intrinsics_file):
    """
    Load camera intrinsics from a JSON file.
    Expects keys: 'fx', 'fy', 'cx', and 'cy'.
    """
    with open(intrinsics_file, 'r') as f:
        intrinsics = json.load(f)
    fx = intrinsics["fx"]
    fy = intrinsics["fy"]
    cx = intrinsics["cx"]
    cy = intrinsics["cy"]
    K = np.array([[fx, 0, cx],
                  [0, fy, cy],
                  [0,  0,  1]], dtype=np.float32)
    return K


def load_lidar_scan(lidar_file):
    """
    Load a LiDAR scan from a file.
    This function handles both npy (returns a NumPy array) and npz files.
    """
    data = np.load(lidar_file, allow_pickle=True)
    if isinstance(data, np.ndarray):
        return data.astype(np.float32)
    else:
        key = list(data.keys())[0]
        return data[key].astype(np.float32)


def transform_lidar_points(lidar_points, R, t):
    """
    Transform LiDAR points from the LiDAR frame into the camera frame.
    p_cam = R * p_lidar + t.
    """
    P_cam = (R @ lidar_points.T + t.reshape(3,1)).T
    return P_cam


def project_points(points_3d, K):
    """
    Project 3D points (in the camera frame) into 2D image coordinates using the camera matrix K.
    Only projects points with z > 0.
    """
    proj_points = []
    for pt in points_3d:
        if pt[2] > 0:  # only project points in front of the camera
            u = K[0, 0] * (pt[0] / pt[2]) + K[0, 2]
            v = K[1, 1] * (pt[1] / pt[2]) + K[1, 2]
            proj_points.append((int(u), int(v)))
    return proj_points


def vis_2d_bbox(image, xywh, box):
    # Setup
    label_text = "Pedestrian "
    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = 0.5
    font_color = (255, 255, 255)
    line_type = 1
    text_thickness = 2

    x, y, w, h = xywh

    if box.id is not None:
        id = box.id.item()
    else:
        id = 0

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

    # Draw main text on top of the outline
    cv2.putText(image, label, (text_x, text_y - baseline), font, font_scale, font_color, text_thickness)

    return image


def visualize_point_cloud(points):
    """
    Visualizes the given point cloud using Open3D.

    Args:
        points (np.ndarray): Nx3 array of point cloud coordinates.
    """
    pc = o3d.geometry.PointCloud()
    pc.points = o3d.utility.Vector3dVector(points)
    pc.paint_uniform_color([0.1, 0.7, 0.9])  # Light blue color
    o3d.visualization.draw_geometries([pc])


def visualize_plane(inlier_cloud, outlier_cloud, bounding_box_2d_points):
    """
    Visualizes the detected plane with its 2D bounding box.

    :param inlier_cloud: Open3D point cloud containing plane points.
    :param outlier_cloud: Open3D point cloud containing non-plane points.
    :param bounding_box_2d_points: 4 corner points of the 2D bounding box on the plane.
    """
    inlier_cloud.paint_uniform_color([1, 0, 0])  # Red for the plane
    outlier_cloud.paint_uniform_color([0.5, 0.5, 0.5])  # Gray for other points

    # Create bounding box visualization
    bounding_box_pcd = o3d.geometry.PointCloud()
    bounding_box_pcd.points = o3d.utility.Vector3dVector(bounding_box_2d_points)
    bounding_box_pcd.paint_uniform_color([0, 1, 0])  # Green for bounding box corners
    
    # Create a bounding box line set (connect corners)
    lines = [
        [0, 1], [1, 2], [2, 3], [3, 0]   # Edges of the rectangle
    ]
    
    bounding_box_lines = o3d.geometry.LineSet()
    bounding_box_lines.points = o3d.utility.Vector3dVector(bounding_box_2d_points)
    bounding_box_lines.lines = o3d.utility.Vector2iVector(lines)
    
    bounding_box_lines.paint_uniform_color([0, 1, 0])  # Green for bounding box edges
    
    # Visualize
    o3d.visualization.draw_geometries([inlier_cloud, outlier_cloud, bounding_box_pcd, bounding_box_lines])