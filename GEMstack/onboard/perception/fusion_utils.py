from sensor_msgs.msg import PointCloud2, PointField
import numpy as np
import sensor_msgs.point_cloud2 as pc2
import open3d as o3d
import cv2
import json
import rospy
import numpy as np
import struct


def convert_pointcloud2_to_xyz(lidar_pc2_msg: PointCloud2):
    """ Convert 1D PointCloud2 data to x, y, z coords """
    return np.array(list(pc2.read_points(lidar_pc2_msg, skip_nans=True)), dtype=np.float32)[:, :3]


def downsample_points(lidar_points):
    """ Downsample point clouds """
    # Convert numpy array to Open3D point cloud
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(lidar_points)

    # Apply voxel grid downsampling
    voxel_size = 0.1  # Adjust for desired resolution
    downsampled_pcd = pcd.voxel_down_sample(voxel_size=voxel_size)
    
    # Convert back to numpy array
    transformed_points = np.asarray(downsampled_pcd.points)
    return transformed_points


def filter_ground_points(lidar_points, ground_threshold = 0):
    """ Filter points given an elevation of ground threshold """
    filtered_array = lidar_points[lidar_points[:, 3] < ground_threshold]
    return filtered_array


def filter_far_points(lidar_points, max_dist_percent=0.85):
    """ Filter points beyond a percentage threshold of max distance in a point cluster """
    max_dist = np.max(lidar_points[:, 4])
    filtered_array = lidar_points[lidar_points[:, 4] < max_dist_percent * max_dist]
    return filtered_array


def calculate_centroid(points):
    """
    Calculate the centroid of a cluster of points in 3D space.
    
    :param points: List of points in the format [[x1, y1, z1], [x2, y2, z2], ...]
    :return: The centroid as a list [cx, cy, cz]
    """
    if not points:
        return None  # Return None if the list is empty

    num_points = len(points)
    centroid = [sum(coord) / num_points for coord in zip(*points)]
    
    return centroid


# Credits: The following lines of codes (from 33 to 92) are adapted from the Calibration Team B
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
            # 5D points that stores original lidar points
            proj_points.append((int(u), int(v), pt[0], pt[1], pt[2]))
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


def create_point_cloud(points, color=(255, 0, 0)):
    """
    Converts a list of (x, y, z) points into a PointCloud2 message.
    """
    header = rospy.Header()
    header.stamp = rospy.Time.now()
    header.frame_id = "map"  # Change to your TF frame

    fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="rgb", offset=12, datatype=PointField.FLOAT32, count=1),
    ]

    # Convert RGB color to packed float32
    r, g, b = color
    packed_color = struct.unpack('f', struct.pack('I', (r << 16) | (g << 8) | b))[0]

    point_cloud_data = [(x, y, z, packed_color) for x, y, z in points]

    return pc2.create_cloud(header, fields, point_cloud_data)