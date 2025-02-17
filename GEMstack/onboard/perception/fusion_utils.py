from sensor_msgs.msg import PointCloud2, PointField
from visualization_msgs.msg import Marker, MarkerArray
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


def filter_ground_points(lidar_points, ground_threshold=0):
    """ Filter points given an elevation of ground threshold """
    filtered_array = lidar_points[lidar_points[:, 3] < ground_threshold]
    return filtered_array


def filter_depth_points(lidar_points, max_human_depth=0.9):
    """ Filter points beyond a max possible human depth in a point cluster """
    lidar_points_dist = lidar_points[:, 4]
    min_dist = np.min(lidar_points_dist)
    max_dist = np.max(lidar_points_dist)
    max_possible_dist = min_dist + max_human_depth
    actual_dist = min(max_dist, max_possible_dist)
    filtered_array = lidar_points[lidar_points_dist < actual_dist]
    return filtered_array


def calculate_centroid(points):
    """
    Calculate the centroid of a cluster of points in 3D space.
    
    :param points: List of points in the format [[x1, y1, z1], [x2, y2, z2], ...]
    :return: The centroid as a list [cx, cy, cz]
    """
    if not points:
        return None
    
    # Convert to numpy array for efficiency
    points_array = np.array(points)
    
    # Compute centroid
    centroid_point = np.mean(points_array, axis=0)
    return centroid_point.tolist()


def calculate_dimensions(points):
    """
    Calculate the dimensions of a cluster of points in 3D space.
    
    :param points: List of points in the format [[x1, y1, z1], [x2, y2, z2], ...]
    :return: the dimensions as a list [length (forward), width (sideways), height (up)]
    """
    if not points:
        return None
    
    # Convert to numpy array for efficiency
    points_array = np.array(points)
    
    # Extract min and max values along each axis
    min_vals = np.min(points_array, axis=0)
    max_vals = np.max(points_array, axis=0)
    
    # Compute dimensions
    dimensions = max_vals - min_vals
    return dimensions.tolist()


def calculate_bbox_corners_3d(centroid, dimensions):
    """
    Compute the 8 corners of a 3D bounding box given its centroid and dimensions.
    
    Parameters:
        centroid (list or np.array): [cx, cy, cz] - Center coordinates of the bounding box.
        dimensions (list or np.array): [length, width, height] - Dimensions of the bounding box.
        
    Returns:
        list: (8,3) array containing the 8 corners of the bounding box.
    """
    if (not centroid) or (not dimensions):
        return None
    
    centroid = np.array(centroid)
    dimensions = np.array(dimensions) / 2  # Half dimensions to calculate corners
    
    # Define the 8 corner offsets
    offsets = np.array([
        [-1, -1, -1],
        [-1, -1,  1],
        [-1,  1, -1],
        [-1,  1,  1],
        [ 1, -1, -1],
        [ 1, -1,  1],
        [ 1,  1, -1],
        [ 1,  1,  1]
    ])
    
    # Compute the corners
    corners = centroid + offsets * dimensions
    return corners.tolist()


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


def create_bbox_marker(centroids, dimensions):
    """
    Create 3D bbox markers from centroids and dimensions
    """
    marker_array = MarkerArray()

    for i, (centroid, dimension) in enumerate(zip(centroids, dimensions)):
        # Skip if no centroid or dimension
        if (centroid == None) or (dimension == None):
            continue

        marker = Marker()
        marker.header.frame_id = "map"  # Reference frame
        marker.header.stamp = rospy.Time.now()
        marker.ns = "bounding_boxes"
        marker.id = i  # Unique ID for each marker
        marker.type = Marker.CUBE  # Cube for bounding box
        marker.action = Marker.ADD

        # Position (center of the bounding box)
        marker.pose.position.x = centroid[0]
        marker.pose.position.y = centroid[1]
        marker.pose.position.z = centroid[2]

        # Orientation (default, no rotation)
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        # Bounding box dimensions
        marker.scale.x = dimension[0]
        marker.scale.y = dimension[1]
        marker.scale.z = dimension[2]

        # Random colors for each bounding box
        marker.color.r = 0.0  # Varying colors
        marker.color.g = 1.0
        marker.color.b = 1.5
        marker.color.a = 0.2  # Transparency

        marker.lifetime = rospy.Duration()  # Persistent
        marker_array.markers.append(marker)
    return marker_array


def delete_bbox_marker():
    """
    Delete 3D bbox markers given ID ranges
    """
    marker_array = MarkerArray()
    for i in range(6):
        marker = Marker()
        marker.ns = "bounding_boxes"
        marker.id = i
        marker.action = Marker.DELETE
        marker_array.markers.append(marker)
    return marker_array

