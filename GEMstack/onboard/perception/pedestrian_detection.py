from ...state import AllState, VehicleState, ObjectPose, ObjectFrameEnum, AgentState, AgentEnum, AgentActivityEnum
from ..interface.gem import GEMInterface
from ..component import Component
from ultralytics import YOLO
import cv2
from typing import Dict
import open3d as o3d
import numpy as np
from sklearn.cluster import DBSCAN
from scipy.spatial.transform import Rotation as R
import rospy
from sensor_msgs.msg import PointCloud2, Image
import sensor_msgs.point_cloud2 as pc2
import struct, ctypes
from message_filters import Subscriber, ApproximateTimeSynchronizer
from cv_bridge import CvBridge
import time

'''
The following function is contributed by Aavi Deora
Essentially this method is simliar to teamB's method, i.e. transform point cloud from lidar to image frame
Though we find this calculation is heavy, this method have potential usage in other parts, e.g. replacing DBSCAN
'''

# def get_average_cam_point_from_lidar(bbox, point_cloud):
#     """
#     Given a bounding box (x, y, w, h) in image coordinates and a lidar point cloud (in the lidar frame),
#     this function:
#       1. Converts the point cloud to homogeneous coordinates.
#       2. Transforms the points into the camera frame using LIDAR_TO_CAMERA_TRANSFORM.
#       3. Projects the points into the image using the camera intrinsics.
#       4. Selects points that fall inside the bounding box.
#       5. Returns the average 3D point (in the camera frame) of those points.
#
#     Returns:
#         avg_point: A NumPy array [x_cam, y_cam, z_cam] in the camera frame,
#                    or None if no points are found inside the bbox.
#     """
#     x, y, w, h = bbox
#     if point_cloud is None or point_cloud.shape[0] == 0:
#         return None
#
#     # Convert the point cloud (assumed shape (N,3)) to homogeneous coordinates (N,4)
#     ones = np.ones((point_cloud.shape[0], 1))
#     points_homog = np.hstack((point_cloud, ones))  # shape (N,4)
#
#     # Transform points from lidar frame to camera frame
#     cam_points_homog = (LIDAR_TO_CAMERA_TRANSFORM @ points_homog.T).T  # shape (N,4)
#     cam_points = cam_points_homog[:, :3]  # (x_cam, y_cam, z_cam)
#
#     # Only consider points in front of the camera (z_cam > 0)
#     valid_mask = cam_points[:, 2] > 0
#     cam_points = cam_points[valid_mask]
#     if cam_points.shape[0] == 0:
#         return None
#
#     # Project points into the image using the pinhole camera model:
#     # u = fx * (x_cam / z_cam) + cx, v = fy * (y_cam / z_cam) + cy
#     u = CAMERA_INTRINSICS['fx'] * (cam_points[:, 0] / cam_points[:, 2]) + CAMERA_INTRINSICS['cx']
#     v = CAMERA_INTRINSICS['fy'] * (cam_points[:, 1] / cam_points[:, 2]) + CAMERA_INTRINSICS['cy']
#
#     # Create a mask for points that fall within the bounding box.
#     in_bbox_mask = (u >= x) & (u <= x + w) & (v >= y) & (v <= y + h)
#     if np.sum(in_bbox_mask) == 0:
#         return None
#
#     selected_points = cam_points[in_bbox_mask]
#     return selected_points


'''
The following function is contributed by Shuning Liu. 
Since the height and velocity detection is not entirely stable, we are not using this matching logic currently.
But we may also test a more robust version of this in future on board test.
'''

# def match_existing_pedestrian(
#     new_center: np.ndarray,
#     new_dims: tuple,
#     existing_agents: Dict[str, AgentState],
#     prev_velocities: Dict[str, np.ndarray],
#     distance_threshold: float = 1.0,
#     size_threshold: float = 0.5,
#     height_threshold: float = 0.3,
#     velocity_threshold: float = 2.0
# ) -> str:
#     """
#     Match a newly detected pedestrian with an existing one using:
#     - Euclidean distance
#     - Bounding box similarity
#     - Height consistency
#     - Velocity consistency
#
#     Returns the best-matching agent_id or None if no good match is found.
#     """
#     best_agent_id = None
#     best_score = float('inf')
#
#     for agent_id, agent_state in existing_agents.items():
#         old_center = np.array([agent_state.pose.x, agent_state.pose.y, agent_state.pose.z])
#         old_dims = agent_state.dimensions
#
#         # 1. Euclidean Distance Check
#         dist = np.linalg.norm(new_center - old_center)
#         if dist > distance_threshold:
#             continue  # Skip if too far away
#
#         # 2. Bounding Box Size Similarity (with Zero-Division Handling)
#         size_norm = np.linalg.norm(np.array(old_dims))
#         if size_norm > 0:
#             size_change = np.linalg.norm(np.array(new_dims) - np.array(old_dims)) / size_norm
#         else:
#             size_change = float('inf')  # Prevent invalid matching
#
#         if size_change > size_threshold:
#             continue  # Skip if bounding box changes too much
#
#         # 3. Height Consistency Check
#         height_change = abs(new_dims[2] - old_dims[2]) / old_dims[2] if old_dims[2] > 0 else 0
#         if height_change > height_threshold:
#             continue  # Skip if height changes drastically
#
#         # 4. Velocity Consistency Check
#         if agent_id in prev_velocities:
#             prev_velocity = prev_velocities[agent_id]
#             estimated_velocity = (new_center - old_center)
#             velocity_change = np.linalg.norm(estimated_velocity - prev_velocity)
#
#             if velocity_change > velocity_threshold:
#                 continue  # Skip if unrealistic velocity jump
#
#         # Score: Lower score = better match (distance is primary factor)
#         score = dist
#         if score < best_score:
#             best_score = score
#             best_agent_id = agent_id
#
#     return best_agent_id

'''
The following function is contributed by Justin Li. An alternative to basic distance-based matching
We plan to test this new tracking logic in the next on board test
'''

class BoundingBox:
    """
    center: center of bounding box in x, y, z coordinates
    dimensions: dimensions of bounding box in x, y, z format
    orientation: 3x3 rotation matrix
    """

    def __init__(
        self,
        center: tuple[float, float, float],
        dimensions: tuple[float, float, float],
        orientation: list[list[float]],
    ):
        self.center = np.array(center)
        self.dimensions = np.array(dimensions)
        self.orientation = np.array(orientation)

def obb_collision(box1: BoundingBox, box2: BoundingBox):
    """
    Checks if two oriented bounding boxes (OBBs) collide using the Separating Axis Theorem (SAT).

    :param box1: Box with 'center' (x, y, z), 'dimensions' (dx, dy, dz), and 'orientation' (3x3 rotation matrix)
    :param box2: Box with 'center' (x, y, z), 'dimensions' (dx, dy, dz), and 'orientation' (3x3 rotation matrix)
    :return: Boolean indicating whether the two OBBs collide
    """

    def get_axes(rotation_matrix):
        """Extracts the axes from the rotation matrix."""
        return [rotation_matrix[:, i] for i in range(3)]

    def project_obb(obb: BoundingBox, axis):
        """Projects the OBB onto a given axis and returns the min and max values."""
        center_proj = np.dot(obb.center, axis)
        extents = np.abs(np.dot(obb.orientation, np.diag(obb.dimensions / 2.0)))
        radius = np.sum(extents * np.abs(axis))
        return center_proj - radius, center_proj + radius

    def overlap_on_axis(axis, box1, box2):
        """Checks if the projections of two OBBs overlap on the given axis."""
        min1, max1 = project_obb(box1, axis)
        min2, max2 = project_obb(box2, axis)
        return max1 >= min2 and max2 >= min1

    # Get OBB axes
    axes1 = get_axes(box1.orientation)
    axes2 = get_axes(box2.orientation)

    # Compute cross products of axes for possible separating axes
    cross_axes = [np.cross(a1, a2) for a1 in axes1 for a2 in axes2]

    # Test all axes
    for axis in axes1 + axes2 + cross_axes:
        if np.linalg.norm(axis) > 1e-6:  # Avoid near-zero axes
            axis = axis / np.linalg.norm(axis)  # Normalize
            if not overlap_on_axis(axis, box1, box2):
                return False  # Separating axis found, no collision

    return True  # No separating axis found, collision detected


# ----- Helper Functions -----

def match_existing_pedestrian(
        new_center: np.ndarray,
        new_dims: tuple,
        existing_agents: Dict[str, AgentState],
        distance_threshold: float = 1.0
) -> str:
    """
    Find the closest existing pedestrian agent within a specified distance threshold.

    Args:
        new_center (np.ndarray): The 3D center (x,y,z) of the new detection.
        new_dims (tuple): The dimensions of the new detection (currently unused).
        existing_agents (Dict[str, AgentState]): Dictionary of currently tracked agents.
        distance_threshold (float): Maximum allowed distance to consider a match.

    Returns:
        str: The agent_id of the best matching pedestrian if within the threshold; otherwise, None.
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


def compute_velocity(old_pose: ObjectPose, new_pose: ObjectPose, dt: float) -> tuple:
    """
    Compute the velocity vector (vx, vy, vz) based on change in pose over a time interval.

    Args:
        old_pose (ObjectPose): The previous pose.
        new_pose (ObjectPose): The current pose.
        dt (float): Time elapsed between the poses.

    Returns:
        tuple: (vx, vy, vz) velocity in the same coordinate frame as the input poses.
    """
    if dt <= 0:
        return (0, 0, 0)
    vx = (new_pose.x - old_pose.x) / dt
    vy = (new_pose.y - old_pose.y) / dt
    vz = (new_pose.z - old_pose.z) / dt
    return (vx, vy, vz)

def exponential_smooth_velocity(raw_vel, old_smooth_vel, alpha=0.3):
    """
    Applies exponential smoothing to the raw velocity.

    v_smoothed = alpha * v_raw + (1 - alpha) * v_old_smooth

    Args:
        raw_vel (tuple): The raw velocity (vx, vy, vz).
        old_smooth_vel (tuple): The previously smoothed velocity (vx_smooth, vy_smooth, vz_smooth).
        alpha (float): The smoothing factor in [0,1].

    Returns:
        tuple: The new smoothed velocity (vx_smooth, vy_smooth, vz_smooth).
    """
    vx_raw, vy_raw, vz_raw = raw_vel
    vx_old, vy_old, vz_old = old_smooth_vel

    vx_new = alpha * vx_raw + (1 - alpha) * vx_old
    vy_new = alpha * vy_raw + (1 - alpha) * vy_old
    vz_new = alpha * vz_raw + (1 - alpha) * vz_old

    return (vx_new, vy_new, vz_new)


def extract_roi_box(lidar_pc, center, half_extents):
    """
    Extract a region of interest (ROI) from the LiDAR point cloud defined by an axis-aligned bounding box.

    Args:
        lidar_pc (np.ndarray): The LiDAR point cloud as an (N, 3) array.
        center (np.ndarray): Center of the ROI box.
        half_extents (np.ndarray): Half-lengths in each dimension defining the box.

    Returns:
        np.ndarray: Subset of points within the ROI.
    """
    lower = center - half_extents
    upper = center + half_extents
    mask = np.all((lidar_pc >= lower) & (lidar_pc <= upper), axis=1)
    return lidar_pc[mask]


def pc2_to_numpy(pc2_msg, want_rgb=False):
    """
    Convert a ROS PointCloud2 message into a numpy array with basic filtering.

    This function extracts the x, y, z coordinates from the point cloud and
    filters out points with x <= 0 and z >= 2.5.

    Args:
        pc2_msg: The ROS PointCloud2 message.
        want_rgb (bool): Flag to indicate if RGB data is desired (not used here).

    Returns:
        np.ndarray: Filtered point cloud array of shape (N, 3).
    """
    gen = pc2.read_points(pc2_msg, skip_nans=True)
    pts = np.array(list(gen), dtype=np.float32)
    pts = pts[:, :3]  # Only x, y, z coordinates
    mask = (pts[:, 0] > 0) & (pts[:, 2] < 2.5)
    return pts[mask]


def backproject_pixel(u, v, K):
    """
    Backproject a pixel coordinate (u, v) into a normalized 3D ray in the camera coordinate system.

    Args:
        u (float): The pixel's x-coordinate.
        v (float): The pixel's y-coordinate.
        K (np.ndarray): The 3x3 camera intrinsic matrix.

    Returns:
        np.ndarray: A unit vector representing the 3D ray direction.
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
    Identify the center of a human detected along a projected ray.

    This function first filters the LiDAR points to only those near the ray. Then it
    sweeps along the ray (from t_min to t_max in steps of t_step) to find a candidate
    location where enough points (>= min_points) lie within the distance_threshold.

    Args:
        lidar_pc (np.ndarray): LiDAR point cloud in 3D.
        ray_origin (np.ndarray): Origin of the ray in LiDAR coordinates.
        ray_direction (np.ndarray): Normalized direction of the ray.
        t_min (float): Minimum distance along the ray to start searching.
        t_max (float): Maximum distance along the ray.
        t_step (float): Increment along the ray.
        distance_threshold (float): Maximum distance from the ray for a point to be considered.
        min_points (int): Minimum number of points needed to validate a candidate.
        ransac_threshold (float): (Unused in current implementation) threshold for RANSAC.

    Returns:
        tuple: (refined_candidate, None, None) if a valid candidate is found; otherwise, (None, None, None).
    """
    # Compute projection distances and the corresponding points on the ray.
    vecs = lidar_pc - ray_origin
    proj_lengths = np.dot(vecs, ray_direction)
    proj_points = ray_origin + np.outer(proj_lengths, ray_direction)
    dists_to_ray = np.linalg.norm(lidar_pc - proj_points, axis=1)
    near_ray_mask = dists_to_ray < distance_threshold
    filtered_pc = lidar_pc[near_ray_mask]

    if filtered_pc.shape[0] < min_points:
        return None, None, None

    # Sweep along the ray to locate a cluster of points.
    t_values = np.arange(t_min, t_max, t_step)
    for t in t_values:
        candidate = ray_origin + t * ray_direction
        dists = np.linalg.norm(filtered_pc - candidate, axis=1)
        nearby_points = filtered_pc[dists < distance_threshold]
        if nearby_points.shape[0] >= min_points:
            refined_candidate = np.mean(nearby_points, axis=0)
            return refined_candidate, None, None
    return None, None, None


def extract_roi(pc, center, roi_radius):
    """
    Extract points from a point cloud that lie within a specified radius of a center point.

    Args:
        pc (np.ndarray): The point cloud array.
        center (np.ndarray): The 3D center point.
        roi_radius (float): The radius of the region of interest.

    Returns:
        np.ndarray: Points from pc that are within roi_radius of center.
    """
    distances = np.linalg.norm(pc - center, axis=1)
    return pc[distances < roi_radius]


def refine_cluster(roi_points, center, eps=0.2, min_samples=10):
    """
    Refine a point cluster by applying DBSCAN and selecting the cluster closest to a given center.

    Args:
        roi_points (np.ndarray): Points within a region of interest.
        center (np.ndarray): The expected center of the cluster.
        eps (float): The maximum distance between two samples for one to be considered in the neighborhood of the other.
        min_samples (int): The number of samples required in a neighborhood for a point to be considered as a core point.

    Returns:
        np.ndarray: The refined cluster points. If no clusters are found, returns the original points.
    """
    clustering = DBSCAN(eps=eps, min_samples=min_samples).fit(roi_points)
    labels = clustering.labels_
    valid_clusters = [roi_points[labels == l] for l in set(labels) if l != -1]
    if not valid_clusters:
        return roi_points
    best_cluster = min(valid_clusters, key=lambda c: np.linalg.norm(np.mean(c, axis=0) - center))
    return best_cluster


def remove_ground_by_min_range(cluster, z_range=0.05):
    """
    Remove ground points from a cluster by excluding points that lie close to the minimum z value.

    Args:
        cluster (np.ndarray): The input point cluster.
        z_range (float): The threshold range above the minimum z-value to consider as ground.

    Returns:
        np.ndarray: The cluster with ground points removed.
    """
    if cluster is None or cluster.shape[0] == 0:
        return cluster
    min_z = np.min(cluster[:, 2])
    filtered = cluster[cluster[:, 2] > (min_z + z_range)]
    return filtered


def get_bounding_box_center_and_dimensions(points):
    """
    Calculate the axis-aligned bounding box's center and dimensions for a set of 3D points.

    Args:
        points (np.ndarray): The input points.

    Returns:
        tuple: (center, dimensions) where center is the midpoint of the bounding box and dimensions is (max - min).
               Returns (None, None) if the input is empty.
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

    Args:
        start (np.ndarray): The start point of the ray.
        end (np.ndarray): The end point of the ray.

    Returns:
        o3d.geometry.LineSet: The LineSet object representing the ray.
    """
    points = [start, end]
    lines = [[0, 1]]
    line_set = o3d.geometry.LineSet()
    line_set.points = o3d.utility.Vector3dVector(points)
    line_set.lines = o3d.utility.Vector2iVector(lines)
    line_set.colors = o3d.utility.Vector3dVector([[1, 1, 0]])
    return line_set


def visualize_geometries(geometries, window_name="Open3D", width=800, height=600, point_size=5.0):
    """
    Visualize a list of Open3D geometry objects in a dedicated window.

    Args:
        geometries (list): A list of Open3D geometry objects.
        window_name (str): Title of the visualization window.
        width (int): Width of the window.
        height (int): Height of the window.
        point_size (float): Size of the rendered points.
    """
    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name=window_name, width=width, height=height)
    for geom in geometries:
        vis.add_geometry(geom)
    opt = vis.get_render_option()
    opt.point_size = point_size
    vis.run()
    vis.destroy_window()


# ----- Pedestrian Detector 2D (with 3D Fusion and Synchronized Callbacks) -----

class PedestrianDetector2D(Component):
    """
    Detects pedestrians by fusing 2D YOLO detections with LiDAR point cloud data to estimate 3D poses.

    This implementation uses message_filters to synchronize incoming RGB images and LiDAR data.
    The synchronized callback caches the latest sensor data for processing in the update() method.
    """

    def __init__(self, vehicle_interface: GEMInterface):
        self.vehicle_interface = vehicle_interface
        self.current_agents = {}
        self.tracked_agents = {}
        self.pedestrian_counter = 0

        # Add a dictionary to hold smoothed velocities:
        # Key: agent_id, Value: (vx_smooth, vy_smooth, vz_smooth)
        self.smoothed_velocities = {} 

        # Variables to store the most recent synchronized sensor data:
        self.latest_image = None
        self.latest_lidar = None
        self.bridge = CvBridge()

    def rate(self) -> float:
        # Process data at 4 Hz.
        return 4.0

    def state_inputs(self) -> list:
        # Expects vehicle state information as input.
        return ['vehicle']

    def state_outputs(self) -> list:
        # Outputs detected pedestrian agents.
        return ['agents']

    def initialize(self):
        """
        Initialize sensor subscriptions and the YOLO detector.

        Uses message_filters to synchronize the image and LiDAR data streams.
        Also sets up the camera intrinsic matrix and LiDAR-to-camera (and vehicle) transformations.
        """
        self.rgb_sub = Subscriber('/oak/rgb/image_raw', Image)
        self.lidar_sub = Subscriber('/ouster/points', PointCloud2)
        self.sync = ApproximateTimeSynchronizer([self.rgb_sub, self.lidar_sub],
                                                queue_size=10, slop=0.1)
        self.sync.registerCallback(self.synchronized_callback)
        # Initialize YOLO detector with the pretrained model and set to CUDA.
        self.detector = YOLO('../../knowledge/detection/yolov8s.pt')
        self.detector.to('cuda')
        # Camera intrinsic matrix.
        self.K = np.array([[684.83331299, 0., 573.37109375],
                           [0., 684.60968018, 363.70092773],
                           [0., 0., 1.]])
        # LiDAR-to-vehicle transformation matrix.
        self.T_l2v = np.array([[0.99939639, 0.02547917, 0.023615, 1.1],
                               [-0.02530848, 0.99965156, -0.00749882, 0.03773583],
                               [-0.02379784, 0.00689664, 0.999693, 1.95320223],
                               [0., 0., 0., 1.]])
        # LiDAR-to-camera transformation matrix.
        self.T_l2c = np.array([
            [0.001090, -0.999489, -0.031941, 0.149698],
            [-0.007664, 0.031932, -0.999461, -0.397813],
            [0.999970, 0.001334, -0.007625, -0.691405],
            [0.000000, 0.000000, 0.000000, 1.000000]
        ])
        # Inverse transformation: camera-to-LiDAR.
        self.T_c2l = np.linalg.inv(self.T_l2c)
        self.R_c2l = self.T_c2l[:3, :3]
        # Camera origin expressed in LiDAR coordinates.
        self.camera_origin_in_lidar = self.T_c2l[:3, 3]

    def synchronized_callback(self, image_msg, lidar_msg):
        """
        Callback function that is invoked when an image and a LiDAR message are received within the allowed time difference.

        This function converts the ROS messages into usable formats:
          - The image is converted to an OpenCV BGR image.
          - The LiDAR message is converted to a numpy array.
        """
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
        except Exception as e:
            rospy.logerr("Failed to convert image: {}".format(e))
            self.latest_image = None
        self.latest_lidar = pc2_to_numpy(lidar_msg, want_rgb=False)


    def downsample_points(self,points: np.ndarray, voxel_size=0.1) -> np.ndarray:
        """
        Downsample a point cloud using Open3D's voxel downsampling.
        points: Nx3 numpy array
        voxel_size: size of each voxel grid, e.g. 0.1
        """
        if points.shape[0] == 0:
            return points

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        downsampled_pcd = pcd.voxel_down_sample(voxel_size=voxel_size)
        return np.asarray(downsampled_pcd.points)
    
    def transform_lidar_to_camera(self,points_lidar: np.ndarray, T_l2c: np.ndarray) -> np.ndarray:
        """
        Transform Nx3 lidar points into the camera frame using T_l2c (4x4).
        Returns an Nx3 array in camera coordinates.
        """
        if points_lidar.shape[0] == 0:
            return points_lidar

        # Convert Nx3 into Nx4 homogeneous
        ones = np.ones((points_lidar.shape[0], 1), dtype=np.float32)
        points_hom = np.hstack((points_lidar, ones))  # shape (N,4)

        # Transform
        points_cam_hom = (T_l2c @ points_hom.T).T  # shape (N,4)
        # Extract X, Y, Z
        points_cam = points_cam_hom[:, :3]
        return points_cam
    
    def project_points_to_image(self,points_cam: np.ndarray, K: np.ndarray) -> np.ndarray:
        """
        Project Nx3 camera-frame points onto 2D image plane using intrinsics K.
        Returns an Nx5 array: [u, v, X, Y, Z].
        Only includes points with Z>0 (in front of the camera).
        """
        if points_cam.shape[0] == 0:
            return np.empty((0, 5), dtype=np.float32)

        # Prepare output list
        projected = []
        fx, fy = K[0, 0], K[1, 1]
        cx, cy = K[0, 2], K[1, 2]

        for pt in points_cam:
            X, Y, Z = pt
            if Z <= 0:
                continue  # behind the camera, skip
            u = fx * (X / Z) + cx
            v = fy * (Y / Z) + cy
            projected.append([u, v, X, Y, Z])

        return np.array(projected, dtype=np.float32)
    


    def update(self, vehicle: VehicleState) -> Dict[str, AgentState]:
        """
        Process the latest synchronized sensor data to detect pedestrians and estimate their 3D poses.

        This method performs the following:
          1. Runs YOLO on the latest image to obtain 2D pedestrian detections.
          2. Backprojects the 2D detections into 3D rays in the LiDAR frame.
          3. Uses LiDAR data along the ray to estimate a 3D position (and size) for the pedestrian.
          4. Matches detections to existing tracked agents or creates new ones.

        Args:
            vehicle (VehicleState): The current vehicle state.

        Returns:
            Dict[str, AgentState]: Dictionary mapping agent IDs to their updated states.
        """
        # Ensure both image and LiDAR data are available.
        if self.latest_image is None or self.latest_lidar is None:
            return {}

        current_time = self.vehicle_interface.time()
        # Run YOLO inference on the current image (detecting only class 0, e.g., persons).
        results = self.detector(self.latest_image, conf=0.4, classes=[0])
        boxes = np.array(results[0].boxes.xywh.cpu())  # Format: [center_x, center_y, width, height]

        agents = {}
        lidar_pc = self.latest_lidar.copy()
        lidar_pc_down = self.downsample_points(lidar_pc, voxel_size=0.1)
        points_cam = self.transform_lidar_to_camera(lidar_pc_down, self.T_l2c)
        projected_points = self.project_points_to_image(points_cam, self.K)


        for i, box in enumerate(boxes):
            cx, cy, w, h = box
            left   = int(cx - w/2)
            right  = int(cx + w/2)
            top    = int(cy - h/2)
            bottom = int(cy + h/2)
            in_bbox_mask = (
                (projected_points[:, 0] >= left) &
                (projected_points[:, 0] <= right) &
                (projected_points[:, 1] >= top) &
                (projected_points[:, 1] <= bottom)
            )
            subset_2d = projected_points[in_bbox_mask]
            if subset_2d.shape[0] == 0:
            # No LiDAR points are inside this bounding box, skip
                continue
            points_cam_3d = subset_2d[:, 2:5]  # just [X_cam, Y_cam, Z_cam]
            ones = np.ones((points_cam_3d.shape[0], 1), dtype=np.float32)
            points_cam_hom = np.hstack((points_cam_3d, ones))
            # Invert T_l2c => T_c2l
            T_c2l = np.linalg.inv(self.T_l2c)
            points_lidar_hom = (T_c2l @ points_cam_hom.T).T
            points_lidar_cluster = points_lidar_hom[:, :3]  # shape (K, 3) in LiDAR frame

            # 8) (Optional) refine cluster by removing ground, DBSCAN, etc.
            # e.g., remove ground with min z-range
            refined_cluster = remove_ground_by_min_range(points_lidar_cluster, z_range=0.03)
            if refined_cluster is None or refined_cluster.shape[0] == 0:
                continue
              # e.g., DBSCAN refinement
            intersection = np.mean(refined_cluster, axis=0)  # approximate center
            refined_cluster = refine_cluster(refined_cluster, intersection, eps=0.15, min_samples=10)
            if refined_cluster.shape[0] <= 5:
            # skip if cluster is too small
                continue
            center, dims = get_bounding_box_center_and_dimensions(refined_cluster)
            if center is None:
                continue
            center_lidar_hom = np.array([center[0], center[1], center[2], 1.0])
            center_vehicle_hom = self.T_l2v @ center_lidar_hom
            center_vehicle = center_vehicle_hom[:3]  # (x, y, z) in vehicle frame
                # curr_x = vehicle.pose.x
                # curr_y = vehicle.pose.y
                # curr_yaw = vehicle.pose.yaw
                # curr_pitch = vehicle.pose.pitch
                # curr_roll = vehicle.pose.roll
                # refined_center[0] += curr_x
                # refined_center[1] += curr_y
                # euler_angles_vehicle[0] += curr_yaw
                # euler_angles_vehicle[1] += curr_pitch
                # euler_angles_vehicle[2] += curr_roll

            # Create a new pose for the detected pedestrian in the vehicle frame.
            new_pose = ObjectPose(
                t=current_time,
                x=center_vehicle[0],
                y=center_vehicle[1],
                z=center_vehicle[2],
                yaw=0,
                pitch=0,
                roll=0,
                frame=ObjectFrameEnum.CURRENT
            )

            # Attempt to match the new detection with an existing tracked pedestrian.
            existing_id = match_existing_pedestrian(
                new_center=np.array([new_pose.x, new_pose.y, new_pose.z]),
                new_dims=dims,
                existing_agents=self.tracked_agents,
                distance_threshold=2.0
            )

            if existing_id is not None:
                # 1) Get the old agent state
                old_agent_state = self.tracked_agents[existing_id]
                dt = new_pose.t - old_agent_state.pose.t

                # 2) Compute raw velocity
                vx_raw, vy_raw, vz_raw = compute_velocity(old_agent_state.pose, new_pose, dt)

                # 3) Retrieve the old smoothed velocity (or initialize if not available)
                old_smooth_vel = self.smoothed_velocities.get(existing_id, (0.0, 0.0, 0.0))

                # 4) Exponential smoothing
                alpha = 0.3  # Tweak this to find a good balance
                vx_smooth, vy_smooth, vz_smooth = exponential_smooth_velocity(
                    (vx_raw, vy_raw, vz_raw),
                    old_smooth_vel,
                    alpha=alpha
                )

                # 5) Update the dictionary with the new smoothed velocity
                self.smoothed_velocities[existing_id] = (vx_smooth, vy_smooth, vz_smooth)

                # 6) Create the updated agent with the smoothed velocity
                updated_agent = AgentState(
                    pose=new_pose,
                    dimensions=dims,
                    outline=None,
                    type=AgentEnum.PEDESTRIAN,
                    activity=AgentActivityEnum.MOVING,
                    velocity=(vx_smooth, vy_smooth, vz_smooth),
                    yaw_rate=0
                )

                # 7) Save it in both the local dict and self.tracked_agents
                agents[existing_id] = updated_agent
                self.tracked_agents[existing_id] = updated_agent

            else:
                # If no match is found, create a new pedestrian agent.
                agent_id = f"pedestrian{self.pedestrian_counter}"
                self.pedestrian_counter += 1

                new_agent = AgentState(
                    pose=new_pose,
                    dimensions=dims,
                    outline=None,
                    type=AgentEnum.PEDESTRIAN,
                    activity=AgentActivityEnum.MOVING,
                    velocity=(0, 0, 0),
                    yaw_rate=0
                )
                agents[agent_id] = new_agent
                self.tracked_agents[agent_id] = new_agent
                self.smoothed_velocities[agent_id] = (0.0, 0.0, 0.0)

        self.current_agents = agents

        stale_ids = [agent_id for agent_id, agent in self.tracked_agents.items()
                     if current_time - agent.pose.t > 5.0]
        for agent_id in stale_ids:
            rospy.loginfo(f"Removing stale agent: {agent_id}")
            del self.tracked_agents[agent_id]
        # Log the details of each detected agent.
        for agent_id, agent in agents.items():
            rospy.loginfo(f"Agent ID: {agent_id}, Pose: {agent.pose}, Velocity: {agent.velocity}")

        return agents


# ----- Fake Pedestrian Detector 2D (for Testing Purposes) -----

class FakePedestrianDetector2D(Component):
    """
    A dummy pedestrian detector that simulates detections during pre-defined time intervals.

    This component is useful for testing the overall system without relying on real sensor data.
    """

    def __init__(self, vehicle_interface: GEMInterface):
        self.vehicle_interface = vehicle_interface
        self.times = [(5.0, 20.0), (30.0, 35.0)]
        self.t_start = None

    def rate(self):
        return 4.0

    def state_inputs(self):
        return ['vehicle']

    def state_outputs(self):
        return ['agents']

    def update(self, vehicle: VehicleState) -> Dict[str, AgentState]:
        if self.t_start is None:
            self.t_start = self.vehicle_interface.time()
        t = self.vehicle_interface.time() - self.t_start
        res = {}
        for time_range in self.times:
            if t >= time_range[0] and t <= time_range[1]:
                res['pedestrian0'] = box_to_fake_agent((0, 0, 0, 0))
                rospy.loginfo("Detected a pedestrian (simulated)")
        return res


def box_to_fake_agent(box):
    """
    Create a fake AgentState from a bounding box represented as (x, y, width, height).

    The pose is computed using the center of the box, and the dimensions are treated as 2D approximations.

    Args:
        box (tuple): A tuple (x, y, w, h) representing the bounding box.

    Returns:
        AgentState: The simulated pedestrian agent.
    """
    x, y, w, h = box
    pose = ObjectPose(t=0, x=x + w / 2, y=y + h / 2, z=0, yaw=0, pitch=0, roll=0, frame=ObjectFrameEnum.CURRENT)
    dims = (w, h, 0)
    return AgentState(pose=pose, dimensions=dims, outline=None,
                      type=AgentEnum.PEDESTRIAN, activity=AgentActivityEnum.MOVING,
                      velocity=(0, 0, 0), yaw_rate=0)


if __name__ == '__main__':
    # This module is intended to be used within the vehicle interface context.
    # For standalone testing, instantiate a fake vehicle state and call update().
    pass
