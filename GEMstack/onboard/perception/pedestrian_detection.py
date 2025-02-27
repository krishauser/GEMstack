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

        for i, box in enumerate(boxes):
            cx, cy, w, h = box
            # Backproject the 2D pixel center into a 3D ray in the LiDAR coordinate frame.
            ray_dir_cam = backproject_pixel(cx, cy, self.K)
            ray_dir_lidar = self.R_c2l @ ray_dir_cam
            ray_dir_lidar /= np.linalg.norm(ray_dir_lidar)

            # Attempt to locate the pedestrian's center along the ray using LiDAR data.
            intersection, _, _ = find_human_center_on_ray(
                lidar_pc, self.camera_origin_in_lidar, ray_dir_lidar,
                t_min=0.4, t_max=25.0, t_step=0.1,
                distance_threshold=0.5, min_points=5, ransac_threshold=0.05
            )
            if intersection is None:
                # If no valid intersection is found, update the positions of already tracked agents
                # based on the vehicle's forward velocity and time elapsed.
                for agent in self.tracked_agents.values():
                    dt = current_time - agent.pose.t
                    agent.pose.x -= vehicle.v * dt  # Adjust x-coordinate assuming vehicle moves forward along x.
                    agent.pose.t = current_time
                continue

            # Estimate the pedestrian's physical dimensions using the detection box size and distance.
            d = np.linalg.norm(intersection - self.camera_origin_in_lidar)
            physical_width = (w * d) / self.K[0, 0]
            physical_height = (h * d) / self.K[1, 1]
            # Since the depth (x-direction) measurement is less reliable, use an empirical value.
            half_extents = np.array([0.4, 1.1 * physical_width / 2, 1.1 * physical_height / 2])

            # Extract LiDAR points within the ROI box around the estimated intersection.
            roi_points = extract_roi_box(lidar_pc, intersection, half_extents)
            if roi_points.shape[0] < 10:
                refined_cluster = roi_points
            else:
                refined_cluster = refine_cluster(roi_points, intersection, eps=0.15, min_samples=10)

            # Remove ground points from the refined cluster.
            refined_cluster = remove_ground_by_min_range(refined_cluster, z_range=0.03)
            if refined_cluster is None or refined_cluster.shape[0] == 0:
                # Fallback to the initial intersection if no valid cluster remains.
                refined_center = intersection
                dims = (0, 0, 0)
                yaw, pitch, roll = 0, 0, 0
            elif refined_cluster.shape[0] <= 5:
                # Skip detections with insufficient LiDAR points.
                continue
            else:
                # Create an Open3D point cloud from the refined cluster and compute its oriented bounding box.
                pcd = o3d.geometry.PointCloud()
                pcd.points = o3d.utility.Vector3dVector(refined_cluster)
                obb = pcd.get_oriented_bounding_box()
                refined_center = obb.center
                dims = tuple(obb.extent)
                R_lidar = obb.R.copy()

                # Transform the refined center from LiDAR coordinates to the vehicle frame.
                refined_center_lidar_hom = np.array([refined_center[0],
                                                     refined_center[1],
                                                     refined_center[2],
                                                     1.0])
                refined_center_vehicle_hom = self.T_l2v @ refined_center_lidar_hom
                refined_center_vehicle = refined_center_vehicle_hom[:3]

                # Compute the orientation (yaw, pitch, roll) in the vehicle frame.
                R_vehicle = self.T_l2v[:3, :3] @ R_lidar
                euler_angles_vehicle = R.from_matrix(R_vehicle).as_euler('zyx', degrees=False)
                yaw, pitch, roll = euler_angles_vehicle
                refined_center = refined_center_vehicle  # Use vehicle frame coordinates for output

                '''
                Note that this part can be used for converting the speed relative to vehicle -> speed relative to START frame
                However, the VehicleState may read in Global coordinate (i.e. long and lat) instead of relative to START frame
                In order to avoid this issue, we found it is actually easier to implement this in the planning code.
                Planning function has access to Allstate instead of just Vehicle state,
                therefore it is easier to specify which coordinate system to use
                '''
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
                x=refined_center[0],
                y=refined_center[1],
                z=refined_center[2],
                yaw=yaw,
                pitch=pitch,
                roll=roll,
                frame=ObjectFrameEnum.CURRENT
            )

            # Attempt to match the new detection with an existing tracked pedestrian.
            existing_id = match_existing_pedestrian(
                new_center=np.array([new_pose.x, new_pose.y, new_pose.z]),
                new_dims=dims,
                existing_agents=self.tracked_agents,
                distance_threshold=1.0
            )

            if existing_id is not None:
                # Update the state of the matched pedestrian using the computed velocity.
                old_agent_state = self.tracked_agents[existing_id]
                dt = new_pose.t - old_agent_state.pose.t
                vx, vy, vz = compute_velocity(old_agent_state.pose, new_pose, dt)

                updated_agent = AgentState(
                    pose=new_pose,
                    dimensions=dims,
                    outline=None,
                    type=AgentEnum.PEDESTRIAN,
                    activity=AgentActivityEnum.MOVING,
                    velocity=(vx, vy, vz),
                    yaw_rate=0
                )
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

        self.current_agents = agents

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
