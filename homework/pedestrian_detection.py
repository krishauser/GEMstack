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
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import struct, ctypes



# ----- Helper Functions -----
def match_existing_pedestrian(
    new_center: np.ndarray,
    new_dims: tuple,
    existing_agents: Dict[str, AgentState],
    prev_velocities: Dict[str, np.ndarray],
    distance_threshold: float = 1.0,
    size_threshold: float = 0.5,
    height_threshold: float = 0.3,
    velocity_threshold: float = 2.0
) -> str:
    """
    Match a newly detected pedestrian with an existing one using:
    - Euclidean distance
    - Bounding box similarity
    - Height consistency
    - Velocity consistency

    Returns the best-matching agent_id or None if no good match is found.
    """
    best_agent_id = None
    best_score = float('inf')

    for agent_id, agent_state in existing_agents.items():
        old_center = np.array([agent_state.pose.x, agent_state.pose.y, agent_state.pose.z])
        old_dims = agent_state.dimensions

        # 1. Euclidean Distance Check
        dist = np.linalg.norm(new_center - old_center)
        if dist > distance_threshold:
            continue  # Skip if too far away

        # 2. Bounding Box Size Similarity (with Zero-Division Handling)
        size_norm = np.linalg.norm(np.array(old_dims))
        if size_norm > 0:
            size_change = np.linalg.norm(np.array(new_dims) - np.array(old_dims)) / size_norm
        else:
            size_change = float('inf')  # Prevent invalid matching

        if size_change > size_threshold:
            continue  # Skip if bounding box changes too much

        # 3. Height Consistency Check
        height_change = abs(new_dims[2] - old_dims[2]) / old_dims[2] if old_dims[2] > 0 else 0
        if height_change > height_threshold:
            continue  # Skip if height changes drastically

        # 4. Velocity Consistency Check
        if agent_id in prev_velocities:
            prev_velocity = prev_velocities[agent_id]
            estimated_velocity = (new_center - old_center)
            velocity_change = np.linalg.norm(estimated_velocity - prev_velocity)

            if velocity_change > velocity_threshold:
                continue  # Skip if unrealistic velocity jump

        # Score: Lower score = better match (distance is primary factor)
        score = dist
        if score < best_score:
            best_score = score
            best_agent_id = agent_id

    return best_agent_id




def compute_velocity(old_pose: ObjectPose, new_pose: ObjectPose, dt: float) -> tuple:
    """
    Returns a (vx, vy, vz) velocity in the same frame as old_pose/new_pose.
    """
    if dt <= 0:
        return (0, 0, 0)
    vx = (new_pose.x - old_pose.x) / dt
    vy = (new_pose.y - old_pose.y) / dt
    vz = (new_pose.z - old_pose.z) / dt
    return (vx, vy, vz)

def extract_roi_box(lidar_pc, center, half_extents):
    lower = center - half_extents
    upper = center + half_extents
    mask = np.all((lidar_pc >= lower) & (lidar_pc <= upper), axis=1)
    return lidar_pc[mask]


def pc2_to_numpy(pc2_msg, want_rgb=False):
    """Convert ROS PointCloud2 message to a numpy array."""
    gen = pc2.read_points(pc2_msg, skip_nans=True)
    if want_rgb:
        # Implement RGB extraction if needed
        xyzpack = np.array(list(gen), dtype=np.float32)
        if xyzpack.shape[1] != 4:
            raise ValueError("PointCloud2 does not have points")
        # Additional processing for RGB if required
    else:
        return np.array(list(gen), dtype=np.float32)[:, :3]


def backproject_pixel(u, v, K):
    """Backprojects pixel (u,v) into a normalized 3D ray (camera coordinates)."""
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
    Pre-filter the point cloud to only include points near the ray, then sweep along the ray.
    For each candidate along the ray, compute the centroid of nearby points and return that as the refined candidate.
    Returns (refined_candidate, None, None) if found; otherwise, (None, None, None).
    """
    # Pre-filter: compute distance from each point to the ray.
    vecs = lidar_pc - ray_origin  # Vectors from origin to points.
    proj_lengths = np.dot(vecs, ray_direction)  # Projection lengths.
    proj_points = ray_origin + np.outer(proj_lengths, ray_direction)
    dists_to_ray = np.linalg.norm(lidar_pc - proj_points, axis=1)
    near_ray_mask = dists_to_ray < distance_threshold
    filtered_pc = lidar_pc[near_ray_mask]

    # If too few points remain, return None.
    if filtered_pc.shape[0] < min_points:
        return None, None, None

    # Sweep along the ray using the filtered point cloud.
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
    """Extract points from pc that lie within roi_radius of center."""
    distances = np.linalg.norm(pc - center, axis=1)
    return pc[distances < roi_radius]


def refine_cluster(roi_points, center, eps=0.2, min_samples=10):
    """Refine a cluster using DBSCAN and return the cluster closest to center."""
    clustering = DBSCAN(eps=eps, min_samples=min_samples).fit(roi_points)
    labels = clustering.labels_
    valid_clusters = [roi_points[labels == l] for l in set(labels) if l != -1]
    if not valid_clusters:
        return roi_points
    best_cluster = min(valid_clusters, key=lambda c: np.linalg.norm(np.mean(c, axis=0) - center))
    return best_cluster


def fit_plane_ransac(points, threshold, min_inliers, iterations=100):
    """
    A more efficient RANSAC plane fitting method.
    - Skips iterations with nearly collinear points.
    - Prioritizes diverse point selection to improve stability.
    """
    best_inliers_count = 0
    best_plane = None
    best_inliers = None
    n_points = points.shape[0]

    if n_points < 3:
        return None, None

    for _ in range(iterations):
        idx = np.random.choice(n_points, 3, replace=False)
        sample = points[idx]
        p1, p2, p3 = sample

        # Ensure the points are sufficiently spread out (avoid collinearity)
        if np.linalg.norm(p1 - p2) < 0.1 or np.linalg.norm(p2 - p3) < 0.1:
            continue

        normal = np.cross(p2 - p1, p3 - p1)
        norm = np.linalg.norm(normal)

        if norm == 0:
            continue  # Skip degenerate cases

        normal = normal / norm
        d = -np.dot(normal, p1)
        plane = np.hstack([normal, d])

        distances = np.abs(points.dot(normal) + d)
        inliers = np.where(distances < threshold)[0]

        if len(inliers) > best_inliers_count:
            best_inliers_count = len(inliers)
            best_plane = plane
            best_inliers = inliers

    if best_inliers_count >= min_inliers:
        return best_plane, best_inliers
    else:
        return None, None
    

def remove_ground(cluster, z_range=0.05, ransac_threshold=0.05, min_inliers=20):
    """
    Improved ground removal using RANSAC plane fitting.
    - First, attempts to remove the dominant ground plane using RANSAC.
    - If no valid plane is found, falls back to simple min-Z filtering.

    Parameters:
    - cluster (np.ndarray): (N,3) point cloud cluster.
    - z_range (float): Threshold for min-Z based ground removal (fallback).
    - ransac_threshold (float): RANSAC distance threshold for ground plane fitting.
    - min_inliers (int): Minimum number of inliers to accept a ground plane.

    Returns:
    - filtered (np.ndarray): The cluster with ground points removed.
    """
    if cluster is None or cluster.shape[0] == 0:
        return cluster

    # Attempt RANSAC-based plane removal
    plane, inliers = fit_plane_ransac(cluster, threshold=ransac_threshold, min_inliers=min_inliers, iterations=100)
    
    if plane is not None and inliers is not None and len(inliers) > 0:
        # Remove inlier points belonging to the ground plane
        filtered = np.delete(cluster, inliers, axis=0)
    else:
        # Fallback to simple min-Z removal
        min_z = np.min(cluster[:, 2])
        filtered = cluster[cluster[:, 2] > (min_z + z_range)]

    return filtered


def get_bounding_box_center_and_dimensions(points):
    """
    Compute the bounding box center and dimensions (max - min) for the given points.
    """
    if points.shape[0] == 0:
        return None, None
    min_vals = np.min(points, axis=0)
    max_vals = np.max(points, axis=0)
    center = (min_vals + max_vals) / 2
    dimensions = max_vals - min_vals
    return center, dimensions


def create_circle_line_set(center, radius, num_points=50):
    """
    Create a LineSet representing a circle (in the X-Y plane) with given center and radius.
    """
    theta = np.linspace(0, 2 * np.pi, num_points, endpoint=False)
    circle_points = []
    for angle in theta:
        x = center[0] + radius * np.cos(angle)
        y = center[1] + radius * np.sin(angle)
        z = center[2]
        circle_points.append([x, y, z])
    circle_points = np.array(circle_points)
    lines = [[i, (i + 1) % num_points] for i in range(num_points)]
    line_set = o3d.geometry.LineSet()
    line_set.points = o3d.utility.Vector3dVector(circle_points)
    line_set.lines = o3d.utility.Vector2iVector(lines)
    line_set.colors = o3d.utility.Vector3dVector([[0, 1, 0] for _ in range(len(lines))])
    return line_set


def create_ray_line_set(start, end):
    """
    Create a LineSet representing a ray from 'start' to 'end' (colored yellow).
    """
    points = [start, end]
    lines = [[0, 1]]
    line_set = o3d.geometry.LineSet()
    line_set.points = o3d.utility.Vector3dVector(points)
    line_set.lines = o3d.utility.Vector2iVector(lines)
    line_set.colors = o3d.utility.Vector3dVector([[1, 1, 0]])
    return line_set


def visualize_geometries(geometries, window_name="Open3D", width=800, height=600, point_size=5.0):
    """Utility to visualize a list of Open3D geometries."""
    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name=window_name, width=width, height=height)
    for geom in geometries:
        vis.add_geometry(geom)
    opt = vis.get_render_option()
    opt.point_size = point_size
    vis.run()
    vis.destroy_window()


# ----- Pedestrian Detector 2D (with 3D fusion) -----

class PedestrianDetector2D(Component):
    """Detects pedestrians using YOLO and LiDAR to estimate 3D pose."""

    def __init__(self, vehicle_interface: GEMInterface):
        self.vehicle_interface = vehicle_interface
        self.last_person_boxes = []
        self.lidar_pc = None  # Will be updated via ROS callback
        self.pc_raw = None
        self.tracked_agents = {}
        self.pedestrian_counter = 0


    def rate(self):
        return 4.0

    def state_inputs(self):
        return ['vehicle']

    def state_outputs(self):
        return ['agents']

    def initialize(self):
        # Subscribe to camera and LiDAR.
        self.detector = YOLO('../../knowledge/detection/yolov8n.pt')
        self.vehicle_interface.subscribe_sensor('front_camera', self.image_callback, cv2.Mat)
        self.vehicle_interface.subscribe_sensor('top_lidar', self.lidar_callback, PointCloud2)
        #self.vehicle_interface.subscribe_sensor('ouster/points', self.lidar_callback, PointCloud2)
        # Set up camera intrinsics and LiDAR-to-camera transformation.
        self.T_l2v = np.array([
            [0.99993639, 0.02547917, 0.023615, -1.1],
            [-0.02530848, 0.9996156, -0.00749882, 0.03773583],
            [-0.02379784, 0.00689664, 0.999693, 1.95320223],
            [0., 0., 0., 1.]
        ])
        self.K = np.array([[684.83331299, 0., 573.37109375],
                           [0., 684.60968018, 363.70092773],
                           [0., 0., 1.]])
        self.T_l2c = np.array([[-0.01909581, -0.9997844, 0.0081547, 0.24521313],
                               [0.06526397, -0.00938524, -0.9978239, -0.80389025],
                               [0.9976853, -0.01852205, 0.06542912, -0.6605772],
                               [0., 0., 0., 1.]])
        self.T_c2l = np.linalg.inv(self.T_l2c)
        self.R_c2l = self.T_c2l[:3, :3]
        self.camera_origin_in_lidar = self.T_c2l[:3, 3]

    def lidar_callback(self, lidar_msg: PointCloud2):
        """Convert ROS PointCloud2 to numpy array and store it."""
        self.lidar_pc = pc2_to_numpy(lidar_msg, want_rgb=False)
    def image_callback(self, image: cv2.Mat):
        results = self.detector(image, conf=0.5, classes=[0])
        boxes = np.array(results[0].boxes.xywh.cpu())  # Format: [center_x, center_y, w, h]
        self.last_person_boxes = boxes

    def update(self, vehicle: VehicleState) -> Dict[str, AgentState]:
        agents = {}
        if self.lidar_pc is None:
            print("NOT FOUND")
            return agents

        current_time = self.vehicle_interface.time()

        for i, box in enumerate(self.last_person_boxes):
            cx, cy, w, h = box
            ray_dir_cam = backproject_pixel(cx, cy, self.K)
            ray_dir_lidar = self.R_c2l @ ray_dir_cam
            ray_dir_lidar /= np.linalg.norm(ray_dir_lidar)

            intersection, _, _ = find_human_center_on_ray(
                self.lidar_pc, self.camera_origin_in_lidar, ray_dir_lidar,
                t_min=0.5, t_max=20.0, t_step=0.1,
                distance_threshold=0.2, min_points=20, ransac_threshold=0.05
            )
            if intersection is None:
                continue

            d = np.linalg.norm(intersection - self.camera_origin_in_lidar)
            physical_width = (w * d) / self.K[0, 0]
            physical_height = (h * d) / self.K[1, 1]
            depth_margin = physical_width
            half_extents = np.array([
                1.1 * physical_width / 2,
                1.1 * depth_margin / 2,
                1.25 * physical_height / 2
            ])

            roi_points = extract_roi_box(self.lidar_pc, intersection, half_extents)
            if roi_points.shape[0] < 10:
                refined_cluster = roi_points
            else:
                refined_cluster = refine_cluster(roi_points, intersection, eps=0.125, min_samples=10)

            refined_cluster = remove_ground(refined_cluster, z_range=0.03)

            # ✅ Ensure existing_agents only contains valid AgentState objects
            existing_agents = {k: v[0] for k, v in self.tracked_agents.items() if v}
            
            # ✅ Ensure prev_velocities are properly formatted
            prev_velocities = {
                k: np.array(v[0].velocity) if v[0].velocity is not None else np.array([0, 0, 0])
                for k, v in self.tracked_agents.items()
            }

            # ✅ Match before using `existing_id`
            existing_id = match_existing_pedestrian(
                new_center=np.array(intersection),
                new_dims=(physical_width, depth_margin, physical_height),
                existing_agents=existing_agents,
                prev_velocities=prev_velocities,
                distance_threshold=1.0,
                size_threshold=0.5,
                height_threshold=0.3,
                velocity_threshold=2.0
            )

            if refined_cluster is None or refined_cluster.shape[0] == 0:
                refined_center = intersection
                # ✅ Fix: Ensure `existing_id` exists in `existing_agents` before accessing `.dimensions`
                if existing_id is not None and existing_id in existing_agents:
                    dims = existing_agents[existing_id].dimensions
                else:
                    dims = (0.5, 0.5, 1.7)  # Default pedestrian size
                yaw, pitch, roll = 0, 0, 0
            else:
                pcd = o3d.geometry.PointCloud()
                pcd.points = o3d.utility.Vector3dVector(refined_cluster)
                obb = pcd.get_oriented_bounding_box()
                refined_center = obb.center
                dims = tuple(obb.extent)
                R_lidar = obb.R.copy()

                refined_center_vehicle_hom = self.T_l2v @ np.array([*refined_center, 1.0])
                refined_center_vehicle = refined_center_vehicle_hom[:3]

                R_vehicle = self.T_l2v[:3, :3] @ R_lidar
                euler_angles_vehicle = R.from_matrix(R_vehicle).as_euler('zyx', degrees=True)
                yaw, pitch, roll = euler_angles_vehicle

                refined_center = refined_center_vehicle  # Override to use vehicle frame

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

            if existing_id is not None and existing_id in self.tracked_agents:
                old_agent_state, old_time = self.tracked_agents[existing_id]
                dt = current_time - old_time
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
                self.tracked_agents[existing_id] = (updated_agent, current_time)

            else:
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
                self.tracked_agents[agent_id] = (new_agent, current_time)

        return agents





# ----- Fake Pedestrian Detector 2D (unchanged) -----

class FakePedestrianDetector2D(Component):
    """Triggers a pedestrian detection at some random time ranges."""

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
        for times in self.times:
            if t >= times[0] and t <= times[1]:
                res['pedestrian0'] = box_to_fake_agent((0, 0, 0, 0))
                print("Detected a pedestrian")
        return res


def box_to_fake_agent(box):
    """Creates a fake agent state from an (x,y,w,h) bounding box.

    The location and size are just 2D approximations.
    """
    x, y, w, h = box
    pose = ObjectPose(t=0, x=x + w / 2, y=y + h / 2, z=0, yaw=0, pitch=0, roll=0, frame=ObjectFrameEnum.CURRENT)
    dims = (w, h, 0)
    return AgentState(pose=pose, dimensions=dims, outline=None,
                      type=AgentEnum.PEDESTRIAN, activity=AgentActivityEnum.MOVING,
                      velocity=(0, 0, 0), yaw_rate=0)


if __name__ == '__main__':
    # This module is meant to be used within the vehicle interface context.
    # For testing standalone, you may create a fake vehicle state and call update().
    pass
