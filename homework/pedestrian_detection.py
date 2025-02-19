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


def remove_ground_by_min_range(cluster, z_range=0.05):
    """
    Remove ground points by finding the minimum z value in the cluster and eliminating
    all points with z within z_range of that minimum.
    """
    if cluster is None or cluster.shape[0] == 0:
        return cluster
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

        # self.lidar_pc = pc2_to_numpy(self.pc_raw, want_rgb=False)
        agents = {}
        if self.lidar_pc is None:
            print("NOT fOUND")
            return agents
        for i, box in enumerate(self.last_person_boxes):
            cx, cy, w, h = box
            # Backproject the center pixel into a 3D ray.
            ray_dir_cam = backproject_pixel(cx, cy, self.K)
            ray_dir_lidar = self.R_c2l @ ray_dir_cam
            ray_dir_lidar /= np.linalg.norm(ray_dir_lidar)
            # Find a candidate 3D point along the ray.
            intersection, _, _ = find_human_center_on_ray(self.lidar_pc, self.camera_origin_in_lidar, ray_dir_lidar,
                                                          t_min=0.5, t_max=20.0, t_step=0.1,
                                                          distance_threshold=0.2, min_points=20, ransac_threshold=0.05)
            if intersection is None:
                continue
            # Use the 2D bounding box dimensions to guide ROI extraction.
            d = np.linalg.norm(intersection - self.camera_origin_in_lidar)
            physical_width = (w * d) / self.K[0, 0]
            physical_height = (h * d) / self.K[1, 1]
            # Here, we use a 3D sphere ROI as a simple approach; you can replace it with a box ROI if needed.
            depth_margin = physical_width  # Alternatively, you can set a constant like 0.5
            half_extents = np.array([
                1.1 * physical_width / 2,
                1.1 * depth_margin / 2,
                1.25 * physical_height / 2
            ])

            # Extract ROI using a 3D box that matches the 2D bounding box.
            roi_points = extract_roi_box(self.lidar_pc, intersection, half_extents)
            if roi_points.shape[0] < 10:
                refined_cluster = roi_points
            else:
                refined_cluster = refine_cluster(roi_points, intersection, eps=0.125, min_samples=10)
            #print(roi_points)
            # Remove ground points by eliminating those within a small z-range of the minimum.
            refined_cluster = remove_ground_by_min_range(refined_cluster, z_range=0.03)
            #print(refined_cluster)
            if refined_cluster is None or refined_cluster.shape[0] == 0:
                refined_center = intersection
                dims = (0, 0, 0)
                yaw, pitch, roll = 0, 0, 0
            else:
                # Compute oriented bounding box for the refined cluster.
                pcd = o3d.geometry.PointCloud()
                pcd.points = o3d.utility.Vector3dVector(refined_cluster)
                obb = pcd.get_oriented_bounding_box()
                refined_center = obb.center
                dims = tuple(obb.extent)

                # Convert rotation matrix to Euler angles (yaw, pitch, roll).
                euler_angles = R.from_matrix(obb.R.copy()).as_euler('zyx', degrees=True)
                yaw, pitch, roll = euler_angles[0], euler_angles[1], euler_angles[2]
                print(f"Detected human - Pose (yaw, pitch, roll): {euler_angles}")
                print(f"Bounding box center: {refined_center}, Dimensions: {dims}")
            # Create agent pose.
            pose = ObjectPose(t=0, x=refined_center[0], y=refined_center[1],
                              z=refined_center[2], yaw=yaw, pitch=pitch, roll=roll,
                              frame=ObjectFrameEnum.CURRENT)
            agent_state = AgentState(pose=pose, dimensions=dims, outline=None,
                                     type=AgentEnum.PEDESTRIAN, activity=AgentActivityEnum.MOVING,
                                     velocity=(0, 0, 0), yaw_rate=0)
            agents[f'pedestrian{i}'] = agent_state
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
