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
    Return the agent_id of the best match if within distance_threshold;
    otherwise return None.
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
    """Convert ROS PointCloud2 message to a numpy array, filtering points with x > 0 and z < 2.5."""
    gen = pc2.read_points(pc2_msg, skip_nans=True)
    pts = np.array(list(gen), dtype=np.float32)
    pts = pts[:, :3]  # Use only x, y, z
    mask = (pts[:, 0] > 0) & (pts[:, 2] < 2.5)
    return pts[mask]

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
    Returns (refined_candidate, None, None) if found; otherwise, (None, None, None).
    """
    vecs = lidar_pc - ray_origin
    proj_lengths = np.dot(vecs, ray_direction)
    proj_points = ray_origin + np.outer(proj_lengths, ray_direction)
    dists_to_ray = np.linalg.norm(lidar_pc - proj_points, axis=1)
    near_ray_mask = dists_to_ray < distance_threshold
    filtered_pc = lidar_pc[near_ray_mask]

    if filtered_pc.shape[0] < min_points:
        return None, None, None

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

# ----- Pedestrian Detector 2D (with 3D fusion and synchronized callbacks) -----
class PedestrianDetector2D(Component):
    """
    Detects pedestrians using YOLO and LiDAR to estimate 3D pose.
    This version uses message_filters to synchronize the image and LiDAR data.
    The synchronized callback stores the latest sensor data, and heavy processing is done in update().
    """

    def __init__(self, vehicle_interface: GEMInterface):
        self.vehicle_interface = vehicle_interface
        self.current_agents = {}
        self.tracked_agents = {}
        self.pedestrian_counter = 0
        # Variables to store synchronized sensor data:
        self.latest_image = None
        self.latest_lidar = None
        self.bridge = CvBridge()

    def rate(self) -> float:
        return 4.0

    def state_inputs(self) -> list:
        return ['vehicle']

    def state_outputs(self) -> list:
        return ['agents']

    def initialize(self):
        # Instead of individual subscriptions, use message_filters to synchronize
        self.rgb_sub = Subscriber('/oak/rgb/image_raw', Image)
        self.lidar_sub = Subscriber('/ouster/points', PointCloud2)
        self.sync = ApproximateTimeSynchronizer([self.rgb_sub, self.lidar_sub],
                                                queue_size=10, slop=0.1)
        self.sync.registerCallback(self.synchronized_callback)
        # Initialize YOLO detector
        self.detector = YOLO('../../knowledge/detection/yolov8n.pt')
        # Set up camera intrinsics and LiDAR-to-camera transformation.
        self.T_l2v = np.array([[ 0.99939639,  0.02547917,  0.023615,    1.1       ],
                                [-0.02530848,  0.99965156, -0.00749882,  0.03773583],
                                [-0.02379784,  0.00689664,  0.999693,    1.95320223],
                                [ 0.,          0.,          0.,          1.        ]])
        self.K = np.array([[684.83331299, 0., 573.37109375],
                           [0., 684.60968018, 363.70092773],
                           [0., 0., 1.]])
        self.T_l2c = np.array([
            [0.001090, -0.999489, -0.031941,  0.149698],
            [-0.007664,  0.031932, -0.999461, -0.397813],
            [0.999970,  0.001334, -0.007625, -0.691405],
            [0.000000,  0.000000,  0.000000,  1.000000]
        ])
        self.T_c2l = np.linalg.inv(self.T_l2c)
        self.R_c2l = self.T_c2l[:3, :3]
        self.camera_origin_in_lidar = self.T_c2l[:3, 3]
        

    def synchronized_callback(self, image_msg, lidar_msg):
        """
        This callback is triggered when both an image and a LiDAR message arrive within the slop.
        It stores the latest synchronized sensor data for processing in update().
        """
        # Convert the image message to an OpenCV image (assuming it is already in cv2.Mat format or convert as needed)
            # Convert the ROS Image message to an OpenCV image (BGR format)
        try:
            # Convert the ROS Image message to an OpenCV image (BGR format)
            self.latest_image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
        except Exception as e:
            rospy.logerr("Failed to convert image: {}".format(e))
            self.latest_image = None
        # Convert the LiDAR message to a numpy array
        self.latest_lidar = pc2_to_numpy(lidar_msg, want_rgb=False)

    def update(self, vehicle: VehicleState) -> Dict[str, AgentState]:
        # Process only if synchronized sensor data is available
        if self.latest_image is None or self.latest_lidar is None:
            rospy.logwarn("Synchronized sensor data not available; skipping update.")
            return {}

        current_time = self.vehicle_interface.time()
        # Run YOLO inference on the latest synchronized image
        results = self.detector(self.latest_image, conf=0.4, classes=[0])
        boxes = np.array(results[0].boxes.xywh.cpu())  # Format: [center_x, center_y, w, h]

        agents = {}
        lidar_pc = self.latest_lidar.copy()

        for i, box in enumerate(boxes):
            cx, cy, w, h = box
            # Convert pixel center to a ray in LiDAR frame
            ray_dir_cam = backproject_pixel(cx, cy, self.K)
            ray_dir_lidar = self.R_c2l @ ray_dir_cam
            ray_dir_lidar /= np.linalg.norm(ray_dir_lidar)

            intersection, _, _ = find_human_center_on_ray(
                lidar_pc, self.camera_origin_in_lidar, ray_dir_lidar,
                t_min=0.4, t_max=20.0, t_step=0.1,
                distance_threshold=0.3, min_points=5, ransac_threshold=0.05
            )
            if intersection is None:
                continue

            d = np.linalg.norm(intersection - self.camera_origin_in_lidar)
            physical_width = (w * d) / self.K[0, 0]
            physical_height = (h * d) / self.K[1, 1]
            depth_margin = physical_width
            half_extents = np.array([0.4, 0.4, 1.25 * physical_height / 2])

            roi_points = extract_roi_box(lidar_pc, intersection, half_extents)
            if roi_points.shape[0] < 10:
                refined_cluster = roi_points
            else:
                refined_cluster = refine_cluster(roi_points, intersection, eps=0.15, min_samples=10)

            refined_cluster = remove_ground_by_min_range(refined_cluster, z_range=0.03)
            if refined_cluster is None or refined_cluster.shape[0] == 0:
                refined_center = intersection
                dims = (0, 0, 0)
                yaw, pitch, roll = 0, 0, 0
            elif refined_cluster.shape[0] <= 5:
                continue
            else:
                pcd = o3d.geometry.PointCloud()
                pcd.points = o3d.utility.Vector3dVector(refined_cluster)
                obb = pcd.get_oriented_bounding_box()
                refined_center = obb.center
                dims = tuple(obb.extent)
                R_lidar = obb.R.copy()

                # Transform refined center to vehicle frame
                refined_center_lidar_hom = np.array([refined_center[0],
                                                     refined_center[1],
                                                     refined_center[2],
                                                     1.0])
                refined_center_vehicle_hom = self.T_l2v @ refined_center_lidar_hom
                refined_center_vehicle = refined_center_vehicle_hom[:3]

                R_vehicle = self.T_l2v[:3, :3] @ R_lidar
                euler_angles_vehicle = R.from_matrix(R_vehicle).as_euler('zyx', degrees=False)
                yaw, pitch, roll = euler_angles_vehicle
                refined_center = refined_center_vehicle  # Use vehicle frame for output
                curr_x = vehicle.pose.x
                curr_y = vehicle.pose.y
                curr_yaw = vehicle.pose.yaw
                curr_pitch = vehicle.pose.pitch
                curr_roll = vehicle.pose.roll
                refined_center[0] += curr_x
                refined_center[1] += curr_y
                refined_center[2] += curr_yaw
                refined_center[3] += curr_pitch
                refined_center[4] += curr_roll
                rospy.loginfo(f"Detected human in vehicle frame - Pose: {euler_angles_vehicle}, "
                              f"Center: {refined_center_vehicle}, Dimensions: {dims}")


            # Create new pose in the vehicle frame
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

            # Attempt to match with an existing pedestrian
            existing_id = match_existing_pedestrian(
                new_center=np.array([new_pose.x, new_pose.y, new_pose.z]),
                new_dims=dims,
                existing_agents=self.tracked_agents,
                distance_threshold=1.0
            )

            if existing_id is not None:
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

        # Remove stale agents that haven't been updated for more than 3 seconds.
        stale_ids = [agent_id for agent_id, agent in self.tracked_agents.items()
                     if current_time - agent.pose.t > 3.0]
        for agent_id in stale_ids:
            rospy.loginfo(f"Removing stale agent: {agent_id}")
            del self.tracked_agents[agent_id]

        self.current_agents = agents
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
                rospy.loginfo("Detected a pedestrian")
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
