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
import math
import ros_numpy


# ----- Helper Functions -----

def cylindrical_roi(points, center, radius, height):
    horizontal_dist = np.linalg.norm(points[:, :2] - center[:2], axis=1)
    vertical_diff = np.abs(points[:, 2] - center[2])
    mask = (horizontal_dist <= radius) & (vertical_diff <= height / 2)
    return points[mask]


def undistort_image(image, K, D):
    h, w = image.shape[:2]
    newK, _ = cv2.getOptimalNewCameraMatrix(K, D, (w, h), 1, (w, h))
    undistorted = cv2.undistort(image, K, D, None, newK)
    return undistorted, newK

def filter_points_within_threshold(points, threshold=15.0):
    distances = np.linalg.norm(points, axis=1)
    mask = distances <= threshold
    return points[mask]


def match_existing_cone(
        new_center: np.ndarray,
        new_dims: tuple,
        existing_agents: Dict[str, AgentState],
        distance_threshold: float = 1.0
) -> str:
    """
    Find the closest existing Cone agent within a specified distance threshold.
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
    Compute the (vx, vy, vz) velocity based on change in pose over time.
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
    mask = (pts[:, 0] > 0) & (pts[:, 2] < -1.5) & (pts[:, 2] > -2.7)
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


def pose_to_matrix(pose):
    """
    Compose a 4x4 transformation matrix from a pose state.
    Assumes pose has attributes: x, y, z, yaw, pitch, roll,
    where the angles are given in degrees.
    """
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


class ConeDetector3D(Component):
    """
    Detects cones by fusing YOLO 2D detections with LiDAR point cloud data.

    New approach:
      1) Downsample the LiDAR point cloud.
      2) Transform it to the camera coordinate system.
      3) Project all points onto the image plane.
      4) Filter the projected points using the YOLO 2D bounding boxes.
      5) Apply DBSCAN clustering and remove ground points.
      6) Compute an oriented bounding box and transform to the Vehicle frame.
      7) (Optional) Reproject the refined cluster onto the image.

    (Note: Do not add any additional visualization beyond what is necessary.)
    """

    def __init__(self, vehicle_interface: GEMInterface):
        self.vehicle_interface = vehicle_interface
        self.current_agents = {}
        self.tracked_agents = {}
        self.cone_counter = 0
        self.latest_image = None
        self.latest_lidar = None
        self.bridge = CvBridge()
        self.start_pose_abs = None
        self.camera_front = False
        self.visualize_2d = True
        self.use_cyl_roi = False
        self.start_time = None
        self.use_start_frame = True

    def rate(self) -> float:
        return 4.0

    def state_inputs(self) -> list:
        return ['vehicle']

    def state_outputs(self) -> list:
        return ['agents']

    def initialize(self):
        self.rgb_sub = Subscriber('/oak/rgb/image_raw', Image)
        self.lidar_sub = Subscriber('/ouster/points', PointCloud2)
        self.sync = ApproximateTimeSynchronizer([self.rgb_sub, self.lidar_sub],
                                                queue_size=10, slop=0.1)
        self.sync.registerCallback(self.synchronized_callback)
        self.detector = YOLO('../../knowledge/detection/cone.pt')
        self.detector.to('cuda')

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
            self.T_l2c = np.array([
        [ 2.89748006e-02, -9.99580136e-01,  3.68439439e-05, -3.07300513e-02],
        [-9.49930618e-03, -3.12215512e-04, -9.99954834e-01, -3.86689354e-01],
        [ 9.99534999e-01,  2.89731321e-02, -9.50437214e-03, -6.71425124e-01],
        [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.00000000e+00]
    ])
        else:
            self.T_l2c = np.array([[-0.71836368, -0.69527204, -0.02346088,  0.05718003],
     [-0.09720448,  0.13371206, -0.98624154, -0.1598301 ],
     [ 0.68884317, -0.7061996 , -0.16363744, -1.04767285],
     [ 0.        ,  0.        ,  0.        ,  1.        ]]
)
        self.T_c2l = np.linalg.inv(self.T_l2c)
        self.R_c2l = self.T_c2l[:3, :3]
        self.camera_origin_in_lidar = self.T_c2l[:3, 3]

    def synchronized_callback(self, image_msg, lidar_msg):
        step1 = time.time()
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
        except Exception as e:
            rospy.logerr("Failed to convert image: {}".format(e))
            self.latest_image = None
        step2 = time.time()
        self.latest_lidar = pc2_to_numpy(lidar_msg, want_rgb=False)
        step3 = time.time()
        print('image callback: ', step2 - step1, 'lidar callback ', step3 - step2)

    def update(self, vehicle: VehicleState) -> Dict[str, AgentState]:
        downsample = False
        if self.latest_image is None or self.latest_lidar is None:
            return {}

        current_time = self.vehicle_interface.time()
        if self.start_time is None:
            self.start_time = current_time
        time_elapsed = current_time - self.start_time

        undistorted_img, current_K = undistort_image(self.latest_image, self.K, self.D)
        self.current_K = current_K
        self.latest_image = undistorted_img
        orig_H, orig_W = undistorted_img.shape[:2]

        # --- Begin modifications for three-angle detection ---
        # Create three images: normal, left rotated (counterclockwise 90°), and right rotated (clockwise 90°)
        img_normal = undistorted_img
        img_left = cv2.rotate(undistorted_img.copy(), cv2.ROTATE_90_COUNTERCLOCKWISE)
        img_right = cv2.rotate(undistorted_img.copy(), cv2.ROTATE_90_CLOCKWISE)

        # Run YOLO on all three images.
        results_normal = self.detector(img_normal, conf=0.3, classes=[0])
        results_left = self.detector(img_left, conf=0.3, classes=[0])
        results_right = self.detector(img_right, conf=0.3, classes=[0])

        boxes_normal = np.array(results_normal[0].boxes.xywh.cpu()) if len(results_normal) > 0 else []
        boxes_left = np.array(results_left[0].boxes.xywh.cpu()) if len(results_left) > 0 else []
        boxes_right = np.array(results_right[0].boxes.xywh.cpu()) if len(results_right) > 0 else []

        # Combine boxes from all three detections.
        # For the left rotated image:
        #  In the left-rotated image, a point (cx, cy) corresponds to the original
        #  coordinates: new_cx = cy, new_cy = orig_W - 1 - cx.
        #  Also, swap the width and height.
        # For the right rotated image:
        #  A point (cx, cy) maps as: new_cx = orig_H - 1 - cy, new_cy = cx,
        #  with width and height swapped.
        combined_boxes = []
        for box in boxes_normal:
            cx, cy, w, h = box
            combined_boxes.append((cx, cy, w, h, AgentActivityEnum.STANDING))
        for box in boxes_left:
            cx, cy, w, h = box
            new_cx = cy
            new_cy = orig_W - 1 - cx
            new_w = h
            new_h = w
            combined_boxes.append((new_cx, new_cy, new_w, new_h, AgentActivityEnum.RIGHT))
        for box in boxes_right:
            cx, cy, w, h = box
            new_cx = orig_H - 1 - cy
            new_cy = cx
            new_w = h
            new_h = w
            combined_boxes.append((new_cx, new_cy, new_w, new_h, AgentActivityEnum.LEFT))
        # --- End modifications for three-angle detection ---

        # Optional 2D visualization: Draw fused 2D boxes on the undistorted image if flag is set.
        if getattr(self, 'visualize_2d', False):
            for (cx, cy, w, h, activity) in combined_boxes:
                left = int(cx - w / 2)
                right = int(cx + w / 2)
                top = int(cy - h / 2)
                bottom = int(cy + h / 2)
                if activity == AgentActivityEnum.STANDING:
                    color = (255, 0, 0)  # Blue
                    label = "STANDING"
                elif activity == AgentActivityEnum.RIGHT:
                    color = (0, 255, 0)  # Green
                    label = "RIGHT"
                elif activity == AgentActivityEnum.LEFT:
                    color = (0, 0, 255)  # Red
                    label = "LEFT"
                else:
                    color = (255, 255, 255)
                    label = "UNKNOWN"
                cv2.rectangle(undistorted_img, (left, top), (right, bottom), color, 2)
                cv2.putText(undistorted_img, label, (left, max(top - 5, 20)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
            cv2.imshow("Detection - Cone 2D", undistorted_img)

        # Remaining LiDAR processing (unchanged)
        if downsample == True:
            lidar_down = downsample_points(self.latest_lidar, voxel_size=0.1)
            pts_cam = transform_points_l2c(lidar_down, self.T_l2c)
            projected_pts = project_points(pts_cam, self.current_K, lidar_down)
        else:
            step00 = time.time()
            lidar_down = self.latest_lidar.copy()
            step01 = time.time()
            pts_cam = transform_points_l2c(lidar_down, self.T_l2c)
            step02 = time.time()
            projected_pts = project_points(pts_cam, self.current_K, lidar_down)
            step03 = time.time()
            print(
                f'copy lidar data {step01 - step00}s, transforming to camera {step02 - step01}s, projecting to image {step03 - step02}s')

        agents = {}

        # For each 2D bounding box (from the combined list), process LiDAR points.
        for i, box_info in enumerate(combined_boxes):
            cx, cy, w, h, activity = box_info
            start = time.time()
            left = int(cx - w / 2)
            right = int(cx + w / 2)
            top = int(cy - h / 2)
            bottom = int(cy + h / 2)
            mask = (projected_pts[:, 0] >= left) & (projected_pts[:, 0] <= right) & \
                   (projected_pts[:, 1] >= top) & (projected_pts[:, 1] <= bottom)
            roi_pts = projected_pts[mask]
            if roi_pts.shape[0] < 5:
                continue
            # Extract the LiDAR 3D points corresponding to the ROI.
            points_3d = roi_pts[:, 2:5]
            points_3d = remove_ground_by_min_range(points_3d, z_range=0.005)
            points_3d = filter_points_within_threshold(points_3d, 15)
            points_3d = filter_depth_points(points_3d, max_depth_diff=0.3)

            # Modification: optionally use cylindrical ROI adjustment.
            if self.use_cyl_roi:
                global_filtered = filter_points_within_threshold(lidar_down, 20)
                # Build a cylindrical ROI based on the global filtered LiDAR points
                # and the mean of the extracted points as the center.
                roi_cyl = cylindrical_roi(global_filtered, np.mean(points_3d, axis=0), radius=0.3, height=1.2)
                refined_cluster = remove_ground_by_min_range(roi_cyl, z_range=0.01)
                refined_cluster = filter_depth_points(refined_cluster, max_depth_diff=0.2)
            else:
                # Use the original method.
                refined_cluster = remove_ground_by_min_range(points_3d, z_range=0.05)
            end1 = time.time()
            print('refine cluster: ', end1 - start)
            if refined_cluster.shape[0] < 4:
                continue

            # Compute the oriented bounding box.
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(refined_cluster)
            obb = pcd.get_oriented_bounding_box()
            refined_center = obb.center
            dims = tuple(obb.extent)
            R_lidar = obb.R.copy()
            end2 = time.time()
            print('compute bounding box ', end2 - end1)
            # Transform the refined center from LiDAR to Vehicle frame.
            refined_center_hom = np.append(refined_center, 1)
            refined_center_vehicle_hom = self.T_l2v @ refined_center_hom
            refined_center_vehicle = refined_center_vehicle_hom[:3]

            R_vehicle = self.T_l2v[:3, :3] @ R_lidar
            euler_vehicle = R.from_matrix(R_vehicle).as_euler('zyx', degrees=False)
            yaw, pitch, roll = euler_vehicle
            refined_center = refined_center_vehicle

            if self.use_start_frame:
                if self.start_pose_abs is None and time_elapsed >= 5:
                    self.start_pose_abs = vehicle.pose
                vehicle_start_pose = vehicle.pose.to_frame(
                    ObjectFrameEnum.START,
                    vehicle.pose,
                    self.start_pose_abs
                )
                T_vehicle_to_start = pose_to_matrix(vehicle_start_pose)
                hom = np.append(refined_center, 1)
                xp, yp, zp = (T_vehicle_to_start @ hom)[:3]
                out_frame = ObjectFrameEnum.START
            else:
                # stay in the CURRENT vehicle frame
                xp, yp, zp = refined_center
                out_frame = ObjectFrameEnum.CURRENT

            new_pose = ObjectPose(
                t=current_time,
                x=xp, y=yp, z=zp,
                yaw=yaw, pitch=pitch, roll=roll,
                frame=out_frame
            )
            end1 = time.time()
            existing_id = match_existing_cone(
                new_center=np.array([new_pose.x, new_pose.y, new_pose.z]),
                new_dims=dims,
                existing_agents=self.tracked_agents,
                distance_threshold=2.0
            )
            if existing_id is not None:
                old_state = self.tracked_agents[existing_id]
                if vehicle.v < 0.1:
                    alpha = 0.1
                    avg_x = alpha * new_pose.x + (1 - alpha) * old_state.pose.x
                    avg_y = alpha * new_pose.y + (1 - alpha) * old_state.pose.y
                    avg_z = alpha * new_pose.z + (1 - alpha) * old_state.pose.z
                    avg_yaw = alpha * new_pose.yaw + (1 - alpha) * old_state.pose.yaw
                    avg_pitch = alpha * new_pose.pitch + (1 - alpha) * old_state.pose.pitch
                    avg_roll = alpha * new_pose.roll + (1 - alpha) * old_state.pose.roll

                    updated_pose = ObjectPose(
                        t=new_pose.t,
                        x=avg_x,
                        y=avg_y,
                        z=avg_z,
                        yaw=avg_yaw,
                        pitch=avg_pitch,
                        roll=avg_roll,
                        frame=new_pose.frame
                    )
                    updated_agent = AgentState(
                        pose=updated_pose,
                        dimensions=dims,
                        outline=None,
                        type=AgentEnum.CONE,
                        activity=activity,
                        velocity=(0, 0, 0),
                        yaw_rate=0
                    )
                else:
                    updated_agent = old_state
                agents[existing_id] = updated_agent
                self.tracked_agents[existing_id] = updated_agent
            else:
                agent_id = f"Cone{self.cone_counter}"
                self.cone_counter += 1
                new_agent = AgentState(
                    pose=new_pose,
                    dimensions=dims,
                    outline=None,
                    type=AgentEnum.CONE,
                    activity=activity,
                    velocity=(0, 0, 0),
                    yaw_rate=0
                )
                agents[agent_id] = new_agent
                self.tracked_agents[agent_id] = new_agent
            end2 = time.time()
            print(f'Tracking time:{end2 - end1}')

        self.current_agents = agents

        stale_ids = [agent_id for agent_id, agent in self.tracked_agents.items()
                     if current_time - agent.pose.t > 5.0]
        for agent_id in stale_ids:
            rospy.loginfo(f"Removing stale agent: {agent_id}\n")
        for agent_id, agent in self.tracked_agents.items():
            p = agent.pose
            rospy.loginfo(
                f"Agent ID: {agent_id}\n"
                f"Pose: (x: {p.x:.3f}, y: {p.y:.3f}, z: {p.z:.3f}, "
                f"yaw: {p.yaw:.3f}, pitch: {p.pitch:.3f}, roll: {p.roll:.3f})\n"
                f"Velocity: (vx: {agent.velocity[0]:.3f}, vy: {agent.velocity[1]:.3f}, vz: {agent.velocity[2]:.3f})\n"
            )
        return self.tracked_agents

    # ----- Fake Cone Detector 2D (for Testing Purposes) -----

class FakConeDetector(Component):
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
                res['cone0'] = box_to_fake_agent((0, 0, 0, 0))
                rospy.loginfo("Detected a Cone (simulated)")
        return res

def box_to_fake_agent(box):
    x, y, w, h = box
    pose = ObjectPose(t=0, x=x + w / 2, y=y + h / 2, z=0, yaw=0, pitch=0, roll=0, frame=ObjectFrameEnum.CURRENT)
    dims = (w, h, 0)
    return AgentState(pose=pose, dimensions=dims, outline=None,
                      type=AgentEnum.CONE, activity=AgentActivityEnum.MOVING,
                      velocity=(0, 0, 0), yaw_rate=0)

if __name__ == '__main__':
    pass
