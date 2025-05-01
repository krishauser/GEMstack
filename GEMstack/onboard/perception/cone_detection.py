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
import os
import yaml


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

    Tracking is optional: set `enable_tracking=False` to disable persistent tracking
    and return only detections from the current frame.

    Supports multiple cameras; each camera’s intrinsics and extrinsics are
    loaded from a single YAML calibration file via plain PyYAML.
    """

    def __init__(
        self,
        vehicle_interface: GEMInterface,
        camera_name: str,
        camera_calib_file: str,
        enable_tracking: bool = True,
        visualize_2d: bool = False,
        use_cyl_roi: bool = False,
        T_l2v: list = None,
        save_data: bool = True,
        orientation: bool = True,
        use_start_frame: bool = True,
        **kwargs
    ):
        # Core interfaces and state
        self.vehicle_interface   = vehicle_interface
        self.current_agents      = {}
        self.tracked_agents      = {}
        self.cone_counter        = 0
        self.latest_image        = None
        self.latest_lidar        = None
        self.bridge              = CvBridge()
        self.start_pose_abs      = None
        self.start_time          = None

        # Config flags
        self.camera_name     = camera_name
        self.enable_tracking = enable_tracking
        self.visualize_2d    = visualize_2d
        self.use_cyl_roi     = use_cyl_roi
        self.save_data       = save_data
        self.orientation     = orientation
        self.use_start_frame = use_start_frame

        # 1) Load lidar→vehicle transform
        if T_l2v is not None:
            self.T_l2v = np.array(T_l2v)
        else:
            self.T_l2v = np.array([
                [0.99939639,  0.02547917,  0.023615,    1.1],
                [-0.02530848, 0.99965156, -0.00749882,  0.03773583],
                [-0.02379784, 0.00689664,  0.999693,     1.95320223],
                [0.0,         0.0,         0.0,          1.0]
            ])

        # 2) Load camera intrinsics/extrinsics from the supplied YAML
        with open(camera_calib_file, 'r') as f:
            calib = yaml.safe_load(f)

        # Expect structure:
        # cameras:
        #   front:
        #     K:   [[...], [...], [...]]
        #     D:   [...]
        #     T_l2c: [[...], ..., [...]]
        cam_cfg = calib['cameras'][camera_name]
        self.K     = np.array(cam_cfg['K'])
        self.D     = np.array(cam_cfg['D'])
        self.T_l2c = np.array(cam_cfg['T_l2c'])

        # Derived transforms

        self.undistort_map1 = None
        self.undistort_map2 = None
        self.camera_front = (camera_name=='front')

    def rate(self) -> float:
        return 8

    def state_inputs(self) -> list:
        return ['vehicle']

    def state_outputs(self) -> list:
        return ['agents']

    def initialize(self):
        # --- Determine the correct RGB topic for this camera ---
        rgb_topic_map = {
            'front': '/oak/rgb/image_raw',
            'front_right': '/camera_fr/arena_camera_node/image_raw',
            # add additional camera mappings here if needed
        }
        rgb_topic = rgb_topic_map.get(
            self.camera_name,
            f'/{self.camera_name}/rgb/image_raw'
        )

        # Subscribe to the RGB and LiDAR streams
        self.rgb_sub = Subscriber(rgb_topic, Image)
        self.lidar_sub = Subscriber('/ouster/points', PointCloud2)
        self.sync = ApproximateTimeSynchronizer([
            self.rgb_sub, self.lidar_sub
        ], queue_size=200, slop=0.1)
        self.sync.registerCallback(self.synchronized_callback)

        # Initialize the YOLO detector
        self.detector = YOLO('GEMstack/knowledge/detection/cone.pt')
        self.detector.to('cuda')
        self.T_c2l                = np.linalg.inv(self.T_l2c)
        self.R_c2l                = self.T_c2l[:3, :3]
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
        # print('image callback: ', step2 - step1, 'lidar callback ', step3 - step2)

    def undistort_image(self, image, K, D):
        h, w = image.shape[:2]
        newK, _ = cv2.getOptimalNewCameraMatrix(K, D, (w, h), 1, (w, h))
        if self.undistort_map1 is None or self.undistort_map2 is None:
            self.undistort_map1, self.undistort_map2 = cv2.initUndistortRectifyMap(K, D, R=None,
                                                                                   newCameraMatrix=newK, size=(w, h),
                                                                                   m1type=cv2.CV_32FC1)

        start = time.time()
        undistorted = cv2.remap(image, self.undistort_map1, self.undistort_map2, interpolation=cv2.INTER_NEAREST)
        end = time.time()
        # print('--------undistort', end-start)
        return undistorted, newK

    def update(self, vehicle: VehicleState) -> Dict[str, AgentState]:

        start = time.time()
        downsample = False
        if self.latest_image is None or self.latest_lidar is None:
            return {}
        lastest_image = self.latest_image.copy()
        # Ensure data/ exists and build timestamp
        if downsample:
            lidar_down = downsample_points(self.latest_lidar, voxel_size=0.1)
        else:
            lidar_down = self.latest_lidar.copy()
        current_time = self.vehicle_interface.time()
        if self.start_time is None:
            self.start_time = current_time
        time_elapsed = current_time - self.start_time

        if self.save_data:
            os.makedirs("data", exist_ok=True)
            tstamp = int(self.vehicle_interface.time() * 1000)
            # 1) Dump raw image
            cv2.imwrite(f"data/{tstamp}_image.png", lastest_image)
            # 2) Dump raw LiDAR
            np.savez(f"data/{tstamp}_lidar.npz", lidar=self.latest_lidar)
            # 3) Write BEFORE_TRANSFORM
            with open(f"data/{tstamp}_vehstate.txt", "w") as f:
                vp = vehicle.pose
                f.write(
                    f"BEFORE_TRANSFORM "
                    f"x={vp.x:.3f}, y={vp.y:.3f}, z={vp.z:.3f}, "
                    f"yaw={vp.yaw:.2f}, pitch={vp.pitch:.2f}, roll={vp.roll:.2f}\n"
                )
            # Compute vehicle_start_pose in either START or CURRENT
            if self.use_start_frame:
                if self.start_pose_abs is None:
                    self.start_pose_abs = vehicle.pose
                vehicle_start_pose = vehicle.pose.to_frame(
                    ObjectFrameEnum.START,
                    vehicle.pose,
                    self.start_pose_abs
                )
                mode = "START"
            else:
                vehicle_start_pose = vehicle.pose
                mode = "CURRENT"
            with open(f"data/{tstamp}_vehstate.txt", "a") as f:
                f.write(
                    f"AFTER_TRANSFORM "
                    f"x={vehicle_start_pose.x:.3f}, "
                    f"y={vehicle_start_pose.y:.3f}, "
                    f"z={vehicle_start_pose.z:.3f}, "
                    f"yaw={vehicle_start_pose.yaw:.2f}, "
                    f"pitch={vehicle_start_pose.pitch:.2f}, "
                    f"roll={vehicle_start_pose.roll:.2f}, "
                    f"frame={mode}\n"
                )
        if self.camera_front == False:
            start = time.time()
            undistorted_img, current_K = self.undistort_image(lastest_image, self.K, self.D)
            end = time.time()
            # print('-------processing time undistort_image---', end -start)
            self.current_K = current_K
            orig_H, orig_W = undistorted_img.shape[:2]

            # --- Begin modifications for three-angle detection ---
            img_normal = undistorted_img
        else:
            img_normal = lastest_image.copy()
            undistorted_img = lastest_image.copy()
            orig_H, orig_W = lastest_image.shape[:2]
            self.current_K = self.K
        results_normal = self.detector(img_normal, conf=0.25, classes=[0])
        combined_boxes = []
        if not self.enable_tracking:
            self.cone_counter = 0
        if self.orientation:
            img_left = cv2.rotate(undistorted_img.copy(), cv2.ROTATE_90_COUNTERCLOCKWISE)
            img_right = cv2.rotate(undistorted_img.copy(), cv2.ROTATE_90_CLOCKWISE)
            results_left = self.detector(img_left, conf=0.05, classes=[0])
            results_right = self.detector(img_right, conf=0.05, classes=[0])
            boxes_left = np.array(results_left[0].boxes.xywh.cpu()) if len(results_left) > 0 else []
            boxes_right = np.array(results_right[0].boxes.xywh.cpu()) if len(results_right) > 0 else []
            for box in boxes_left:
                cx, cy, w, h = box
                new_cx = cy
                new_cy = orig_W - 1 - cx
                combined_boxes.append((new_cx, new_cy, h, w, AgentActivityEnum.RIGHT))
            for box in boxes_right:
                cx, cy, w, h = box
                new_cx = orig_H - 1 - cy
                new_cy = cx
                combined_boxes.append((new_cx, new_cy, h, w, AgentActivityEnum.LEFT))

        boxes_normal = np.array(results_normal[0].boxes.xywh.cpu()) if len(results_normal) > 0 else []
        for box in boxes_normal:
            cx, cy, w, h = box
            combined_boxes.append((cx, cy, w, h, AgentActivityEnum.STANDING))

        if getattr(self, 'visualize_2d', False):
            for (cx, cy, w, h, activity) in combined_boxes:
                left = int(cx - w / 2)
                right = int(cx + w / 2)
                top = int(cy - h / 2)
                bottom = int(cy + h / 2)
                if activity == AgentActivityEnum.STANDING:
                    color = (255, 0, 0)
                    label = "STANDING"
                elif activity == AgentActivityEnum.RIGHT:
                    color = (0, 255, 0)
                    label = "RIGHT"
                elif activity == AgentActivityEnum.LEFT:
                    color = (0, 0, 255)
                    label = "LEFT"
                else:
                    color = (255, 255, 255)
                    label = "UNKNOWN"
                cv2.rectangle(undistorted_img, (left, top), (right, bottom), color, 2)
                cv2.putText(undistorted_img, label, (left, max(top - 5, 20)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
            cv2.imshow("Detection - Cone 2D", undistorted_img)

        start = time.time()
        pts_cam = transform_points_l2c(lidar_down, self.T_l2c)
        projected_pts = project_points(pts_cam, self.current_K, lidar_down)
        end = time.time()
        # print('-------processing time1---', end -start)

        agents = {}

        for i, box_info in enumerate(combined_boxes):
            cx, cy, w, h, activity = box_info
            left = int(cx - w / 2)
            right = int(cx + w / 2)
            top = int(cy - h / 2)
            bottom = int(cy + h / 2)
            mask = (projected_pts[:, 0] >= left) & (projected_pts[:, 0] <= right) & \
                   (projected_pts[:, 1] >= top) & (projected_pts[:, 1] <= bottom)
            roi_pts = projected_pts[mask]
            if roi_pts.shape[0] < 5:
                continue

            points_3d = roi_pts[:, 2:5]
            points_3d = filter_points_within_threshold(points_3d, 30)
            points_3d = filter_depth_points(points_3d, max_depth_diff=0.3)

            if self.use_cyl_roi:
                global_filtered = filter_points_within_threshold(lidar_down, 30)
                roi_cyl = cylindrical_roi(global_filtered, np.mean(points_3d, axis=0), radius=0.4, height=1.2)
                refined_cluster = remove_ground_by_min_range(roi_cyl, z_range=0.01)
                refined_cluster = filter_depth_points(refined_cluster, max_depth_diff=0.3)
            else:
                refined_cluster = points_3d.copy()
            # end1 = time.time()
            if refined_cluster.shape[0] < 4:
                continue

            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(refined_cluster)
            obb = pcd.get_oriented_bounding_box()
            refined_center = obb.center
            dims = tuple(obb.extent)
            R_lidar = obb.R.copy()

            refined_center_hom = np.append(refined_center, 1)
            refined_center_vehicle_hom = self.T_l2v @ refined_center_hom
            refined_center_vehicle = refined_center_vehicle_hom[:3]

            R_vehicle = self.T_l2v[:3, :3] @ R_lidar
            yaw, pitch, roll = R.from_matrix(R_vehicle).as_euler('zyx', degrees=False)
            refined_center = refined_center_vehicle

            if self.use_start_frame:
                if self.start_pose_abs is None:
                    self.start_pose_abs = vehicle.pose
                vehicle_start_pose = vehicle.pose.to_frame(
                    ObjectFrameEnum.START,
                    vehicle.pose,
                    self.start_pose_abs
                )
                T_vehicle_to_start = pose_to_matrix(vehicle_start_pose)
                xp, yp, zp = (T_vehicle_to_start @ np.append(refined_center, 1))[:3]
                out_frame = ObjectFrameEnum.START
            else:
                xp, yp, zp = refined_center
                out_frame = ObjectFrameEnum.CURRENT

            new_pose = ObjectPose(
                t=current_time,
                x=xp, y=yp, z=zp,
                yaw=yaw, pitch=pitch, roll=roll,
                frame=out_frame
            )

            # --- Optional tracking ---
            if self.enable_tracking:
                existing_id = match_existing_cone(
                    np.array([new_pose.x, new_pose.y, new_pose.z]),
                    dims,
                    self.tracked_agents,
                    distance_threshold=2.0
                )
                if existing_id is not None:
                    old_state = self.tracked_agents[existing_id]
                    if vehicle.v < 100:
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

        self.current_agents = agents

        # If tracking not enabled, return only current frame detections
        if not self.enable_tracking:
            for agent_id, agent in self.current_agents.items():
                p = agent.pose
                rospy.loginfo(
                    f"Agent ID: {agent_id}\n"
                    f"Pose: (x: {p.x:.3f}, y: {p.y:.3f}, z: {p.z:.3f}, "
                    f"yaw: {p.yaw:.3f}, pitch: {p.pitch:.3f}, roll: {p.roll:.3f})\n"
                    f"Velocity: (vx: {agent.velocity[0]:.3f}, vy: {agent.velocity[1]:.3f}, vz: {agent.velocity[2]:.3f})\n"
                    f"type:{agent.activity}"
                )
            end = time.time()
            # print('-------processing time', end -start)
            return self.current_agents

        stale_ids = [agent_id for agent_id, agent in self.tracked_agents.items()
                     if current_time - agent.pose.t > 20.0]
        for agent_id in stale_ids:
            rospy.loginfo(f"Removing stale agent: {agent_id}\n")
            del self.tracked_agents[agent_id]
        if self.enable_tracking:
            for agent_id, agent in self.tracked_agents.items():
                p = agent.pose
                rospy.loginfo(
                    f"Agent ID: {agent_id}\n"
                    f"Pose: (x: {p.x:.3f}, y: {p.y:.3f}, z: {p.z:.3f}, "
                    f"yaw: {p.yaw:.3f}, pitch: {p.pitch:.3f}, roll: {p.roll:.3f})\n"
                    f"Velocity: (vx: {agent.velocity[0]:.3f}, vy: {agent.velocity[1]:.3f}, vz: {agent.velocity[2]:.3f})\n"
                    f"type:{agent.activity}"
                )
        end = time.time()
        # print('-------processing time', end -start)
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