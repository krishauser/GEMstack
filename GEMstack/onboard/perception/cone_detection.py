from ...state import AllState, VehicleState, ObjectPose, ObjectFrameEnum, Obstacle, ObstacleMaterialEnum, \
    ObstacleStateEnum
from ..interface.gem import GEMInterface
from ..component import Component
from .perception_utils import *
from .perception_utils_gem import *
from ultralytics import YOLO
import cv2
from typing import Dict
import open3d as o3d
import numpy as np
from scipy.spatial.transform import Rotation as R
import rospy
from sensor_msgs.msg import PointCloud2, Image
from message_filters import Subscriber, ApproximateTimeSynchronizer
from cv_bridge import CvBridge
import time
import os
import yaml


class ConeDetector3D(Component):
    """
    Detects traffic cones by fusing YOLO 2D detections with LiDAR point cloud data.
    Tracking is optional: disable persistent tracking via `enable_tracking`.
    Supports multiple cameras; each camera’s intrinsics/extrinsics are loaded via YAML.
    """

    def __init__(
            self,
            vehicle_interface: GEMInterface,
            camera_name: str,
            camera_calib_file: str,
            enable_tracking: bool = True,
            visualize_2d: bool = False,
            use_cyl_roi: bool = False,
            save_data: bool = True,
            orientation: bool = True,
            use_start_frame: bool = True,
            downsample: bool = False,
            conf_normal: float = 0.35,
            conf_left: float = 0.15,
            conf_right: float = 0.15,
            max_depth: float = 40,
            z_range: float = 0.08,
            max_depth_diff: float = 0.5,
            alpha: float = 0.1,
            expiring_time: float = 20.0,
            update_rate: int = 8,
            **kwargs
    ):
        # --- Core interfaces ---
        self.vehicle_interface = vehicle_interface
        # Containers for current and tracked obstacles
        self.current_obstacles = {}
        self.tracked_obstacles = {}
        self.cone_counter = 0  # unique ID counter

        # Latest sensor data placeholders
        self.latest_image = None
        self.latest_lidar = None
        self.bridge = CvBridge()

        # Start pose & time references
        self.start_pose_abs = None
        self.start_time = None
        self.update_rate = update_rate
        # --- Config flags from init args ---
        self.camera_name = camera_name
        self.enable_tracking = enable_tracking
        self.visualize_2d = visualize_2d
        self.use_cyl_roi = use_cyl_roi
        self.save_data = save_data
        self.orientation = orientation
        self.use_start_frame = use_start_frame
        # Processing parameters
        self.downsample = downsample
        self.conf_normal = conf_normal
        self.conf_left = conf_left
        self.conf_right = conf_right
        self.max_depth = max_depth
        self.z_range = z_range
        self.max_depth_diff = max_depth_diff
        self.alpha = alpha
        self.expiring_time = expiring_time

        # --- Load camera calibration (intrinsics/extrinsics) ---
        with open(camera_calib_file, 'r') as f:
            calib = yaml.safe_load(f)
        cam_cfg = calib['cameras'][camera_name]
        self.K = np.array(cam_cfg['K'])           # Camera intrinsic matrix
        self.D = np.array(cam_cfg['D'])           # Distortion coefficients
        self.T_l2c = np.array(cam_cfg['T_l2c'])   # LiDAR-to-camera transform
        self.T_l2v = np.array(cam_cfg['T_l2v'])   # LiDAR-to-vehicle transform

        # Placeholders for undistortion maps
        self.undistort_map1 = None
        self.undistort_map2 = None
        self.camera_front = (camera_name == 'front')

    def rate(self) -> float:
        # Desired update frequency (Hz)
        return self.update_rate

    def state_inputs(self) -> list:
        # Inputs required: vehicle state
        return ['vehicle']

    def state_outputs(self) -> list:
        # Outputs provided: detected obstacles
        return ['obstacles']

    def initialize(self):
        # Determine the correct ROS topic for the camera
        rgb_topic_map = {
            'front': '/oak/rgb/image_raw',
            'front_right': '/camera_fr/arena_camera_node/image_raw',
        }
        rgb_topic = rgb_topic_map.get(
            self.camera_name,
            f'/{self.camera_name}/rgb/image_raw'
        )

        # Subscribe to synchronized RGB and LiDAR streams
        self.rgb_sub = Subscriber(rgb_topic, Image)
        self.lidar_sub = Subscriber('/ouster/points', PointCloud2)
        self.sync = ApproximateTimeSynchronizer([
            self.rgb_sub, self.lidar_sub
        ], queue_size=500, slop=0.03)
        self.sync.registerCallback(self.synchronized_callback)

        # Initialize the YOLO detector model on GPU
        self.detector = YOLO('GEMstack/knowledge/detection/cone.pt')
        self.detector.to('cuda')

        # Precompute camera-to-lidar transform and camera origin
        self.T_c2l = np.linalg.inv(self.T_l2c)
        self.R_c2l = self.T_c2l[:3, :3]
        self.camera_origin_in_lidar = self.T_c2l[:3, 3]

    def synchronized_callback(self, image_msg, lidar_msg):
        # Callback to convert incoming ROS messages to numpy formats
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
        except Exception as e:
            rospy.logerr("Failed to convert image: {}".format(e))
            self.latest_image = None
        # Convert PointCloud2 to numpy array
        self.latest_lidar = pc2_to_numpy(lidar_msg, want_rgb=False)

    def undistort_image(self, image, K, D):
        # Remove lens distortion and rectify the image
        h, w = image.shape[:2]
        newK, _ = cv2.getOptimalNewCameraMatrix(K, D, (w, h), 1, (w, h))
        if self.undistort_map1 is None or self.undistort_map2 is None:
            self.undistort_map1, self.undistort_map2 = cv2.initUndistortRectifyMap(
                K, D, R=None, newCameraMatrix=newK, size=(w, h), m1type=cv2.CV_32FC1
            )
        undistorted = cv2.remap(
            image, self.undistort_map1, self.undistort_map2,
            interpolation=cv2.INTER_NEAREST
        )
        return undistorted, newK

    def update(self, vehicle: VehicleState) -> Dict[str, Obstacle]:
        # Return empty if data not yet received from both sensors
        if self.latest_image is None or self.latest_lidar is None:
            return {}

        latest_image = self.latest_image.copy()
        current_time = self.vehicle_interface.time()

        # Optionally downsample point cloud
        if self.downsample:
            lidar_down = downsample_points(self.latest_lidar, voxel_size=0.1)
        else:
            lidar_down = self.latest_lidar.copy()

        # Initialize start time
        if self.start_time is None:
            self.start_time = current_time
        time_elapsed = current_time - self.start_time

        # Optionally save raw sensor data
        if self.save_data:
            self.save_sensor_data(vehicle=vehicle, latest_image=latest_image)

        # Undistort if not using front camera
        if not self.camera_front:
            undistorted_img, current_K = self.undistort_image(latest_image, self.K, self.D)
            img_normal = undistorted_img
        else:
            img_normal = latest_image.copy()
            undistorted_img = latest_image.copy()
            current_K = self.K

        # 2D detection on normal orientation
        results_normal = self.detector(img_normal, conf=self.conf_normal, classes=[0])

        combined_boxes = []
        if not self.enable_tracking:
            # Reset ID counter if no tracking
            self.cone_counter = 0

        # Optionally run detectors on rotated images for orientation
        if self.orientation:
            img_left = cv2.rotate(undistorted_img.copy(), cv2.ROTATE_90_COUNTERCLOCKWISE)
            img_right = cv2.rotate(undistorted_img.copy(), cv2.ROTATE_90_CLOCKWISE)
            results_left = self.detector(img_left, conf=self.conf_left, classes=[0])
            results_right = self.detector(img_right, conf=self.conf_right, classes=[0])
            boxes_left = np.array(results_left[0].boxes.xywh.cpu()) if len(results_left) > 0 else []
            boxes_right = np.array(results_right[0].boxes.xywh.cpu()) if len(results_right) > 0 else []
            # Transform rotated boxes back to normal image coords
            for box in boxes_left:
                cx, cy, w, h = box
                new_cx = undistorted_img.shape[1] - 1 - cy
                new_cy = cx
                combined_boxes.append((new_cx, new_cy, h, w, ObstacleStateEnum.RIGHT))
            for box in boxes_right:
                cx, cy, w, h = box
                new_cx = cy
                new_cy = undistorted_img.shape[0] - 1 - cx
                combined_boxes.append((new_cx, new_cy, h, w, ObstacleStateEnum.LEFT))

        # Add normal orientation detections
        boxes_normal = np.array(results_normal[0].boxes.xywh.cpu()) if len(results_normal) > 0 else []
        for box in boxes_normal:
            cx, cy, w, h = box
            combined_boxes.append((cx, cy, w, h, ObstacleStateEnum.STANDING))

        # 2D visualization if enabled
        if self.visualize_2d:
            # Draw bounding boxes and state labels on the image
            for (cx, cy, w, h, state) in combined_boxes:
                left = int(cx - w / 2)
                right = int(cx + w / 2)
                top = int(cy - h / 2)
                bottom = int(cy + h / 2)
                color = (255, 255, 255)
                if state == ObstacleStateEnum.STANDING:
                    color = (255, 0, 0)
                    label = "STANDING"
                elif state == ObstacleStateEnum.RIGHT:
                    color = (0, 255, 0)
                    label = "RIGHT"
                elif state == ObstacleStateEnum.LEFT:
                    color = (0, 0, 255)
                    label = "LEFT"
                cv2.rectangle(undistorted_img, (left, top), (right, bottom), color, 2)
                cv2.putText(undistorted_img, label, (left, max(top - 5, 20)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
            cv2.imshow("Detection - Cone 2D", undistorted_img)

        # Transform LiDAR points to camera frame and project to image
        pts_cam = transform_points_l2c(lidar_down, self.T_l2c)
        projected_pts = project_points(pts_cam, current_K, lidar_down)

        obstacles = {}
        # Fuse 2D boxes with 3D points
        for box_info in combined_boxes:
            cx, cy, w, h, state = box_info
            # Define ROI in image coords
            left = int(cx - w / 1.6)
            right = int(cx + w / 1.6)
            top = int(cy - h / 2)
            bottom = int(cy + h / 2)
            mask = (
                (projected_pts[:, 0] >= left) & (projected_pts[:, 0] <= right) &
                (projected_pts[:, 1] >= top) & (projected_pts[:, 1] <= bottom)
            )
            roi_pts = projected_pts[mask]
            if roi_pts.shape[0] < 5:
                continue

            # Extract 3D points and apply depth filters
            points_3d = roi_pts[:, 2:5]
            points_3d = filter_points_within_threshold(points_3d, self.max_depth)
            points_3d = remove_ground_by_min_range(points_3d, z_range=self.z_range)
            points_3d = filter_depth_points(points_3d, max_depth_diff=self.max_depth_diff)

            if self.use_cyl_roi:
                # Optionally refine with cylindrical ROI
                global_filtered = filter_points_within_threshold(lidar_down, 30)
                roi_cyl = cylindrical_roi(
                    global_filtered, np.mean(points_3d, axis=0), radius=0.4, height=1.2
                )
                refined_cluster = remove_ground_by_min_range(roi_cyl, z_range=0.01)
                refined_cluster = filter_depth_points(refined_cluster, max_depth_diff=0.3)
            else:
                refined_cluster = points_3d.copy()

            if refined_cluster.shape[0] < 4:
                continue

            # Compute oriented bounding box of the cluster
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(refined_cluster)
            obb = pcd.get_oriented_bounding_box()
            refined_center = obb.center
            dims = tuple(obb.extent)
            R_lidar = obb.R.copy()

            # Transform center to vehicle frame
            refined_center_hom = np.append(refined_center, 1)
            refined_center_vehicle_hom = self.T_l2v @ refined_center_hom
            refined_center_vehicle = refined_center_vehicle_hom[:3]

            # Compute orientation in vehicle frame
            R_vehicle = self.T_l2v[:3, :3] @ R_lidar
            yaw, pitch, roll = R.from_matrix(R_vehicle).as_euler('zyx', degrees=False)
            refined_center = refined_center_vehicle

            # Optionally convert to start frame
            if self.use_start_frame:
                if self.start_pose_abs is None:
                    self.start_pose_abs = vehicle.pose
                vehicle_start_pose = vehicle.pose.to_frame(
                    ObjectFrameEnum.START,
                    vehicle.pose,
                    self.start_pose_abs
                )
                xp, yp, zp = (vehicle_start_pose.transform() @ np.append(refined_center, 1))[:3]
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

            # Tracking: match to existing cone or add new
            if self.enable_tracking:
                existing_id = match_existing_cone(
                    np.array([new_pose.x, new_pose.y, new_pose.z]),
                    dims,
                    self.tracked_obstacles,
                    distance_threshold=2
                )
                if existing_id is not None:
                    # Smooth pose using exponential moving average
                    old_state = self.tracked_obstacles[existing_id]
                    alpha = self.alpha
                    avg_x = alpha * new_pose.x + (1 - alpha) * old_state.pose.x
                    avg_y = alpha * new_pose.y + (1 - alpha) * old_state.pose.y
                    avg_z = alpha * new_pose.z + (1 - alpha) * old_state.pose.z
                    avg_yaw = alpha * new_pose.yaw + (1 - alpha) * old_state.pose.yaw
                    avg_pitch = alpha * new_pose.pitch + (1 - alpha) * old_state.pose.pitch
                    avg_roll = alpha * new_pose.roll + (1 - alpha) * old_state.pose.roll

<<<<<<< Updated upstream
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
                        updated_obstacle = Obstacle(
                            pose=updated_pose,
                            dimensions=dims,
                            outline=None,
                            material=ObstacleMaterialEnum.TRAFFIC_CONE,
                            state=state,
                            collidable=False
                        )
                    else:
                        updated_obstacle = old_state
=======
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
                    updated_obstacle = Obstacle(
                        pose=updated_pose,
                        dimensions=dims,
                        outline=None,
                        material=ObstacleMaterialEnum.TRAFFIC_CONE,
                        state=state,
                        collidable=True
                    )
>>>>>>> Stashed changes
                    obstacles[existing_id] = updated_obstacle
                    self.tracked_obstacles[existing_id] = updated_obstacle
                else:
                    # Create new tracked obstacle
                    obstacle_id = f"Cone{self.cone_counter}"
                    self.cone_counter += 1
                    new_obstacle = Obstacle(
                        pose=new_pose,
                        dimensions=dims,
                        outline=None,
                        material=ObstacleMaterialEnum.TRAFFIC_CONE,
                        state=state,
<<<<<<< Updated upstream
                        collidable=False
=======
                        collidable=True
>>>>>>> Stashed changes
                    )
                    obstacles[obstacle_id] = new_obstacle
                    self.tracked_obstacles[obstacle_id] = new_obstacle
            else:
                # No tracking: return only current frame detections
                obstacle_id = f"Cone{self.cone_counter}"
                self.cone_counter += 1
                new_obstacle = Obstacle(
                    pose=new_pose,
                    dimensions=dims,
                    outline=None,
                    material=ObstacleMaterialEnum.TRAFFIC_CONE,
                    state=state,
<<<<<<< Updated upstream
                    collidable=False
=======
                    collidable=True
>>>>>>> Stashed changes
                )
                obstacles[obstacle_id] = new_obstacle

        # Clean up stale tracked cones
        stale_ids = [oid for oid, obs in self.tracked_obstacles.items()
                     if current_time - obs.pose.t > self.expiring_time]
        for oid in stale_ids:
            rospy.loginfo(f"Removing stale obstacle: {oid}\n")
            del self.tracked_obstacles[oid]

        # Log and return tracked obstacles if tracking enabled
        if self.enable_tracking:
            for obstacle_id, obstacle in self.tracked_obstacles.items():
                p = obstacle.pose
                rospy.loginfo(
                    f"Cone ID: {obstacle_id}\n"
                    f"Pose: (x: {p.x:.3f}, y: {p.y:.3f}, z: {p.z:.3f}, "
                    f"yaw: {p.yaw:.3f}, pitch: {p.pitch:.3f}, roll: {p.roll:.3f})\n"
                    f"state:{obstacle.state}"
                )
            return self.tracked_obstacles
        else:
            # Log and return detections for current frame only
            for obstacle_id, obstacle in obstacles.items():
                p = obstacle.pose
                rospy.loginfo(
                    f"Cone ID: {obstacle_id}\n"
                    f"Pose: (x: {p.x:.3f}, y: {p.y:.3f}, z: {p.z:.3f}, "
                    f"yaw: {p.yaw:.3f}, pitch: {p.pitch:.3f}, roll: {p.roll:.3f})\n"
                    f"state:{obstacle.state}"
                )
            return obstacles

    def save_sensor_data(self, vehicle: VehicleState, latest_image) -> None:
        # Save raw image, LiDAR, and vehicle state to disk for debugging
        os.makedirs("data", exist_ok=True)
        tstamp = int(self.vehicle_interface.time() * 1000)
        cv2.imwrite(f"data/{tstamp}_image.png", latest_image)
        np.savez(f"data/{tstamp}_lidar.npz", lidar=self.latest_lidar)
        with open(f"data/{tstamp}_vehstate.txt", "w") as f:
            vp = vehicle.pose
            f.write(
                f"BEFORE_TRANSFORM "
                f"x={vp.x:.3f}, y={vp.y:.3f}, z={vp.z:.3f}, "
                f"yaw={vp.yaw:.2f}, pitch={vp.pitch:.2f}, roll={vp.roll:.2f}\n"
            )
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
                f"x={vehicle_start_pose.x:.3f}, y={vehicle_start_pose.y:.3f}, z={vehicle_start_pose.z:.3f}, "
                f"yaw={vehicle_start_pose.yaw:.2f}, pitch={vehicle_start_pose.pitch:.2f}, "
                f"roll={vehicle_start_pose.roll:.2f}, frame={mode}\n"
            )


# ----- Fake Cone Detector 2D (for Testing Purposes) -----

class FakConeDetector(Component):
    """
    Simulates cone detections at preset time intervals for testing pipeline.
    """
    def __init__(self, vehicle_interface: GEMInterface):
        self.vehicle_interface = vehicle_interface
        # Simulated detection windows (start, end)
        self.times = [(5.0, 20.0), (30.0, 35.0)]
        self.t_start = None

    def rate(self):
        return 4.0

    def state_inputs(self):
        return ['vehicle']

    def state_outputs(self):
        return ['obstacles']

    def update(self, vehicle: VehicleState) -> Dict[str, Obstacle]:
        if self.t_start is None:
            self.t_start = self.vehicle_interface.time()
        t = self.vehicle_interface.time() - self.t_start
        res = {}
        for time_range in self.times:
            if time_range[0] <= t <= time_range[1]:
                res['cone0'] = box_to_fake_obstacle((0, 0, 0, 0))
                rospy.loginfo("Detected a Cone (simulated)")
        return res


def box_to_fake_obstacle(box):
    x, y, w, h = box
    pose = ObjectPose(t=0, x=x + w / 2, y=y + h / 2, z=0,
                      yaw=0, pitch=0, roll=0, frame=ObjectFrameEnum.CURRENT)
    dims = (w, h, 0)
    return Obstacle(pose=pose, dimensions=dims, outline=None,
                    material=ObstacleMaterialEnum.TRAFFIC_CONE,
                    state=ObstacleStateEnum.STANDING, collidable=True)


if __name__ == '__main__':
    pass
