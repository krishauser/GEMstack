from ...state import AllState, VehicleState, ObjectPose, ObjectFrameEnum, Obstacle, ObstacleMaterialEnum, \
    ObstacleStateEnum
from ..interface.gem import GEMInterface
from ..component import Component
from .perception_utils import *
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
            save_data: bool = True,
            orientation: bool = True,
            use_start_frame: bool = True,
            **kwargs
    ):
        # Core interfaces and state
        self.vehicle_interface = vehicle_interface
        self.current_obstacles = {}
        self.tracked_obstacles = {}
        self.cone_counter = 0
        self.latest_image = None
        self.latest_lidar = None
        self.bridge = CvBridge()
        self.start_pose_abs = None
        self.start_time = None

        # Config flags
        self.camera_name = camera_name
        self.enable_tracking = enable_tracking
        self.visualize_2d = visualize_2d
        self.use_cyl_roi = use_cyl_roi
        self.save_data = save_data
        self.orientation = orientation
        self.use_start_frame = use_start_frame

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
        self.K = np.array(cam_cfg['K'])
        self.D = np.array(cam_cfg['D'])
        self.T_l2c = np.array(cam_cfg['T_l2c'])
        self.T_l2v = np.array(cam_cfg['T_l2v'])

        # Derived transforms

        self.undistort_map1 = None
        self.undistort_map2 = None
        self.camera_front = (camera_name == 'front')

    def rate(self) -> float:
        return 8

    def state_inputs(self) -> list:
        return ['vehicle']

    def state_outputs(self) -> list:
        return ['obstacles']

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
        ], queue_size=500, slop=0.03)
        self.sync.registerCallback(self.synchronized_callback)

        # Initialize the YOLO detector
        self.detector = YOLO('GEMstack/knowledge/detection/cone.pt')
        self.detector.to('cuda')
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

    def update(self, vehicle: VehicleState) -> Dict[str, Obstacle]:
        downsample = False
        # Gate guards against data not being present for both sensors:
        if self.latest_image is None or self.latest_lidar is None:
            return {}
        latest_image = self.latest_image.copy()

        # Set up current time variables
        start = time.time()
        current_time = self.vehicle_interface.time()

        if downsample:
            lidar_down = downsample_points(self.latest_lidar, voxel_size=0.1)
        else:
            lidar_down = self.latest_lidar.copy()

        if self.start_time is None:
            self.start_time = current_time
        time_elapsed = current_time - self.start_time

        # Ensure data/ exists and build timestamp
        if self.save_data:
            self.save_sensor_data(vehicle=vehicle, latest_image=latest_image)

        if self.camera_front == False:
            start = time.time()
            undistorted_img, current_K = self.undistort_image(latest_image, self.K, self.D)
            end = time.time()
            # print('-------processing time undistort_image---', end -start)
            self.current_K = current_K
            orig_H, orig_W = undistorted_img.shape[:2]

            # --- Begin modifications for three-angle detection ---
            img_normal = undistorted_img
        else:
            img_normal = latest_image.copy()
            undistorted_img = latest_image.copy()
            orig_H, orig_W = latest_image.shape[:2]
            self.current_K = self.K
            # print(self.K)
            # print(self.T_l2c)
        results_normal = self.detector(img_normal, conf=0.35, classes=[0])
        combined_boxes = []
        if not self.enable_tracking:
            self.cone_counter = 0
        if self.orientation:
            img_left = cv2.rotate(undistorted_img.copy(), cv2.ROTATE_90_COUNTERCLOCKWISE)
            img_right = cv2.rotate(undistorted_img.copy(), cv2.ROTATE_90_CLOCKWISE)
            results_left = self.detector(img_left, conf=0.15, classes=[0])
            results_right = self.detector(img_right, conf=0.15, classes=[0])
            boxes_left = np.array(results_left[0].boxes.xywh.cpu()) if len(results_left) > 0 else []
            boxes_right = np.array(results_right[0].boxes.xywh.cpu()) if len(results_right) > 0 else []
            for box in boxes_left:
                cx, cy, w, h = box
                new_cx = orig_W - 1 - cy
                new_cy = cx
                new_w = h  # Swap width and height.
                new_h = w
                combined_boxes.append((new_cx, new_cy, new_w, new_h, ObstacleStateEnum.RIGHT))
            for box in boxes_right:
                cx, cy, w, h = box
                new_cx = cy
                new_cy = orig_H - 1 - cx
                new_w = h  # Swap width and height.
                new_h = w
                combined_boxes.append((new_cx, new_cy, new_w, new_h, ObstacleStateEnum.LEFT))

        boxes_normal = np.array(results_normal[0].boxes.xywh.cpu()) if len(results_normal) > 0 else []
        for box in boxes_normal:
            cx, cy, w, h = box
            combined_boxes.append((cx, cy, w, h, ObstacleStateEnum.STANDING))

        # Visualize the received images in 2D with their corresponding labels
        # It draws rectangles and labels on the images:
        if getattr(self, 'visualize_2d', False):
            for (cx, cy, w, h, state) in combined_boxes:
                left = int(cx - w / 2)
                right = int(cx + w / 2)
                top = int(cy - h / 2)
                bottom = int(cy + h / 2)
                if state == ObstacleStateEnum.STANDING:
                    color = (255, 0, 0)
                    label = "STANDING"
                elif state == ObstacleStateEnum.RIGHT:
                    color = (0, 255, 0)
                    label = "RIGHT"
                elif state == ObstacleStateEnum.LEFT:
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
        # Transform the lidar points from lidar frame of reference to camera EXTRINSIC frame of reference.
        # Then project the pixels onto the lidar points to "paint them" (essentially determine which points are associated with detected objects)
        pts_cam = transform_points_l2c(lidar_down, self.T_l2c)
        projected_pts = project_points(pts_cam, self.current_K, lidar_down)
        # What is returned:
        # projected_pts[:, 0]: u-coordinate in the image (horizontal pixel position)
        # projected_pts[:, 1]: v-coordinate in the image (vertical pixel position)
        # projected_pts[:, 2:5]: original X, Y, Z coordinates in the LiDAR frame

        end = time.time()
        # print('-------processing time1---', end -start)

        obstacles = {}

        for i, box_info in enumerate(combined_boxes):
            cx, cy, w, h, state = box_info
            # print(cx, cy, w, h)
            left = int(cx - w / 1.6)
            right = int(cx + w / 1.6)
            top = int(cy - h / 2)
            bottom = int(cy + h / 2)
            mask = (projected_pts[:, 0] >= left) & (projected_pts[:, 0] <= right) & \
                   (projected_pts[:, 1] >= top) & (projected_pts[:, 1] <= bottom)
            roi_pts = projected_pts[mask]
            # print(roi_pts)
            if roi_pts.shape[0] < 5:
                continue

            points_3d = roi_pts[:, 2:5]

            points_3d = filter_points_within_threshold(points_3d, 40)
            points_3d = remove_ground_by_min_range(points_3d, z_range=0.08)
            points_3d = filter_depth_points(points_3d, max_depth_diff=0.5)

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
                T_vehicle_to_start = vehicle_start_pose.transform()
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
                    self.tracked_obstacles,
                    distance_threshold=2
                )
                if existing_id is not None:
                    old_state = self.tracked_obstacles[existing_id]
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
                        updated_obstacle = Obstacle(
                            pose=updated_pose,
                            dimensions=dims,
                            outline=None,
                            material=ObstacleMaterialEnum.TRAFFIC_CONE,
                            state=state,
                            collidable=True
                        )
                    else:
                        updated_obstacle = old_state
                    obstacles[existing_id] = updated_obstacle
                    self.tracked_obstacles[existing_id] = updated_obstacle
                else:
                    obstacle_id = f"Cone{self.cone_counter}"
                    self.cone_counter += 1
                    new_obstacle = Obstacle(
                        pose=new_pose,
                        dimensions=dims,
                        outline=None,
                        material=ObstacleMaterialEnum.TRAFFIC_CONE,
                        state=state,
                        collidable=True
                    )
                    obstacles[obstacle_id] = new_obstacle
                    self.tracked_obstacles[obstacle_id] = new_obstacle
            else:
                obstacle_id = f"Cone{self.cone_counter}"
                self.cone_counter += 1
                new_obstacle = Obstacle(
                    pose=new_pose,
                    dimensions=dims,
                    outline=None,
                    material=ObstacleMaterialEnum.TRAFFIC_CONE,
                    state=state,
                    collidable = True
                )
                obstacles[obstacle_id] = new_obstacle

        self.current_obstacles = obstacles

        # If tracking not enabled, return only current frame detections
        if not self.enable_tracking:
            for obstacle_id, obstacle in self.current_obstacles.items():
                p = obstacle.pose
                rospy.loginfo(
                    f"Cone ID: {obstacle_id}\n"
                    f"Pose: (x: {p.x:.3f}, y: {p.y:.3f}, z: {p.z:.3f}, "
                    f"yaw: {p.yaw:.3f}, pitch: {p.pitch:.3f}, roll: {p.roll:.3f})\n"
                    f"state:{obstacle.state}"
                )
            end = time.time()
            # print('-------processing time', end -start)
            return self.current_obstacles

        stale_ids = [obstacle_id for obstacle_id, obstacle in self.tracked_obstacles.items()
                     if current_time - obstacle.pose.t > 20.0]
        for obstacle_id in stale_ids:
            rospy.loginfo(f"Removing stale obstacle: {obstacle_id}\n")
            del self.tracked_obstacles[obstacle_id]
        if self.enable_tracking:
            for obstacle_id, obstacle in self.tracked_obstacles.items():
                p = obstacle.pose
                rospy.loginfo(
                    f"Cone ID: {obstacle_id}\n"
                    f"Pose: (x: {p.x:.3f}, y: {p.y:.3f}, z: {p.z:.3f}, "
                    f"yaw: {p.yaw:.3f}, pitch: {p.pitch:.3f}, roll: {p.roll:.3f})\n"
                    f"state:{obstacle.state}"
                )
        end = time.time()
        # print('-------processing time', end -start)
        return self.tracked_obstacles

    def save_sensor_data(self, vehicle: VehicleState, latest_image) -> None:
        os.makedirs("data", exist_ok=True)
        tstamp = int(self.vehicle_interface.time() * 1000)
        # 1) Dump raw image
        cv2.imwrite(f"data/{tstamp}_image.png", latest_image)
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

    # ----- Fake Cone Detector 2D (for Testing Purposes) -----


class FakConeDetector(Component):
    def __init__(self, vehicle_interface: GEMInterface):
        """
        Initializes the FakeConeDetector with two parking spots:
        - One standard 2x2 rectangular spot
        - One wider mouth for easier entry
        """
        self.vehicle_interface = vehicle_interface

    def rate(self):
        return 4.0

    def state_inputs(self):
        return ['vehicle']

    def state_outputs(self):
        return ['obstacles']

    def update(self, vehicle: VehicleState) -> Dict[str, Obstacle]:
        """
        Called every simulation step, updates the timestamp and publishes the obstacle states.
        """

        current_time = self.vehicle_interface.time()
        
        # === Standard Parking Spot ===
        # cones_standard = {
        #     'cone0': (5.0, 5.0, 0.5, 0.5),   # Front Left
        #     'cone1': (5.0, 7.0, 0.5, 0.5),   # Back Left
        #     'cone2': (7.0, 5.0, 0.5, 0.5),   # Front Right
        #     'cone3': (7.0, 7.0, 0.5, 0.5)    # Back Right
        # }

        # === Wider Mouth Parking Spot ===
        cones_wide = {
            'cone4': (32.0, 13.0, 0.5, 0.5),  # Front Left (wide)
            'cone5': (36.0, 13.0, 0.5, 0.5),  # Back Left
            'cone6': (29.0, 8.0, 0.5, 0.5),  # Front Right (wide)
            'cone7': (33.0, 8.0, 0.5, 0.5)   # Back Right
        }

        # Populate the obstacle states
        res = {}
        for name, box in {**cones_wide}.items():
            res[name] = box_to_fake_obstacle(box, current_time)

        # Update timestamp
        for obstacle in res.values():
            obstacle.pose.t = current_time

        rospy.loginfo(f"[FakeConeDetector] Simulated Two Parking Spots Detected")
        return res

def box_to_fake_obstacle(box, current_time):
    """
    Helper function to create a fake obstacle (cone) from bounding box coordinates.
    """
    x, y, w, h = box
    pose = ObjectPose(
        t=current_time,
        x=x,
        y=y,
        z=0.0,
        yaw=0.0,
        pitch=0.0,
        roll=0.0,
        frame=ObjectFrameEnum.START
    )

    dims = (w, h, 1.0)

    new_obstacle = Obstacle(
                                pose=pose,
                                dimensions=dims,
                                outline=None,
                                material=ObstacleMaterialEnum.TRAFFIC_CONE,
                                collidable=True,
                                state=ObstacleStateEnum.STANDING
                            )
    print("Obstacles made")
    return new_obstacle



if __name__ == '__main__':
    pass