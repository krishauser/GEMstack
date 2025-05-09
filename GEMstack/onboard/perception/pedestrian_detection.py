from ...state import AllState, VehicleState, ObjectPose, ObjectFrameEnum, AgentState, AgentEnum, AgentActivityEnum
from ..interface.gem import GEMInterface
from ..component import Component
from .perception_utils import *  # If you want to alias functions for clarity, do so in perception_utils
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


class PedestrianDetector3D(Component):
    """
    Detects pedestrians by fusing YOLO 2D detections with LiDAR point cloud data.
    Tracking is optional: `enable_tracking=False` returns only current-frame detections.
    Calculates and outputs each pedestrian's velocity and yaw rate.
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
            use_start_frame: bool = True,
            **kwargs
    ):
        # Core interfaces and state
        self.vehicle_interface = vehicle_interface
        self.current_agents = {}
        self.tracked_agents = {}
        self.pedestrian_counter = 0
        self.latest_image = None
        self.latest_lidar = None
        self.bridge = CvBridge()
        self.start_pose_abs = None
        self.start_time = None

        # Configuration
        self.camera_name = camera_name
        self.enable_tracking = enable_tracking
        self.visualize_2d = visualize_2d
        self.use_cyl_roi = use_cyl_roi
        self.save_data = save_data
        self.use_start_frame = use_start_frame

        # 1) Load LiDAR-to-vehicle transform
        self.T_l2v = np.array(T_l2v) if T_l2v is not None else np.array([
            [0.99939639, 0.02547917, 0.023615, 1.1],
            [-0.02530848, 0.99965156, -0.00749882, 0.03773583],
            [-0.02379784, 0.00689664, 0.999693, 1.95320223],
            [0.0, 0.0, 0.0, 1.0]
        ])

        # 2) Load camera intrinsics/extrinsics from YAML
        with open(camera_calib_file, 'r') as f:
            calib = yaml.safe_load(f)
        cam_cfg = calib['cameras'][camera_name]
        self.K = np.array(cam_cfg['K'])
        self.D = np.array(cam_cfg['D'])
        self.T_l2c = np.array(cam_cfg['T_l2c'])

        self.undistort_map1 = None
        self.undistort_map2 = None
        self.camera_front = (camera_name == 'front')

    def rate(self) -> float:
        return 8.0

    def state_inputs(self) -> list:
        return ['vehicle']

    def state_outputs(self) -> list:
        return ['agents']

    def initialize(self):
        # Subscribe to RGB & LiDAR streams
        rgb_topic_map = {
            'front': '/oak/rgb/image_raw',
            'front_right': '/camera_fr/arena_camera_node/image_raw',
        }
        rgb_topic = rgb_topic_map.get(self.camera_name,
                                      f'/{self.camera_name}/rgb/image_raw')
        self.rgb_sub = Subscriber(rgb_topic, Image)
        self.lidar_sub = Subscriber('/ouster/points', PointCloud2)
        self.sync = ApproximateTimeSynchronizer(
            [self.rgb_sub, self.lidar_sub], queue_size=500, slop=0.025
        )
        self.sync.registerCallback(self.synchronized_callback)

        # Initialize YOLO pedestrian detection model (COCO class 0)
        self.detector = YOLO('GEMstack/knowledge/detection/yolov8n.pt')
        self.detector.to('cuda')

        # Compute camera-to-LiDAR transform
        self.T_c2l = np.linalg.inv(self.T_l2c)
        self.R_c2l = self.T_c2l[:3, :3]
        self.camera_origin_in_lidar = self.T_c2l[:3, 3]

    def synchronized_callback(self, image_msg, lidar_msg):
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
        except Exception as e:
            rospy.logerr(f"Failed to convert image: {e}")
            self.latest_image = None
        self.latest_lidar = pc2_to_numpy(lidar_msg, want_rgb=False)

    def undistort_image(self, image, K, D):
        h, w = image.shape[:2]
        newK, _ = cv2.getOptimalNewCameraMatrix(K, D, (w, h), 1, (w, h))
        if self.undistort_map1 is None:
            self.undistort_map1, self.undistort_map2 = cv2.initUndistortRectifyMap(
                K, D, None, newK, (w, h), cv2.CV_32FC1
            )
        undistorted = cv2.remap(
            image, self.undistort_map1, self.undistort_map2,
            interpolation=cv2.INTER_NEAREST
        )
        return undistorted, newK

    def update(self, vehicle: VehicleState) -> Dict[str, AgentState]:
        # Ensure both image and LiDAR data are available
        if self.latest_image is None or self.latest_lidar is None:
            return {}

        current_time = self.vehicle_interface.time()
        if self.start_time is None:
            self.start_time = current_time

        # Optionally save raw sensor data
        if self.save_data:
            self.save_sensor_data(vehicle, self.latest_image)

        # Undistort image if needed
        if not self.camera_front:
            img, self.current_K = self.undistort_image(
                self.latest_image, self.K, self.D)
        else:
            img = self.latest_image.copy()
            self.current_K = self.K

        # Perform 2D detection
        results = self.detector(img, conf=0.35, classes=[0])
        boxes2d = (np.array(results[0].boxes.xywh.cpu())
                   if len(results) > 0 else [])

        # Project LiDAR points into image plane
        lidar_pts = self.latest_lidar.copy()
        pts_cam = transform_points_l2c(lidar_pts, self.T_l2c)
        projected = project_points(pts_cam, self.current_K, lidar_pts)

        agents: Dict[str, AgentState] = {}
        for (cx, cy, w, h) in boxes2d:
            # print(cx, cy, w, h)
            # Define ROI in image for each detection
            left = int(cx - w / 2)
            right = int(cx + w / 2)
            top = int(cy - h / 2)
            bottom = int(cy + h / 2)
            mask = ((projected[:, 0] >= left) & (projected[:, 0] <= right) &
                    (projected[:, 1] >= top) & (projected[:, 1] <= bottom))
            roi = projected[mask]
            if roi.shape[0] < 5:
                continue

            # Filter point cloud
            pts3d = roi[:, 2:5]
            pts3d = filter_points_within_threshold(pts3d, 40)
            pts3d = remove_ground_by_min_range(pts3d, z_range=0.08)
            pts3d = filter_depth_points(pts3d, max_depth_diff=0.5)

            if self.use_cyl_roi:
                glob = filter_points_within_threshold(lidar_pts, 30)
                cyl = cylindrical_roi(glob, np.mean(pts3d, axis=0), radius=0.5, height=2.0)
                pts3d = remove_ground_by_min_range(cyl, z_range=0.01)
                pts3d = filter_depth_points(pts3d, max_depth_diff=0.3)

            if pts3d.shape[0] < 4:
                continue

            # Fit oriented bounding box
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(pts3d)
            obb = pcd.get_oriented_bounding_box()
            center = obb.center
            dims = tuple(obb.extent)
            R_lidar = obb.R

            # Transform to vehicle frame
            c_hom = np.append(center, 1)
            veh_hom = self.T_l2v @ c_hom
            veh_center = veh_hom[:3]

            # Compute orientation
            R_veh = self.T_l2v[:3, :3] @ R_lidar
            yaw, pitch, roll = R.from_matrix(R_veh).as_euler('zyx', degrees=False)

            # Convert to START or CURRENT frame
            if self.use_start_frame:
                if self.start_pose_abs is None:
                    self.start_pose_abs = vehicle.pose
                start_pose = vehicle.pose.to_frame(
                    ObjectFrameEnum.START, vehicle.pose, self.start_pose_abs)
                T_vs = pose_to_matrix(start_pose)
                xp, yp, zp = (T_vs @ np.append(veh_center, 1))[:3]
                frame = ObjectFrameEnum.START
            else:
                xp, yp, zp = veh_center
                frame = ObjectFrameEnum.CURRENT

            new_pose = ObjectPose(
                t=current_time,
                x=xp, y=yp, z=zp,
                yaw=yaw, pitch=pitch, roll=roll,
                frame=frame
            )

            # Matching and tracking
            if self.enable_tracking:
                existing = match_existing_cone(
                    np.array([new_pose.x, new_pose.y, new_pose.z]),
                    dims, self.tracked_agents,
                    distance_threshold=1.0
                )
                if existing is not None:
                    old = self.tracked_agents[existing]
                    dt = max(new_pose.t - old.pose.t, 1e-3)
                    # Instantaneous velocity
                    vx = (new_pose.x - old.pose.x) / dt
                    vy = (new_pose.y - old.pose.y) / dt
                    vz = (new_pose.z - old.pose.z) / dt
                    # Yaw rate
                    yaw_rate = (new_pose.yaw - old.pose.yaw) / dt
                    speed = np.linalg.norm([vx, vy, vz])
                    activity = (AgentActivityEnum.MOVING
                                if speed > 0.1 else AgentActivityEnum.STOPPED)

                    updated = AgentState(
                        pose=new_pose,
                        dimensions=dims,
                        outline=None,
                        type=AgentEnum.PEDESTRIAN,
                        activity=activity,
                        velocity=(vx, vy, vz),
                        yaw_rate=yaw_rate
                    )
                    agents[existing] = updated
                    self.tracked_agents[existing] = updated

                else:
                    aid = f"Pedestrian{self.pedestrian_counter}"
                    self.pedestrian_counter += 1
                    new_agent = AgentState(
                        pose=new_pose,
                        dimensions=dims,
                        outline=None,
                        type=AgentEnum.PEDESTRIAN,
                        activity=AgentActivityEnum.STOPPED,
                        velocity=(0, 0, 0),
                        yaw_rate=0
                    )
                    agents[aid] = new_agent
                    self.tracked_agents[aid] = new_agent
                for agent_id, agent in self.current_agents.items():
                    p = agent.pose
                    rospy.loginfo(
                        f"Agent ID: {agent_id}\n"
                        f"Pose: (x: {p.x:.3f}, y: {p.y:.3f}, z: {p.z:.3f}, "
                        f"yaw: {p.yaw:.3f}, pitch: {p.pitch:.3f}, roll: {p.roll:.3f})\n"
                        f"Velocity: (vx: {agent.velocity[0]:.3f}, vy: {agent.velocity[1]:.3f}, vz: {agent.velocity[2]:.3f})\n"
                        f"type:{agent.activity}"
                    )
            else:
                aid = f"Pedestrian{self.pedestrian_counter}"
                self.pedestrian_counter += 1
                agents[aid] = AgentState(
                    pose=new_pose,
                    dimensions=dims,
                    outline=None,
                    type=AgentEnum.PEDESTRIAN,
                    activity=AgentActivityEnum.STOPPED,
                    velocity=(0, 0, 0),
                    yaw_rate=0
                )

        self.current_agents = agents

        # Remove stale tracked agents
        if self.enable_tracking:
            stale = [
                aid for aid, a in self.tracked_agents.items()
                if current_time - a.pose.t > 10.0
            ]
            for aid in stale:
                del self.tracked_agents[aid]
            return self.tracked_agents

        return self.current_agents

    def save_sensor_data(self, vehicle: VehicleState, latest_image) -> None:
        os.makedirs("data", exist_ok=True)
        t = int(self.vehicle_interface.time() * 1000)
        cv2.imwrite(f"data/{t}_image.png", latest_image)
        np.savez(f"data/{t}_lidar.npz", lidar=self.latest_lidar)
        # Write BEFORE_TRANSFORM state
        with open(f"data/{t}_vehstate.txt", "w") as f:
            vp = vehicle.pose
            f.write(
                f"BEFORE_TRANSFORM "
                f"x={vp.x:.3f}, y={vp.y:.3f}, z={vp.z:.3f}, "
                f"yaw={vp.yaw:.2f}, pitch={vp.pitch:.2f}, roll={vp.roll:.2f}\n"
            )
        # Compute start or current frame state
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
        with open(f"data/{t}_vehstate.txt", "a") as f:
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
            if time_range[0] <= t <= time_range[1]:
                res['Pedestrian0'] = box_to_fake_agent((0, 0, 0, 0))
                rospy.loginfo("Detected a Pedestrian (simulated)")
        return res


def box_to_fake_agent(box):
    x, y, w, h = box
    pose = ObjectPose(
        t=0, x=x + w / 2, y=y + h / 2, z=0,
        yaw=0, pitch=0, roll=0,
        frame=ObjectFrameEnum.CURRENT
    )
    dims = (w, h, 0)
    return AgentState(
        pose=pose,
        dimensions=dims,
        outline=None,
        type=AgentEnum.PEDESTRIAN,
        activity=AgentActivityEnum.MOVING,
        velocity=(0, 0, 0),
        yaw_rate=0
    )


if __name__ == '__main__':
    pass
