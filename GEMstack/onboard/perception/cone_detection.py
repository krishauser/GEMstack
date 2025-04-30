from ...state import AllState, VehicleState, ObjectPose, ObjectFrameEnum, AgentState, AgentEnum, AgentActivityEnum
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


class ConeDetector3D(Component):
    """
    Detects cones by fusing YOLO 2D detections with LiDAR point cloud data.

    Tracking is optional: set `enable_tracking=False` to disable persistent tracking
    and return only detections from the current frame.
    """

    def __init__(self, vehicle_interface: GEMInterface):
        self.vehicle_interface = vehicle_interface
        self.enable_tracking = False
        self.current_agents = {}
        self.tracked_agents = {}
        self.cone_counter = 0
        self.latest_image = None
        self.latest_lidar = None
        self.bridge = CvBridge()
        self.start_pose_abs = None
        self.camera_front = True
        self.visualize_2d = False
        self.use_cyl_roi = False
        self.start_time = None
        self.use_start_frame = False
        self.save_data = False
        self.orientation = False

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
        self.detector = YOLO('/home/gem/s2025_perception_merge/GEMstack/GEMstack/knowledge/detection/cone.pt')
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
            self.D = np.array([-2.70136325e-01, 1.64393255e-01, -1.60720782e-03, -7.41246708e-05,
                               -6.19939758e-02])

        self.T_l2v = np.array([[0.99939639, 0.02547917, 0.023615, 1.1],
                               [-0.02530848, 0.99965156, -0.00749882, 0.03773583],
                               [-0.02379784, 0.00689664, 0.999693, 1.95320223],
                               [0., 0., 0., 1.]])
        if self.camera_front:
            self.T_l2c = np.array([
                [0.001090, -0.999489, -0.031941, 0.149698],
                [-0.007664, 0.031932, -0.999461, -0.397813],
                [0.999970, 0.001334, -0.007625, -0.691405],
                [0., 0., 0., 1.000000]
            ])
        else:
            self.T_l2c = np.array([[-0.71836368, -0.69527204, -0.02346088, 0.05718003],
                                   [-0.09720448, 0.13371206, -0.98624154, -0.1598301],
                                   [0.68884317, -0.7061996, -0.16363744, -1.04767285],
                                   [0., 0., 0., 1.]]
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

        # Build timestamp
        current_time = self.vehicle_interface.time()
        if self.start_time is None:
            self.start_time = current_time
        time_elapsed = current_time - self.start_time

        if self.save_data:
            self.save_sensor_data(vehicle=vehicle)

        undistorted_img, current_K = undistort_image(self.latest_image, self.K, self.D)
        self.current_K = current_K
        self.latest_image = undistorted_img
        orig_H, orig_W = undistorted_img.shape[:2]

        # --- Begin modifications for three-angle detection ---
        img_normal = undistorted_img
        results_normal = self.detector(img_normal, conf=0.3, classes=[0])
        combined_boxes = []

        # If we wish to determine the orientation of the cones do the following maybe?
        # Why are we fabricating left and right oriented detections and adding it to the same list?
        # Shouldn't we be just determining whether something is oriented left or right using the received images?
        if self.orientation:
            img_left = cv2.rotate(undistorted_img.copy(), cv2.ROTATE_90_COUNTERCLOCKWISE)
            img_right = cv2.rotate(undistorted_img.copy(), cv2.ROTATE_90_CLOCKWISE)
            results_left = self.detector(img_left, conf=0.3, classes=[0])
            results_right = self.detector(img_right, conf=0.3, classes=[0])
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

        # Visualize the received images in 2D with their corresponding labels
        # It draws rectangles and labels on the images:
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

        # Downsample the lidar data if enabled:
        if downsample:
            lidar_down = downsample_points(self.latest_lidar, voxel_size=0.1)
        else:
            lidar_down = self.latest_lidar.copy() # Do we need to make a copy here?

        # Transform the lidar points from lidar frame of reference to camera EXTRINSIC frame of reference.
        # Then project the pixels onto the lidar points to "paint them" (essentially determine which ones are relevant for us)
        pts_cam = transform_points_l2c(lidar_down, self.T_l2c)
        projected_pts = project_points(pts_cam, self.current_K, lidar_down)
        # What is returned:
        # projected_pts[:, 0]: u-coordinate in the image (horizontal pixel position)
        # projected_pts[:, 1]: v-coordinate in the image (vertical pixel position)
        # projected_pts[:, 2:5]: original X, Y, Z coordinates in the LiDAR frame
        
        # WE WOULD RUN POINT PILLARS ON THE TRANSFORMED POINTS HERE

        agents = {}

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
            points_3d = roi_pts[:, 2:5]
            points_3d = filter_points_within_threshold(points_3d, 30)
            points_3d = filter_depth_points(points_3d, max_depth_diff=0.3)

            if self.use_cyl_roi:
                global_filtered = filter_points_within_threshold(lidar_down, 30)
                roi_cyl = cylindrical_roi(global_filtered, np.mean(points_3d, axis=0), radius=0.3, height=1.2)
                refined_cluster = remove_ground_by_min_range(roi_cyl, z_range=0.01)
                refined_cluster = filter_depth_points(refined_cluster, max_depth_diff=0.2)
            else:
                refined_cluster = remove_ground_by_min_range(points_3d, z_range=0.05)
            end1 = time.time()
            if refined_cluster.shape[0] < 4:
                continue

            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(refined_cluster)
            obb = pcd.get_oriented_bounding_box()
            refined_center = obb.center
            dims = tuple(obb.extent)
            R_lidar = obb.R.copy()
            end2 = time.time()

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
            return self.current_agents

        stale_ids = [agent_id for agent_id, agent in self.tracked_agents.items()
                     if current_time - agent.pose.t > 5.0]
        for agent_id in stale_ids:
            rospy.loginfo(f"Removing stale agent: {agent_id}\n")
            del self.tracked_agents[agent_id]

        return self.tracked_agents
    
    def save_sensor_data(self, vehicle: VehicleState) -> None:
        # Ensure data directory exists:
        os.makedirs("data", exist_ok=True)
        
        tstamp = int(self.vehicle_interface.time() * 1000)
        # 1) Dump raw image
        cv2.imwrite(f"data/{tstamp}_image.png", self.latest_image)
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
