from ...state import AllState, VehicleState, ObjectPose, ObjectFrameEnum, AgentState, AgentEnum, AgentActivityEnum
from ..interface.gem import GEMInterface
from ..component import Component
from ultralytics import YOLO
from typing import Dict
from sklearn.cluster import DBSCAN
from scipy.spatial.transform import Rotation as R
import rospy
from sensor_msgs.msg import PointCloud2, Image
from message_filters import Subscriber, ApproximateTimeSynchronizer
from cv_bridge import CvBridge
import time
from .pedestrian_utils import * # Import the moved helper functions
from .pedestrian_utils_gem import * # Import the moved GEM related helper functions


# ----- Pedestrian Detector 2D (New Approach) -----

class PedestrianDetector2D(Component):
    """
    Detects pedestrians by fusing YOLO 2D detections with LiDAR point cloud data.

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
        self.pedestrian_counter = 0
        self.latest_image = None
        self.latest_lidar = None
        self.bridge = CvBridge()
        self.start_pose_abs = None

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
        # self.detector = YOLO('../../knowledge/detection/yolov8s.pt')
        self.detector = YOLO('GEMstack/knowledge/detection/yolov8s.pt')
        self.detector.to('cuda')
        self.K = np.array([[684.83331299, 0., 573.37109375],
                           [0., 684.60968018, 363.70092773],
                           [0., 0., 1.]])
        self.T_l2v = np.array([[0.99939639, 0.02547917, 0.023615, 1.1],
                               [-0.02530848, 0.99965156, -0.00749882, 0.03773583],
                               [-0.02379784, 0.00689664, 0.999693, 1.95320223],
                               [0., 0., 0., 1.]])
        self.T_l2c = np.array([
            [0.001090, -0.999489, -0.031941, 0.149698],
            [-0.007664, 0.031932, -0.999461, -0.397813],
            [0.999970, 0.001334, -0.007625, -0.691405],
            [0., 0., 0., 1.000000]
        ])
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
        print('image callback: ', step2-step1, 'lidar callback ', step3- step2)

    def update(self, vehicle: VehicleState) -> Dict[str, AgentState]:
        downsample = False
        if self.latest_image is None or self.latest_lidar is None:
            return {}

        current_time = self.vehicle_interface.time()
        # Run YOLO to obtain 2D detections (class 0: persons)
        results = self.detector(self.latest_image, conf=0.4, classes=[0])
        boxes = np.array(results[0].boxes.xywh.cpu())
        agents = {}

        if downsample == True:
            lidar_down = downsample_points(self.latest_lidar, voxel_size=0.1)
            pts_cam = transform_points_l2c(lidar_down, self.T_l2c)
            projected_pts = project_points(pts_cam, self.K, lidar_down)

        else:
            # New approach: project the entire LiDAR point cloud to the image plane
            step00 = time.time()
            lidar_down = self.latest_lidar.copy()
            step01 = time.time()
            pts_cam = transform_points_l2c(lidar_down, self.T_l2c)
            step02 = time.time()
            projected_pts = project_points(pts_cam, self.K, lidar_down)  # shape (N,5): [u, v, X, Y, Z]
            step03 = time.time()
            print(f'copy lidar data {step01-step00}s, transoforming to camear {step02-step01}s, transforming to image {step03-step02}s')

        # For each 2D bounding box, filter projected points instead of ray-casting
        for i, box in enumerate(boxes):
            start = time.time()
            cx, cy, w, h = box
            left = int(cx - w / 2)
            right = int(cx + w / 2)
            top = int(cy - h / 2)
            bottom = int(cy + h / 2)
            mask = (projected_pts[:, 0] >= left) & (projected_pts[:, 0] <= right) & \
                   (projected_pts[:, 1] >= top) & (projected_pts[:, 1] <= bottom)
            roi_pts = projected_pts[mask]
            if roi_pts.shape[0] < 5:
                continue
            # Extract the LiDAR 3D points corresponding to the ROI
            points_3d = roi_pts[:, 2:5]
            points_3d = filter_depth_points(points_3d, max_human_depth=0.8)



            # Cluster the points and remove ground
            refined_cluster = refine_cluster(points_3d, np.mean(points_3d, axis=0), eps=0.15, min_samples=10)
            refined_cluster = remove_ground_by_min_range(refined_cluster, z_range=0.03)
            end1 = time.time()
            print('refine cluster: ', end1-start)
            if refined_cluster.shape[0] < 5:
                continue

            # Compute the oriented bounding box
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(refined_cluster)
            obb = pcd.get_oriented_bounding_box()
            refined_center = obb.center
            dims = tuple(obb.extent)
            R_lidar = obb.R.copy()
            end2 = time.time()
            print('compute bounding box ', end2-end1)
            # Transform the refined center from LiDAR to Vehicle frame
            refined_center_hom = np.append(refined_center, 1)
            refined_center_vehicle_hom = self.T_l2v @ refined_center_hom
            refined_center_vehicle = refined_center_vehicle_hom[:3]

            R_vehicle = self.T_l2v[:3, :3] @ R_lidar
            euler_vehicle = R.from_matrix(R_vehicle).as_euler('zyx', degrees=False)
            yaw, pitch, roll = euler_vehicle
            refined_center = refined_center_vehicle

            # Convert from Vehicle frame to START frame
            if self.start_pose_abs is None:
                self.start_pose_abs = vehicle.pose  # Initialize once

            # Obtain the vehicle's pose in the START frame as a pose state.
            # Assume vehicle.pose.to_frame returns a pose state with attributes x, y, z, yaw, pitch, roll.
            vehicle_start_pose = vehicle.pose.to_frame(ObjectFrameEnum.START, vehicle.pose, self.start_pose_abs)

            # Compose the 4x4 transformation matrix from the vehicle_start_pose
            T_vehicle_to_start = pose_to_matrix(vehicle_start_pose)

            # Transform the refined center (in Vehicle frame) to the START frame
            refined_center_hom_vehicle = np.append(refined_center, 1)
            refined_center_start = (T_vehicle_to_start @ refined_center_hom_vehicle)[:3]

            new_pose = ObjectPose(
                t=current_time,
                x=refined_center_start[0],
                y=refined_center_start[1],
                z=refined_center_start[2],
                yaw=yaw,
                pitch=pitch,
                roll=roll,
                frame=ObjectFrameEnum.START
            )

            existing_id = match_existing_pedestrian(
                new_center=np.array([new_pose.x, new_pose.y, new_pose.z]),
                new_dims=dims,
                existing_agents=self.tracked_agents,
                distance_threshold=2.0
            )
            if existing_id is not None:
                old_state = self.tracked_agents[existing_id]
                dt = new_pose.t - old_state.pose.t
                vx, vy, vz = compute_velocity(old_state.pose, new_pose, dt)
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

        self.current_agents = agents

        stale_ids = [agent_id for agent_id, agent in self.tracked_agents.items()
                     if current_time - agent.pose.t > 5.0]
        for agent_id in stale_ids:
            rospy.loginfo(f"Removing stale agent: {agent_id}\n")
        for agent_id, agent in agents.items():
            p = agent.pose
            # Format pose and velocity with 3 decimals (or as needed)
            rospy.loginfo(
                f"Agent ID: {agent_id}\n"
                f"Pose: (x: {p.x:.3f}, y: {p.y:.3f}, z: {p.z:.3f}, "
                f"yaw: {p.yaw:.3f}, pitch: {p.pitch:.3f}, roll: {p.roll:.3f})\n"
                f"Velocity: (vx: {agent.velocity[0]:.3f}, vy: {agent.velocity[1]:.3f}, vz: {agent.velocity[2]:.3f})\n"
    )
        return agents


# ----- Fake Pedestrian Detector 2D (for Testing Purposes) -----

class FakePedestrianDetector2D(Component):
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
    x, y, w, h = box
    pose = ObjectPose(t=0, x=x + w / 2, y=y + h / 2, z=0, yaw=0, pitch=0, roll=0, frame=ObjectFrameEnum.CURRENT)
    dims = (w, h, 0)
    return AgentState(pose=pose, dimensions=dims, outline=None,
                      type=AgentEnum.PEDESTRIAN, activity=AgentActivityEnum.MOVING,
                      velocity=(0, 0, 0), yaw_rate=0)


if __name__ == '__main__':
    pass
