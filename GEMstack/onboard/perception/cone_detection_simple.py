import rospy
import numpy as np
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import PointCloud2, Image
import sensor_msgs.point_cloud2 as pc2
from typing import Dict
from ultralytics import YOLO
from message_filters import Subscriber, ApproximateTimeSynchronizer
from ..component import Component 
from ...state import VehicleState, AgentState, ObjectPose, ObjectFrameEnum, AgentState, AgentEnum, AgentActivityEnum
from ..interface.gem import GEMInterface
from scipy.spatial import ConvexHull
from .utils.detection_utils import *
from .utils.parking_utils import *
from .utils.visualization_utils import *


class ConeDetectorSimple3D(Component):
    def __init__(self, vehicle_interface: GEMInterface):
        # Init Variables
        self.vehicle_interface = vehicle_interface
        self.ground_threshold = -0.15
        self.centroids_vehicle_frame = []

        # Params
        self.bridge = CvBridge()
        self.detector = YOLO('./GEMstack/knowledge/detection/cone.pt')
        self.detector.to('cuda')
        self.K = np.array([[1.17625545e+03, 0, 9.66432645e+02],
                           [0, 1.17514569e+03, 6.08580326e+02],
                           [0, 0, 1]])
        self.D = np.array([-0.2701, 0.1643, -0.0016, -0.00007, -0.0619])

        self.T_l2c = np.array([[-0.7183, -0.6953, -0.0235, 0.0572],
                               [-0.0972, 0.1337, -0.9862, -0.1598],
                               [0.6888, -0.7062, -0.1636, -1.0477],
                               [0, 0, 0, 1]])
        self.T_c2l = np.linalg.inv(self.T_l2c)

        self.T_l2v = np.array([[0.99939639, 0.02547917, 0.023615, 1.1],
                               [-0.02530848, 0.99965156, -0.00749882, 0.03773583],
                               [-0.02379784, 0.00689664, 0.999693, 1.95320223],
                               [0., 0., 0., 1.]])

        # Subscribers
        self.rgb_sub = Subscriber("/camera_fr/arena_camera_node/image_raw", Image)
        self.lidar_sub = Subscriber("/ouster/points", PointCloud2)
        self.sync = ApproximateTimeSynchronizer([self.rgb_sub, self.lidar_sub], queue_size=10, slop=0.1)
        self.sync.registerCallback(self.callback)

        # Publishers
        self.pub_lidar_top_vehicle_pc2 = rospy.Publisher("lidar_top_vehicle/point_cloud", PointCloud2, queue_size=10)
        self.pub_vehicle_marker = rospy.Publisher("vehicle/marker", MarkerArray, queue_size=10)
        self.pub_cones_image_detection = rospy.Publisher("cones_detection/annotated_image", Image, queue_size=1)
        self.pub_cones_centers_pc2 = rospy.Publisher("cones_detection/centers/point_cloud", PointCloud2, queue_size=10)
        self.pub_parking_spot_marker = rospy.Publisher("parking_spot_detection/marker", MarkerArray, queue_size=10)
        self.pub_polygon_marker = rospy.Publisher("polygon_detection/marker", MarkerArray, queue_size=10)
        self.pub_obstacles_marker = rospy.Publisher("obstacle_detection/marker", MarkerArray, queue_size=10)


    # Main sensors callback
    def callback(self, img_msg, lidar_msg):
        # Convert data
        image = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
        undistorted_img, K = self.undistort_image(image)
        lidar = pc2_to_numpy(lidar_msg)

        # Detect cones
        results = self.detector(undistorted_img, conf=0.3, classes=[0])
        if not results or results[0].boxes is None:
            return

        bboxes = results[0].boxes
        boxes = np.array(bboxes.xywh.cpu())
        pts_cam = self.transform_points_l2c(lidar, self.T_l2c)
        projected_pts = self.project_points(pts_cam, lidar, K)

        # Collect cone centroids
        centroids_lidar_frame = []
        for cx, cy, w, h in boxes:
            u_min, u_max = int(cx - w/2), int(cx + w/2)
            v_min, v_max = int(cy - h/2), int(cy + h/2)
            mask = (projected_pts[:, 0] >= u_min) & (projected_pts[:, 0] <= u_max) & \
                   (projected_pts[:, 1] >= v_min) & (projected_pts[:, 1] <= v_max)
            roi_pts = projected_pts[mask][:, 2:5]
            if roi_pts.shape[0] < 25:
                continue
            roi_pts = self.remove_ground(roi_pts)
            if roi_pts.shape[0] < 10:
                continue

            # Extract the LiDAR 3D points corresponding to the ROI
            roi_pts = self.filter_depth_points(roi_pts, max_depth_diff=0.2)

            # Compute center
            center = np.mean(roi_pts, axis=0)
            centroids_lidar_frame.append(center)

        if len(centroids_lidar_frame) > 0:
            # Transform centroids to vehicle frame
            self.centroids_vehicle_frame = transform_points(np.array(centroids_lidar_frame), self.T_l2v)


    # All local helper functions
    def filter_depth_points(self, lidar_points, max_depth_diff=0.9, use_norm=True):
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

    def transform_points_l2c(self, lidar_points, T_l2c):
        N = lidar_points.shape[0]
        pts_hom = np.hstack((lidar_points, np.ones((N, 1))))  # (N,4)
        pts_cam = (T_l2c @ pts_hom.T).T  # (N,4)
        return pts_cam[:, :3]

    def order_points_convex_hull(self, points_2d):
        points_np = np.array(points_2d)
        hull = ConvexHull(points_np)
        ordered = [points_np[i] for i in hull.vertices]
        return ordered

    def undistort_image(self, img):
        h, w = img.shape[:2]
        new_K, _ = cv2.getOptimalNewCameraMatrix(self.K, self.D, (w, h), 1)
        return cv2.undistort(img, self.K, self.D, None, new_K), new_K

    def project_points(self, cam_pts, lidar_pts, K):
        mask = cam_pts[:, 2] > 0
        cam_pts = cam_pts[mask]
        lidar_pts = lidar_pts[mask]
        x = (K[0, 0] * (cam_pts[:, 0] / cam_pts[:, 2]) + K[0, 2]).astype(np.int32)
        y = (K[1, 1] * (cam_pts[:, 1] / cam_pts[:, 2]) + K[1, 2]).astype(np.int32)
        return np.column_stack((x, y, lidar_pts))

    def remove_ground(self, pts, z_thresh=0.05):
        min_z = np.min(pts[:, 2])
        return pts[pts[:, 2] > min_z + z_thresh]

    def create_pc2(self, points, frame_id="os_sensor"):
        header = rospy.Header(stamp=rospy.Time.now(), frame_id=frame_id)
        fields = [
            pc2.PointField('x', 0, pc2.PointField.FLOAT32, 1),
            pc2.PointField('y', 4, pc2.PointField.FLOAT32, 1),
            pc2.PointField('z', 8, pc2.PointField.FLOAT32, 1)
        ]
        return pc2.create_cloud(header, fields, [(p[0], p[1], p[2]) for p in points])

    def spin(self):
        rospy.spin()

    def rate(self) -> float:
        return 10.0  # Hz

    def state_inputs(self) -> list:
        return ['vehicle']

    def state_outputs(self) -> list:
        return ['agents']

    def update(self, vehicle: VehicleState) -> Dict[str, AgentState]:
        current_time = self.vehicle_interface.time()
        
        # Constructing agent states
        agent_id = 0
        agents = {}
        for cone_centroid in self.centroids_vehicle_frame:
            x, y, z = cone_centroid
            new_pose = ObjectPose(
                                t=current_time,
                                x=x,
                                y=y,
                                z=z,
                                yaw=0.0,
                                pitch=0.0,
                                roll=0.0,
                                frame=ObjectFrameEnum.CURRENT
                            )
            new_agent = AgentState(
                                pose=new_pose,
                                dimensions=[0.1, 0.1, 0.3],
                                outline=None,
                                type=AgentEnum.CONE,
                                activity=AgentActivityEnum.STOPPED,
                                velocity=(0, 0, 0),
                                yaw_rate=0
                            )
            agents[agent_id] = new_agent
            agent_id += 1
        
        new_state = agents
        return new_state

if __name__ == "__main__":
    node = ConeDetectorSimple3D()
    node.spin()