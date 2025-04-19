import rospy
import numpy as np
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import PointCloud2, Image
import sensor_msgs.point_cloud2 as pc2
import ros_numpy
from ultralytics import YOLO
from message_filters import Subscriber, ApproximateTimeSynchronizer
import open3d as o3d
from ..component import Component 
from ...state import VehicleState, AgentState
from .parking_utils import *


class ConeDetector3D(Component):
    def __init__(self):
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

        self.pub_centroids = rospy.Publisher("/cones_detection/centroids", PointCloud2, queue_size=10)

        self.rgb_sub = Subscriber("/camera_fr/arena_camera_node/image_raw", Image)
        self.lidar_sub = Subscriber("/ouster/points", PointCloud2)
        self.sync = ApproximateTimeSynchronizer([self.rgb_sub, self.lidar_sub], queue_size=10, slop=0.1)
        self.sync.registerCallback(self.callback)


    def detect_parking_spot(self, cone_3d_centers):
         cone_ground_centers = np.array(cone_3d_centers)
         cone_ground_centers_2D = cone_ground_centers[:, :2]
         # print(f"-----cone_ground_centers_2D: {cone_ground_centers_2D}")
         candidates = findAllCandidateParkingLot(cone_ground_centers_2D)
         # print(f"-----candidates: {candidates}")
         if len(candidates) > 0:
             closest_spot = candidates[0]
             # print(f"-----closest_spot: {closest_spot}")
             # Create parking spot marker
            #  ros_parking_spot_marker = create_parking_spot_marker(closest_spot, ref_frame="vehicle")
            #  self.pub_parking_spot_marker.publish(ros_parking_spot_marker)
         return

    def callback(self, img_msg, lidar_msg):
        # Convert data
        image = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
        undistorted_img, K = self.undistort_image(image)
        lidar = self.pc2_to_numpy(lidar_msg)

        # Detect cones
        results = self.detector(undistorted_img, conf=0.3, classes=[0])
        if not results or results[0].boxes is None:
            return

        boxes = results[0].boxes.xywh.cpu().numpy()
        lidar_cam = self.transform_lidar_to_camera(lidar)
        projections = self.project_points(lidar_cam, lidar, K)

        # Collect cone centroids
        centroids = []
        for cx, cy, w, h in boxes:
            u_min, u_max = int(cx - w/2), int(cx + w/2)
            v_min, v_max = int(cy - h/2), int(cy + h/2)
            mask = (projections[:, 0] >= u_min) & (projections[:, 0] <= u_max) & \
                   (projections[:, 1] >= v_min) & (projections[:, 1] <= v_max)
            region_pts = projections[mask][:, 2:5]
            if region_pts.shape[0] < 5:
                continue
            region_pts = self.remove_ground(region_pts)
            if region_pts.shape[0] < 3:
                continue
            center = np.mean(region_pts, axis=0)
            center[2] = np.min(region_pts[:, 2]) + 0.15  # z offset to get cone center
            centroids.append(center)

        # Publish centroids
        if centroids:
            self.pub_centroids.publish(self.create_pc2(centroids))

        # Parking space detection logic here



    def undistort_image(self, img):
        h, w = img.shape[:2]
        new_K, _ = cv2.getOptimalNewCameraMatrix(self.K, self.D, (w, h), 1)
        return cv2.undistort(img, self.K, self.D, None, new_K), new_K

    def pc2_to_numpy(self, pc2_msg, want_rgb=False):
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

    def transform_lidar_to_camera(self, pts):
        N = pts.shape[0]
        pts_hom = np.hstack([pts, np.ones((N, 1))])
        return (self.T_l2c @ pts_hom.T).T[:, :3]

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

    def update(self, vehicle: VehicleState) -> dict:
        # return dictionary with one key: 'agents'
        return {'agents': {}}  # or real AgentState dict


if __name__ == "__main__":
    node = ConeDetector3D()
    node.spin()
