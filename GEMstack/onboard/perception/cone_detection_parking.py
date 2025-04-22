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
from sklearn.cluster import DBSCAN
from scipy.spatial import ConvexHull
from .parking_utils import *
from .visualization_utils import *


class ConeDetector3D(Component):
    def __init__(self):
        # Init Variables
        self.ground_threshold = -0.15
        self.vis_2d_annotate = False
        self.vis_lidar_pc = True
        self.vis_3d_cones_centers = True

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


    # Main sensors callback
    def callback(self, img_msg, lidar_msg):
        # Convert data
        image = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
        undistorted_img, K = self.undistort_image(image)
        lidar = self.pc2_to_numpy(lidar_msg)

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
            if roi_pts.shape[0] < 5:
                continue
            roi_pts = self.remove_ground(roi_pts)
            if roi_pts.shape[0] < 3:
                continue

            # Extract the LiDAR 3D points corresponding to the ROI
            roi_pts = self.filter_depth_points(roi_pts, max_depth_diff=0.2)

            # Compute center
            center = np.mean(roi_pts, axis=0)
            centroids_lidar_frame.append(center)

        if len(centroids_lidar_frame) > 0:
            # Transform centroids to vehicle frame
            centroids_vehicle_frame = self.transform_points(np.array(centroids_lidar_frame), self.T_l2v)

            # Detect parking spot if 4 or more cones are detected
            ordered_cone_ground_centers_2D = []
            closest_parking_spot = None

            all_candidates = []
            if len(centroids_vehicle_frame) % 4 == 0:
                cone_2d = np.array([[p[0], p[1]] for p in centroids_vehicle_frame], dtype=np.float32)
                all_candidates, closest_parking_spot = detect_parking_spots_and_select_closest(cone_2d, vehicle_position=(0.0, 0.0))
                ordered_cone_ground_centers_2D = cone_2d


            # Visualization
            self.viz_object_states(centroids_vehicle_frame,
                       ordered_cone_ground_centers_2D, 
                       closest_parking_spot,
                       all_candidates,
                       image, bboxes, lidar)


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

    def transform_points(self, points, transform):
        ones_column = np.ones((points.shape[0], 1))
        points_extended = np.hstack((points, ones_column))
        points_transformed = ((transform @ (points_extended.T)).T)
        return points_transformed[:, :3]

    def transform_points_l2c(self, lidar_points, T_l2c):
        N = lidar_points.shape[0]
        pts_hom = np.hstack((lidar_points, np.ones((N, 1))))  # (N,4)
        pts_cam = (T_l2c @ pts_hom.T).T  # (N,4)
        return pts_cam[:, :3]

    def filter_ground_points(self, lidar_points, ground_threshold = 0):
        filtered_array = lidar_points[lidar_points[:, 2] > ground_threshold]
        return filtered_array

    def order_points_convex_hull(self, points_2d):
        points_np = np.array(points_2d)
        hull = ConvexHull(points_np)
        ordered = [points_np[i] for i in hull.vertices]
        return ordered

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
    
    def viz_object_states(self, 
                      cone_3d_centers, 
                      ordered_cone_ground_centers_2D,
                      closest_parking_spot,
                      all_parking_candidates,
                      cv_image, boxes, 
                      lidar_ouster_frame):

        # Transform top lidar pointclouds to vehicle frame for visualization
        if self.vis_lidar_pc:
            latest_lidar_vehicle = self.transform_points(lidar_ouster_frame, self.T_l2v)
            latest_lidar_vehicle = self.filter_ground_points(latest_lidar_vehicle, self.ground_threshold)
            ros_lidar_top_vehicle_pc2 = create_point_cloud(latest_lidar_vehicle, (255, 0, 0), "vehicle")
            self.pub_lidar_top_vehicle_pc2.publish(ros_lidar_top_vehicle_pc2)

        # Create vehicle marker
        ros_vehicle_marker = create_bbox_marker([[0.0, 0.0, 0.0]], [[0.8, 0.5, 0.3]], (0.0, 0.0, 1.0, 1), "vehicle")
        self.pub_vehicle_marker.publish(ros_vehicle_marker)

        # Delete previous markers
        ros_delete_polygon_marker = delete_markers("polygon", 1)
        self.pub_polygon_marker.publish(ros_delete_polygon_marker)
        ros_delete_parking_spot_markers = delete_markers("parking_spot", 1)
        self.pub_parking_spot_marker.publish(ros_delete_parking_spot_markers)

        # Draw polygon first
        if len(ordered_cone_ground_centers_2D) > 0:
            ros_polygon_marker = create_polygon_marker(ordered_cone_ground_centers_2D, ref_frame="vehicle")
            self.pub_polygon_marker.publish(ros_polygon_marker)

        # Create parking spot marker
        if closest_parking_spot:
            ros_parking_spot_marker = create_parking_spot_marker(closest_parking_spot, ref_frame="vehicle")
            self.pub_parking_spot_marker.publish(ros_parking_spot_marker)

        # Draw 2D bboxes
        if self.vis_2d_annotate:
            for ind, bbox in enumerate(boxes):
                xywh = bbox.xywh[0].tolist()
                cv_image = vis_2d_bbox(cv_image, xywh, bbox)
            ros_img = self.bridge.cv2_to_imgmsg(cv_image, 'bgr8')
            self.pub_cones_image_detection.publish(ros_img)  

        # Draw 3D cone centers
        if len(cone_3d_centers) > 0:
            if self.vis_3d_cones_centers:
                cone_ground_centers = np.array(cone_3d_centers)
                cone_ground_centers[:, 2] = 0.0
                cone_ground_centers = [tuple(point) for point in cone_ground_centers]
                ros_cones_centers_pc2 = create_point_cloud(cone_ground_centers, color=(255, 0, 255))
                self.pub_cones_centers_pc2.publish(ros_cones_centers_pc2)

        # If there are multiple slots
        if len(all_parking_candidates) > 0:
            for i, pose in enumerate(all_parking_candidates):
                # Show candidate poses in a lighter color
                ros_candidate_marker = create_parking_spot_marker(pose, ref_frame="vehicle", id_offset=100 + i, color=(0.0, 1.0, 0.0, 0.3))
                self.pub_parking_spot_marker.publish(ros_candidate_marker)

    def detect_parking_spot(self, cone_3d_centers):
        closest_parking_spot = None
        cone_ground_centers = np.array(cone_3d_centers)
        cone_ground_centers_2D = cone_ground_centers[:, :2]
        ordered_cone_ground_centers_2D = self.order_points_convex_hull(cone_ground_centers_2D)
        # print(f"-----cone_ground_centers_2D: {cone_ground_centers_2D}")
        candidates = find_all_candidate_parking_spots(ordered_cone_ground_centers_2D)
        # print(f"-----candidates: {candidates}")
        if len(candidates) > 0:
            closest_parking_spot = select_best_candidate(candidates, ordered_cone_ground_centers_2D)
            # print(f"-----closest_parking_spot: {closest_parking_spot}")
        return ordered_cone_ground_centers_2D, closest_parking_spot


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