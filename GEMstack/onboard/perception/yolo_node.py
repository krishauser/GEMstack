from pedestrian_utils import *
from combined_detection_utils import add_bounding_box
from ultralytics import YOLO
import cv2
from scipy.spatial.transform import Rotation as R
import rospy
from sensor_msgs.msg import PointCloud2, Image
from message_filters import Subscriber, ApproximateTimeSynchronizer
from cv_bridge import CvBridge
import time

# Publisher imports:
from jsk_recognition_msgs.msg import BoundingBoxArray
# from geometry_msgs.msg import Pose, Vector3


class YoloNode():
    """
    Detects Pedestrians by fusing YOLO 2D detections with LiDAR point cloud 
    data by painting the points. The painted data is converted to vehicle 
    frame and then published as a list of bounding boxes.

    Tracking is optional: set `enable_tracking=False` to disable persistent tracking
    and return only detections from the current frame.
    """

    def __init__(self):
        self.latest_image = None
        self.latest_lidar = None
        self.bridge = CvBridge()
        self.camera_name = 'front'
        self.camera_front = (self.camera_name == 'front')
        self.score_threshold = 0.4
        self.debug = True
        # self.undistort_map1 = None
        # self.undistort_map2 = None
        self.initialize()

    def initialize(self):
        """Initialize the YOLO node with camera calibration and ROS connections."""
        # # --- Determine the correct RGB topic for this camera ---
        rgb_topic_map = {
            'front': '/oak/rgb/image_raw',
            'front_right': '/camera_fr/arena_camera_node/image_raw',
            # add additional camera mappings here if needed
        }
        rgb_topic = rgb_topic_map.get(
            self.camera_name,
            f'/{self.camera_name}/rgb/image_raw'
        )

        # Initialize YOLO node
        rospy.init_node('yolo_box_publisher')
        
        # Create bounding box publisher
        self.pub = rospy.Publisher('/yolo_boxes', BoundingBoxArray, queue_size=1)
        rospy.loginfo("YOLO node initialized and waiting for messages.")

        # Set camera intrinsic parameters based on camera
        if self.camera_front:
            self.K = np.array([[684.83331299, 0., 573.37109375],
                               [0., 684.60968018, 363.70092773],
                               [0., 0., 1.]])
            self.D = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
        else:
            self.K = np.array([[1.17625545e+03, 0.00000000e+00, 9.66432645e+02],
                               [0.00000000e+00, 1.17514569e+03, 6.08580326e+02],
                               [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])
            self.D = np.array([-2.70136325e-01, 1.64393255e-01, -1.60720782e-03, 
                              -7.41246708e-05, -6.19939758e-02])

        # Transformation matrix from LiDAR to vehicle frame 
        self.T_l2v = np.array([[0.99939639, 0.02547917, 0.023615, 1.1],
                               [-0.02530848, 0.99965156, -0.00749882, 0.03773583],
                               [-0.02379784, 0.00689664, 0.999693, 1.95320223],
                               [0., 0., 0., 1.]])
                               
        # Transformation matrix from LiDAR to camera frame
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
                                  
        # Compute inverse transformation (camera to LiDAR)
        self.T_c2l = np.linalg.inv(self.T_l2c)
        self.R_c2l = self.T_c2l[:3, :3]
        self.camera_origin_in_lidar = self.T_c2l[:3, 3]

        # Initialize the YOLO detector and move to GPU if available
        self.detector = YOLO('yolov8n.pt')
        self.detector.to('cuda')

        # Subscribe to the RGB and LiDAR streams
        self.rgb_sub = Subscriber(rgb_topic, Image)
        self.lidar_sub = Subscriber('/ouster/points', PointCloud2)
        self.sync = ApproximateTimeSynchronizer([
            self.rgb_sub, self.lidar_sub
        ], queue_size=10, slop=0.1)
        self.sync.registerCallback(self.synchronized_callback)

    def synchronized_callback(self, image_msg, lidar_msg):
        """Process synchronized RGB and LiDAR messages to detect pedestrians."""
        rospy.loginfo("Received synchronized RGB and LiDAR messages")
        
        # Convert image message to OpenCV format
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
        except Exception as e:
            rospy.logerr(f"Failed to convert image: {e}")
            self.latest_image = None
            
        # Convert LiDAR message to numpy array
        self.latest_lidar = pc2_to_numpy(lidar_msg, want_rgb=False)

        # Gate guards against data not being present for both sensors:
        if self.latest_image is None or self.latest_lidar is None:
            return {} # Skip
            
        latest_image = self.latest_image.copy()

        # Optionally downsample LiDAR points
        downsample = False
        if downsample:
            lidar_down = downsample_points(self.latest_lidar, voxel_size=0.1)
        else:
            lidar_down = self.latest_lidar.copy()
        
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
            img_normal = latest_image.copy()
            undistorted_img = latest_image.copy()
            orig_H, orig_W = latest_image.shape[:2]
            self.current_K = self.K
            
        # Run YOLO detection on the image
        results_normal = self.detector(img_normal, conf=0.4, classes=[0])
        combined_boxes = []

        boxes_normal = np.array(results_normal[0].boxes.xywh.cpu()) if len(results_normal) > 0 else []
        # for box in boxes_normal:
        #     cx, cy, w, h = box
        #     combined_boxes.append((cx, cy, w, h, AgentActivityEnum.STANDING))

        start = time.time()
        # Transform the lidar points from lidar frame of reference to camera EXTRINSIC frame of reference.
        # Then project the pixels onto the lidar points to "paint them" (essentially determine which points are associated with detected objects)
        pts_cam = transform_points_l2c(lidar_down, self.T_l2c)
        projected_pts = project_points(pts_cam, self.current_K, lidar_down)
        # What is returned:
        # projected_pts[:, 0]: u-coordinate in the image (horizontal pixel position)
        # projected_pts[:, 1]: v-coordinate in the image (vertical pixel position)
        # projected_pts[:, 2:5]: original X, Y, Z coordinates in the LiDAR frame


        # Create empty list of bounding boxes to fill and publish later
        boxes = BoundingBoxArray()
        boxes.header.frame_id = 'currentVehicleFrame'
        boxes.header.stamp = lidar_msg.header.stamp

        # Process YOLO detections
        boxes_normal = np.array(results_normal[0].boxes.xywh.cpu()) if len(results_normal) > 0 else []
        conf_scores = np.array(results_normal[0].boxes.conf.cpu()) if len(results_normal) > 0 else []
    
        for i, box in enumerate(boxes_normal):
            # Skip low confidence detections
            if conf_scores[i] < self.score_threshold:
                continue
                
            # Calculate the 2D bounding box in the image
            cx, cy, w, h = box
            left = int(cx - w / 2)
            right = int(cx + w / 2)
            top = int(cy - h / 2)
            bottom = int(cy + h / 2)

            # Find LiDAR points that project to this box
            mask = ((projected_pts[:, 0] >= left) & (projected_pts[:, 0] <= right) & 
                   (projected_pts[:, 1] >= top) & (projected_pts[:, 1] <= bottom))
            roi_pts = projected_pts[mask]
            
            # Ignore regions with too few points
            if roi_pts.shape[0] < 5:
                continue

            # Get the 3D points corresponding to the box
            points_3d = roi_pts[:, 2:5]
            points_3d = filter_depth_points(points_3d, max_human_depth=0.8)
            refined_cluster = refine_cluster(points_3d, np.mean(points_3d, axis=0), eps=0.15, min_samples=10)
            refined_cluster = remove_ground_by_min_range(refined_cluster, z_range=0.1)

            if refined_cluster.shape[0] < 5:
                continue
                
            # Create a point cloud from the filtered points
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(refined_cluster)
            
            # Get an oriented bounding box
            obb = pcd.get_oriented_bounding_box()
            refined_center = obb.center
            dims = tuple(obb.extent)
            R_lidar = obb.R.copy()
            
            # We are assuming that dims[0] is height and dims[2] is length of obb.extent

            # Transform from LiDAR to vehicle coordinates
            refined_center_hom = np.append(refined_center, 1)
            refined_center_vehicle_hom = self.T_l2v @ refined_center_hom
            refined_center_vehicle = refined_center_vehicle_hom[:3]
            
            # Calculate rotation in vehicle frame
            R_vehicle = self.T_l2v[:3, :3] @ R_lidar
            # yaw, pitch, roll = R.from_matrix(R_vehicle).as_euler('zyx', degrees=False)
            yaw = np.arctan2(R_vehicle[1, 0], R_vehicle[0, 0])

            # Add the bounding box
            boxes = add_bounding_box(
                boxes=boxes, 
                frame_id='currentVehicleFrame', 
                stamp=lidar_msg.header.stamp, 
                x=refined_center_vehicle[0], 
                y=refined_center_vehicle[1], 
                z=refined_center_vehicle[2], 
                l=dims[2],  # length 
                w=dims[1],  # width 
                h=dims[0],  # height 
                yaw=yaw,
                conf_score=float(conf_scores[i]),
                label=0  # person/pedestrian class
            )
            
            rospy.loginfo(f"Person detected at ({refined_center_vehicle[0]:.2f}, "
                         f"{refined_center_vehicle[1]:.2f}, {refined_center_vehicle[2]:.2f}) "
                         f"with score {conf_scores[i]:.2f}")
        
        # Publish the bounding boxes
        rospy.loginfo(f"Publishing {len(boxes.boxes)} person bounding boxes")
        self.pub.publish(boxes)

    def undistort_image(self, image, K, D):
        """Undistort an image using the camera calibration parameters."""
        h, w = image.shape[:2]
        newK, _ = cv2.getOptimalNewCameraMatrix(K, D, (w, h), 1, (w, h))
        
        # Initialize undistortion maps if not already done
        if self.undistort_map1 is None or self.undistort_map2 is None:
            self.undistort_map1, self.undistort_map2 = cv2.initUndistortRectifyMap(K, D, R=None,
                                                                                   newCameraMatrix=newK, size=(w, h),
                                                                                   m1type=cv2.CV_32FC1)

        start = time.time()
        undistorted = cv2.remap(image, self.undistort_map1, self.undistort_map2, interpolation=cv2.INTER_NEAREST)
        end = time.time()
        # print('--------undistort', end-start)
        return undistorted, newK


if __name__ == '__main__':
    try:
        node = YoloNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
