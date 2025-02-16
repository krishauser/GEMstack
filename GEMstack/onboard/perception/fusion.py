# Python
from collections import defaultdict
import os
# ROS + CV
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, PointCloud2
import rospy
import message_filters
import tf
# YOLO + GEM
from ultralytics import YOLO
from fusion_utils import *
from ...state import AgentState


class Fusion3D():
    def __init__(self):
        # Setup variables
        self.bridge = CvBridge()
        self.detector = YOLO(os.getcwd()+'/GEMstack/knowledge/detection/yolov8n.pt')
        self.current_dets_bboxes = [] 
        self.prev_agents = []         # dict{id: agent} is more efficient, but list for
        self.current_agents = []      # simplicity to match update() output to start
        self.confidence = 0.7
        self.classes_to_detect = 0
        self.ground_threshold = 1.6
        self.max_dist_percent = 0.7

        # Setup visualization variables
        self.visualization = True # Set this to true for visualization, later change to get value from sys arg
        self.label_text = "Pedestrian "
        self.font = cv2.FONT_HERSHEY_SIMPLEX
        self.font_scale = 0.5
        self.font_color = (255, 255, 255)  # White text
        self.outline_color = (0, 0, 0)  # Black outline
        self.line_type = 1
        self.text_thickness = 2 # Text thickness
        self.outline_thickness = 1  # Thickness of the text outline

        # Load calibration data
        self.R = load_extrinsics(os.getcwd() + '/GEMstack/onboard/perception/calibration/extrinsics/R.npy')
        self.t = load_extrinsics(os.getcwd() + '/GEMstack/onboard/perception/calibration/extrinsics/t.npy')
        self.K = load_intrinsics(os.getcwd() + '/GEMstack/onboard/perception/calibration/camera_intrinsics.json')

        # TF listener to get transformation from LiDAR to Camera
        # self.tf_listener = tf.TransformListener()S

        # Publishers
        self.pub_pedestrians_pc2 = rospy.Publisher("/point_cloud/pedestrians", PointCloud2, queue_size=10)
        if(self.visualization):
            self.pub_image = rospy.Publisher("/camera/image_detection", Image, queue_size=1)

    def fusion_callback(self, rgb_image_msg: Image, lidar_pc2_msg: PointCloud2):
        # Convert to cv2 image and run detector
        cv_image = self.bridge.imgmsg_to_cv2(rgb_image_msg, "bgr8") 
        track_result = self.detector.track(source=cv_image, classes=self.classes_to_detect, persist=True, conf=self.confidence)

        # Convert 1D PointCloud2 data to x, y, z coords
        lidar_points = convert_pointcloud2_to_xyz(lidar_pc2_msg)

        # Downsample xyz point clouds
        downsampled_points = downsample_points(lidar_points)
        
        # Transform LiDAR points into the camera coordinate frame.
        lidar_in_camera = transform_lidar_points(downsampled_points, self.R, self.t)
    
        # Project the transformed points into the image plane.
        projected_pts = project_points(lidar_in_camera, self.K)

        # Process bboxes
        self.current_dets_bboxes = []
        boxes = track_result[0].boxes

        # Unpacking box dimentions detected into x,y,w,h
        pedestrians_3d_pts = []
        flattened_pedestrians_2d_pts = []
        flattened_pedestrians_3d_pts = []

        for box in boxes:
            xywh = box.xywh[0].tolist()
            self.current_dets_bboxes.append(xywh)

            # Extracting projected pts
            x, y, w, h = xywh
            left_bound = int(x - w / 2)
            right_bound = int(x + w / 2)
            top_bound = int(y - h / 2)
            bottom_bound = int(y + h / 2)

            if len(projected_pts) > 0:
                pts = np.array(projected_pts)
                extracted_pts = pts[(pts[:, 0] > left_bound) &
                                    (pts[:, 0] < right_bound) &
                                    (pts[:, 1] > top_bound) &
                                    (pts[:, 1] < bottom_bound)
                                    ]
                
                #if no points extracted for this bbox, skip
                if len(extracted_pts) < 1:
                    continue
                
                # Apply ground and max distance filter to the extracted 5D points
                extracted_pts = filter_ground_points(extracted_pts, self.ground_threshold)
                extracted_pts = filter_far_points(extracted_pts)
                
                # Extract 2D pedestrians points in camera frame
                extracted_2d_pts = list(extracted_pts[:, :2].astype(int))
                flattened_pedestrians_2d_pts = flattened_pedestrians_2d_pts + extracted_2d_pts

                # Extract 3D pedestrians points in lidar frame
                # ** These are camera frame after transform_lidar_points, right?
                #    We publish lidar transform in transform.py?
                extracted_3d_pts = list(extracted_pts[:, -3:])
                pedestrians_3d_pts.append(extracted_3d_pts)
                flattened_pedestrians_3d_pts = flattened_pedestrians_3d_pts + extracted_3d_pts

            # Used for visualization
            if(self.visualization):
                cv_image = vis_2d_bbox(cv_image, xywh, box)
        
        # TODO: Refactor from calling here?
        # Fine or a little awk?
        update_object_states(track_result, flattened_pedestrians_3d_pts)
        
        if len(flattened_pedestrians_3d_pts) > 0:
            # Draw projected 2D LiDAR points on the image.
            for pt in flattened_pedestrians_2d_pts:
                cv2.circle(cv_image, pt, 2, (0, 0, 255), -1)

            # Create point cloud from extracted 3D points
            ros_extracted_pedestrian_pc2 = create_point_cloud(flattened_pedestrians_3d_pts)
            self.pub_pedestrians_pc2.publish(ros_extracted_pedestrian_pc2)

        # Used for visualization
        if(self.visualization):
            ros_img = self.bridge.cv2_to_imgmsg(cv_image, 'bgr8')
            self.pub_image.publish(ros_img)  


# if __name__ == '__main__':
#     rospy.init_node('fusion_node', anonymous=True)
#     Fusion3D()
#     while not rospy.core.is_shutdown():
#         rospy.rostime.wallsleep(0.5)

