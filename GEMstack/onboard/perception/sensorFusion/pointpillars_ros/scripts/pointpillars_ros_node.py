#!/usr/bin/env python3

## To run:
#        Create Package: catkin_create_pkg pointpillars_ros rospy std_msgs sensor_msgs geometry_msgs visualization_msgs message_filters tf2_ros cv_bridge ros_numpy
#        In launch/pointpillars_test.launch: Update Checkpoint_path and config_path. Set target_frame.
#        chmod +x ~/pointpillars_ros/scripts/pointpillars_ros_node.py
#        catkin_make
#        source ~/catkin_ws/devel/setup.bash
#        export ROS_PACKAGE_PATH=~Gem**/pointpillars_ros/../:$ROS_PACKAGE_PATH
#        roslaunch pointpillars_ros pointpillars_test.launch (Make sure roscore and your data source (e.g., rosbag play) are running).
#        Publish  K and D on CameraInfo topic, and a working TF tree providing the transform between the LiDAR frame and the camera frame (self.target_frame).
#!/usr/bin/env python3

#!/usr/bin/env python3

import rospy
import ros_numpy
import numpy as np
import torch
import cv2
import os
import sys
import time

###### ROS Message Imports
from sensor_msgs.msg import PointCloud2, Image # Removed CameraInfo
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

###### Synchronization and TF Imports
import message_filters
import tf2_ros
import tf2_geometry_msgs
from tf.transformations import quaternion_from_euler

###### Image Processing Import
from cv_bridge import CvBridge, CvBridgeError

pointpillars_repo_path = '~/PointPillars' # <<< CHANGE THIS >>>

if not os.path.isdir(pointpillars_repo_path):
    try: import rospy; rospy.logfatal(f"PointPillars repository path not found: {pointpillars_repo_path}")
    except ImportError: print(f"[FATAL] PointPillars repository path not found: {pointpillars_repo_path}")
    sys.exit(1)
sys.path.insert(0, pointpillars_repo_path)

###### Import necessary PointPillars components
try:
    from pointpillars.model import PointPillars
    from pointpillars.utils import setup_seed
except ImportError as e:
    try: import rospy; rospy.logfatal(f"Failed to import PointPillars modules from {pointpillars_repo_path}. Error: {e}")
    except ImportError: print(f"[FATAL] Failed to import PointPillars modules from {pointpillars_repo_path}. Error: {e}")
    sys.exit(1)


class PointPillarsROS:
    def __init__(self):
        rospy.init_node('pointpillars_ros_node')

        ###### Load essential parameters
        self.checkpoint_path = rospy.get_param('~checkpoint_path')
        self.config_path = rospy.get_param('~config_path') # Still potentially needed for class names etc.
        lidar_topic = rospy.get_param('~lidar_topic', '/ouster/points')
        image_topic = rospy.get_param('~image_topic', '/oak/rgb/image_raw')
        # Removed camera_info_topic parameter
        self.score_threshold = rospy.get_param('~score_threshold', 0.3)
        self.target_frame = rospy.get_param('~target_frame') # Must match child frame in TF publisher

        ###### Load Calibration K and D from Parameters
        try:
            k_list = rospy.get_param('~camera_K')
            d_list = rospy.get_param('~camera_D')

            # Validate and convert K
            if not isinstance(k_list, list) or len(k_list) != 3 or \
               not all(isinstance(row, list) and len(row) == 3 for row in k_list):
                raise ValueError("Parameter '~camera_K' must be a 3x3 list of lists.")
            self.camera_matrix = np.array(k_list, dtype=np.float64) # Use float64 for cv2

            # Validate and convert D
            if not isinstance(d_list, list):
                 raise ValueError("Parameter '~camera_D' must be a list.")
            self.dist_coeffs = np.array(d_list, dtype=np.float64) # Use float64 for cv2

            rospy.loginfo("Camera K and D matrices loaded from parameters.")
            rospy.logdebug(f"K = {self.camera_matrix.tolist()}")
            rospy.logdebug(f"D = {self.dist_coeffs.tolist()}")

        except Exception as e:
            rospy.logfatal(f"Failed to load camera calibration K/D from parameters: {e}")
            rospy.logfatal("Please ensure '~camera_K' (3x3 list) and '~camera_D' (list) parameters are set correctly in the launch file.")
            sys.exit(1)

        ###### Load PointPillars configuration (might still need for class names etc.)
        try:
            # If config isn't needed by model(), this might be simplified further
            # depending on how class names are handled for visualization
            from pointpillars.utils import load_config # Import here if needed
            self.cfg = load_config(self.config_path)
            rospy.loginfo(f"Loaded model config from {self.config_path}")
        except FileNotFoundError:
             # Config might be optional if class names handled differently
             rospy.logwarn(f"Model config file not found: {self.config_path}. Class names might not be available.")
             self.cfg = {} # Use empty dict if config optional/missing
        except Exception as e:
             rospy.logwarn(f"Error loading config {self.config_path}: {e}. Class names might not be available.")
             self.cfg = {}

        ###### Setup Seed
        setup_seed(self.cfg.get('seed', 0)) # Use seed from config or default

        ###### Define Point Cloud Ranges (from test.py)
        self.pcd_limit_range = np.array([0, -39.68, -3, 69.12, 39.68, 1], dtype=np.float32)
        self.pcd_post_limit_range = np.array([0, -40, -3, 70.4, 40, 0.0], dtype=np.float32)

        ###### Determine number of classes
        n_classes = self.cfg.get('nclasses', 3) # Get from config or default to 3
        rospy.loginfo(f"Initializing PointPillars model for {n_classes} classes.")

        ###### Setup device and load model
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.model = PointPillars(nclasses=n_classes).to(self.device)

        if not os.path.exists(self.checkpoint_path):
             rospy.logfatal(f"Checkpoint file not found: {self.checkpoint_path}")
             sys.exit(1)
        try:
            map_location = self.device if not torch.cuda.is_available() else None
            checkpoint = torch.load(self.checkpoint_path, map_location=map_location)
            state_dict = checkpoint.get('model_state_dict', checkpoint)
            self.model.load_state_dict(state_dict)
            self.model.eval()
            rospy.loginfo(f"PointPillars model loaded successfully from {self.checkpoint_path} on device {self.device}.")
        except Exception as e:
             rospy.logfatal(f"Error loading model checkpoint: {e}", exc_info=True)
             sys.exit(1)

        ###### Initialize CV Bridge
        self.cv_bridge = CvBridge()

        ###### TF Listener Setup
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        ###### Publisher Setup
        self.image_pub = rospy.Publisher("~output/image_annotated", Image, queue_size=2)
        self.marker_pub = rospy.Publisher("~output/bounding_boxes", MarkerArray, queue_size=5)

        ###### Subscriber and Synchronizer Setup
        lidar_sub = message_filters.Subscriber(lidar_topic, PointCloud2)
        image_sub = message_filters.Subscriber(image_topic, Image)
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [lidar_sub, image_sub], queue_size=15, slop=0.1)
        self.sync.registerCallback(self.callback)

        rospy.loginfo("PointPillars ROS node initialized and ready.")


    def point_range_filter(self, pts, point_range):
        ###### Filter points outside the specified range
        keep_mask = (pts[:, 0] > point_range[0]) & \
                    (pts[:, 1] > point_range[1]) & \
                    (pts[:, 2] > point_range[2]) & \
                    (pts[:, 0] < point_range[3]) & \
                    (pts[:, 1] < point_range[4]) & \
                    (pts[:, 2] < point_range[5])
        return pts[keep_mask]

    # Removed process_camera_info callback

    def get_transform(self, target_frame, source_frame, timestamp):
       ###### Get TF transform
        try:
            return self.tf_buffer.lookup_transform(target_frame, source_frame, timestamp, rospy.Duration(0.1))
        except Exception as e:
            rospy.logwarn_throttle(5.0, f"TF lookup failed from {source_frame} to {target_frame}: {e}")
            return None

    def callback(self, lidar_msg, image_msg):
        start_time = time.time()

        ######  Convert Lidar
        try:
            pc_data = ros_numpy.point_cloud2.pointcloud2_to_array(lidar_msg)
            if 'intensity' in pc_data.dtype.names:
                points = np.vstack([pc_data['x'], pc_data['y'], pc_data['z'], pc_data['intensity']]).T
            else:
                rospy.logwarn_once("Lidar 'intensity' field not found. Using 0.")
                points = np.vstack([pc_data['x'], pc_data['y'], pc_data['z'], np.zeros(len(pc_data))]).T
            points = points.astype(np.float32)
        except Exception as e:
            rospy.logerr(f"Error converting PointCloud2: {e}")
            return

        ###### Filter points by range
        points_filtered = self.point_range_filter(points, self.pcd_limit_range)
        if points_filtered.shape[0] == 0:
            rospy.logdebug("No points left after range filtering.")
            if self.marker_pub.get_num_connections() > 0: self.clear_markers(lidar_msg.header)
            if self.image_pub.get_num_connections() > 0: self.image_pub.publish(image_msg)
            return

        ###### Convert to Tensor
        pc_torch = torch.from_numpy(points_filtered).to(self.device)
        batched_pts = [pc_torch]

        ######  Run Inference
        result_dict = {}
        with torch.no_grad():
            try:
                results = self.model(batched_pts=batched_pts, mode='test')
                if isinstance(results, list) and len(results) > 0:
                     result_dict = results[0]
                else:
                     rospy.logwarn(f"Unexpected model output format: {type(results)}. Expected list.")
            except Exception as e:
                 rospy.logerr(f"Error during model inference: {e}", exc_info=True)
                 return # Exit callback on inference error

        ###### Extract Results
        det_boxes_lidar = result_dict.get('lidar_bboxes', np.array([]))
        det_labels = result_dict.get('labels', np.array([]))
        det_scores = result_dict.get('scores', np.array([]))

        ###### Apply Score Thresholding & Lidar Range Filtering
        if det_boxes_lidar.shape[0] > 0:
             score_mask = det_scores >= self.score_threshold
             det_boxes_lidar = det_boxes_lidar[score_mask]
             det_labels = det_labels[score_mask]
             det_scores = det_scores[score_mask]

             if det_boxes_lidar.shape[0] > 0:
                 center_x = det_boxes_lidar[:, 0]; center_y = det_boxes_lidar[:, 1]; center_z = det_boxes_lidar[:, 2]
                 range_mask = (center_x >= self.pcd_post_limit_range[0]) & (center_y >= self.pcd_post_limit_range[1]) & \
                              (center_z >= self.pcd_post_limit_range[2]) & (center_x < self.pcd_post_limit_range[3]) & \
                              (center_y < self.pcd_post_limit_range[4]) & (center_z < self.pcd_post_limit_range[5])
                 det_boxes_lidar = det_boxes_lidar[range_mask]
                 det_labels = det_labels[range_mask]
                 det_scores = det_scores[range_mask]

        ###### Get Transform for Visualization
        # target_frame now comes directly from param, MUST match static transform publisher child frame
        transform_lidar_to_target = self.get_transform(self.target_frame, lidar_msg.header.frame_id, lidar_msg.header.stamp)

        ###### Visualization
        num_detections = len(det_boxes_lidar)
        cv_image = None
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(image_msg, desired_encoding="bgr8")
        except CvBridgeError as e:
            rospy.logerr(f"Error converting Image: {e}")

        if num_detections > 0:
            rospy.logdebug(f"Detected {num_detections} objects after filtering.")
            if self.marker_pub.get_num_connections() > 0:
                self.publish_markers(det_boxes_lidar, lidar_msg.header, det_labels, det_scores)

            # Use K, D loaded from params for projection
            if self.image_pub.get_num_connections() > 0 and cv_image is not None:
                if self.camera_matrix is not None and self.dist_coeffs is not None and transform_lidar_to_target is not None:
                    annotated_image = self.project_and_draw_boxes_on_image(
                        cv_image.copy(), det_boxes_lidar, transform_lidar_to_target,
                        self.camera_matrix, self.dist_coeffs, det_labels, det_scores) # Pass K, D from params
                    try:
                         img_msg_out = self.cv_bridge.cv2_to_imgmsg(annotated_image, encoding="bgr8")
                         img_msg_out.header = image_msg.header
                         self.image_pub.publish(img_msg_out)
                    except CvBridgeError as e:
                         rospy.logerr(f"Error converting annotated image to message: {e}")
                else:
                     rospy.logwarn_throttle(5.0, "Cannot annotate image: Missing TF transform (K/D loaded from params).")
                     if cv_image is not None: self.image_pub.publish(image_msg) # Publish original
        else:
             rospy.logdebug("No detections passed filters.")
             if self.marker_pub.get_num_connections() > 0: self.clear_markers(lidar_msg.header)
             if self.image_pub.get_num_connections() > 0 and cv_image is not None: self.image_pub.publish(image_msg) # Publish original

        processing_time = time.time() - start_time
        rospy.logdebug(f"Processed frame in {processing_time:.4f}s ({(1.0/processing_time if processing_time > 0 else 0):.1f} Hz)")

    def project_and_draw_boxes_on_image(self, image, boxes_lidar, transform, K, D, labels=None, scores=None):
        ###### Project 3D boxes to image plane and draw using OpenCV
        if K is None or D is None or transform is None: return image
        ###### Convert transform to 4x4 matrix
        trans = transform.transform.translation; quat = transform.transform.rotation
        tf_matrix = tf.transformations.quaternion_matrix([quat.x, quat.y, quat.z, quat.w])
        tf_matrix[0, 3] = trans.x; tf_matrix[1, 3] = trans.y; tf_matrix[2, 3] = trans.z
        ###### Box corners (unit cube) and edges
        corners_norm = np.array([[ .5, .5, .5],[ .5, .5,-.5],[ .5,-.5,-.5],[ .5,-.5, .5],
                                 [-.5, .5, .5],[-.5, .5,-.5],[-.5,-.5,-.5],[-.5,-.5, .5]])
        edges = [(0,1),(1,2),(2,3),(3,0),(4,5),(5,6),(6,7),(7,4),(0,4),(1,5),(2,6),(3,7)]

        for i, box in enumerate(boxes_lidar):
            x, y, z, dx, dy, dz, yaw = box
            if dx<=0 or dy<=0 or dz<=0: continue
            ###### Calculate 8 corners in lidar frame
            Rz = np.array([[np.cos(yaw),-np.sin(yaw),0],[np.sin(yaw),np.cos(yaw),0],[0,0,1]])
            corners = corners_norm * np.array([dx, dy, dz]); corners = corners @ Rz.T; corners += np.array([x, y, z])
            ###### Transform corners to camera frame
            corners_lidar_h = np.hstack((corners, np.ones((8,1))))
            corners_cam_h = (tf_matrix @ corners_lidar_h.T).T; corners_cam = corners_cam_h[:,:3]
            ###### Keep only points in front of the camera
            front_mask = corners_cam[:, 2] > 0.1;
            if not np.all(front_mask): continue
            ###### Project to image plane using cv2.projectPoints
            try: image_points, _ = cv2.projectPoints(corners_cam, np.zeros(3), np.zeros(3), K, D); image_points = image_points.reshape(-1, 2).astype(int)
            except: continue # Ignore projection errors
            ###### Draw edges
            color = (0, 255, 0); thickness = 1
            for start_idx, end_idx in edges: cv2.line(image, tuple(image_points[start_idx]), tuple(image_points[end_idx]), color, thickness)
            ###### Add label/score text (optional)
            if labels is not None and scores is not None and i < len(labels) and i < len(scores):
                try:
                     label_name = self.cfg.get('class_names', {}).get(int(labels[i]), f"L:{int(labels[i])}") # Use cfg if available, else basic
                     score = scores[i]; text = f"{label_name}:{score:.2f}"
                     text_pos = tuple(((image_points[0] + image_points[4]) // 2) + np.array([0,-5])) # Pos above top front center
                     cv2.putText(image, text, text_pos, cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255,255,255), 1, cv2.LINE_AA)
                except: pass # Ignore text errors
        return image

    def clear_markers(self, header):
         marker_array = MarkerArray(); marker = Marker(); marker.header = header
         marker.ns = "pointpillars_boxes"; marker.id = 0; marker.action = Marker.DELETEALL
         marker_array.markers.append(marker); self.marker_pub.publish(marker_array)

    def publish_markers(self, boxes_lidar, header, labels=None, scores=None):
        marker_array = MarkerArray(); marker_id = 0; default_lifetime = rospy.Duration(0.2)
        for i, box in enumerate(boxes_lidar):
            x,y,z,dx,dy,dz,yaw = box; label_id = int(labels[i]) if labels is not None else 0; score = scores[i] if scores is not None else 1.0
            marker = Marker(); marker.header = header; marker.ns = "pointpillars_boxes"; marker.id = marker_id; marker.type = Marker.CUBE; marker.action = Marker.ADD; marker.lifetime = default_lifetime
            marker.pose.position.x=x; marker.pose.position.y=y; marker.pose.position.z=z
            q = quaternion_from_euler(0,0,yaw); marker.pose.orientation.x=q[0]; marker.pose.orientation.y=q[1]; marker.pose.orientation.z=q[2]; marker.pose.orientation.w=q[3]
            marker.scale.x=max(dx,0.01); marker.scale.y=max(dy,0.01); marker.scale.z=max(dz,0.01); marker.color.r=0.0; marker.color.g=1.0; marker.color.b=0.0; marker.color.a=0.4
            marker_array.markers.append(marker); marker_id += 1
            text_marker = Marker(); text_marker.header = header; text_marker.ns = "pointpillars_labels"; text_marker.id = marker_id; text_marker.type = Marker.TEXT_VIEW_FACING; text_marker.action = Marker.ADD; text_marker.lifetime = default_lifetime
            text_marker.pose.position.x=x; text_marker.pose.position.y=y; text_marker.pose.position.z = z + dz/2.0 + 0.3; text_marker.scale.z = 0.4; text_marker.color.r=1.0; text_marker.color.g=1.0; text_marker.color.b=1.0; text_marker.color.a=0.9
            label_name = self.cfg.get('class_names', {}).get(label_id, f"L:{label_id}") # Use cfg if available
            text_marker.text = f"{label_name} ({score:.2f})"
            marker_array.markers.append(text_marker); marker_id += 1
        if marker_array.markers: self.marker_pub.publish(marker_array)

if __name__ == '__main__':
    try: node = PointPillarsROS(); rospy.spin()
    except rospy.ROSInterruptException: rospy.loginfo("PointPillars ROS node shutting down.")
    except Exception as e: rospy.logfatal(f"Unhandled exception: {e}", exc_info=True)
    finally:
        if 'node' in locals() and hasattr(node, 'device') and node.device == torch.device("cuda"):
             try: import torch.cuda; torch.cuda.empty_cache(); rospy.loginfo("Cleared CUDA cache.")
             except: pass
        rospy.loginfo("PointPillars ROS node finished.")
