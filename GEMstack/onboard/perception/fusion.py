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

class Fusion3D():
    def __init__(self):
        # Setup variables
        self.bridge = CvBridge()
        self.detector = YOLO(os.getcwd()+'/GEMstack/knowledge/detection/yolov8n.pt')
        self.current_dets_bboxes = [] 
        self.prev_agents = []         # dict{id: agent} is more efficient, but list for
        self.current_agents = []      # simplicity to match update() output to start
        self.visualization = True
        self.confidence = 0.7
        self.classes_to_detect = 0
        self.ground_threshold = 1.6
        self.max_dist_percent = 0.7

        # Load calibration data
        self.R = load_extrinsics(os.getcwd() + '/GEMstack/onboard/perception/calibration/extrinsics/R.npy')
        self.t = load_extrinsics(os.getcwd() + '/GEMstack/onboard/perception/calibration/extrinsics/t.npy')
        self.K = load_intrinsics(os.getcwd() + '/GEMstack/onboard/perception/calibration/camera_intrinsics.json')

        # Subscribers and sychronizers
        self.rgb_rosbag = message_filters.Subscriber('/oak/rgb/image_raw', Image)
        self.top_lidar_rosbag = message_filters.Subscriber('/ouster/points', PointCloud2)
        self.sync = message_filters.ApproximateTimeSynchronizer([self.rgb_rosbag, self.top_lidar_rosbag], queue_size=10, slop=0.1)
        self.sync.registerCallback(self.fusion_callback)

        # TF listener to get transformation from LiDAR to Camera
        self.tf_listener = tf.TransformListener()

        # Publishers
        self.pub_pedestrians_pc2 = rospy.Publisher("/point_cloud/pedestrians", PointCloud2, queue_size=10)
        if(self.visualization):
            self.pub_image = rospy.Publisher("/camera/image_detection", Image, queue_size=1)

    # TODO: refactor into pedestrian_detection.py
    def update(self, vehicle : VehicleState) -> Dict[str,AgentState]:
        self.prev_agents = self.current_agents
        return self.prev_agents

    # clusters: list[ np.array(shape = (N, XYZ) ]
    # TODO: Improve Algo Knn, ransac, etc.
    def find_centers(clusters: list[np.ndarray]):
        centers = [np.mean(clust, axis=0) for clust in clusters]
        return centers
    
    # Beware: AgentState(PhysicalObject) builds bbox from 
    # dims [-l/2,l/2] x [-w/2,w/2] x [0,h], not
    # [-l/2,l/2] x [-w/2,w/2] x [-h/2,h/2]
    def find_dims(clusters):
        # Add some small constant to height to compensate for
        # objects distance to ground we filtered from lidar, etc.?
        dims = [np.max(clust, axis= 0) - np.min(clust, axis= 0) for clust in clusters]
        return dims

    # track_result:                 self.detector.track (YOLO) output
    # flattened_pedestrians_3d_pts: Camera frame points, ind 
    #                               corresponds to object
    # TODO: Finish adapt this func + fusion.py to multiple classes
    def update_object_states(track_result, flattened_pedestrians_3d_pts: list[np.ndarray]):
        track_ids = track_result[0].boxes.id.int().cpu().tolist()
        num_objs = len(track_ids)
        
        if len(flattened_pedestrians_3d_pts) != num_objs:
            raise Exception('Perception - Camera detections, points num. mismatch')
        
        # Combine funcs for efficiency in C
        # Separate numpy prob still faster
        obj_centers = find_centers(flattened_pedestrians_3d_pts)
        obj_dims = find_dims(flattened_pedestrians_3d_pts)

        # Calculate new velocities
        # TODO: Improve velocity algo + remove O(n) dict
        # Moving Average across last N iterations pos/vel?
        track_id_center_map = dict(zip(track_ids, obj_centers))
        # Object not seen -> velocity = None
        vels = defaultdict(None)
        for prev_agent in self.prev_agents:
            if prev_agent.track_id in track_ids:
                vels[track_id] = track_id_center_map[track_id] - np.array([prev_agent.pose.x, prev_agent.pose.y, prev_agent.pose.z])

        # Update Current AgentStates
        self.current_agents = [
            AgentState(
                track_id = track_ids[ind]
                pose=ObjectPose(t=0,obj_centers[0],obj_centers[1],obj_centers[2],yaw=0,pitch=0,roll=0,frame=ObjectFrameEnum.CURRENT),
                # (l, w, h)
                # TODO: confirm (z -> l, x -> w, y -> h)
                dimensions=tuple(obj_dims[ind][2], obj_dims[ind][0], obj_dims[ind][1]),  
                outline=None,
                type=AgentEnum.PEDESTRIAN,
                activity=AgentActivityEnum.MOVING,
                velocity=tuple(vels[track_ids[ind]]),
                yaw_rate=0
            )
            for ind in len(num_objs)
        ]
    

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



if __name__ == '__main__':
    rospy.init_node('fusion_node', anonymous=True)
    Fusion3D()
    while not rospy.core.is_shutdown():
        rospy.rostime.wallsleep(0.5)

