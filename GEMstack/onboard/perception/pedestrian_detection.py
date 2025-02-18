"""
Top ouster lidar + Oak front camera fusion, object detection
"""
"""
Terminal 1:
---------------
source /opt/ros/noetic/setup.bash && source /catkin_ws/devel/setup.bash
roscore

Terminal 2:
---------------
source /opt/ros/noetic/setup.bash && source /catkin_ws/devel/setup.bash
python3 GEMstack/onboard/perception/transform.py

Terminal 3:
---------------
source /opt/ros/noetic/setup.bash && source /catkin_ws/devel/setup.bash
rosbag play -l ~/rosbags/vehicle.bag

Terminal 4:
---------------
source /opt/ros/noetic/setup.bash && source /catkin_ws/devel/setup.bash
cd GEMStack
python3 main.py --variant=detector_only launch/pedestrian_detection.yaml

Terminal 5:
---------------
source /opt/ros/noetic/setup.bash && source /catkin_ws/devel/setup.bash
rviz
"""

# Python 
import os
from typing import List, Dict
from collections import defaultdict
# ROS, CV
import rospy
import message_filters
import tf
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, PointCloud2
from visualization_msgs.msg import MarkerArray
# YOLO
from ultralytics import YOLO
from ultralytics.engine.results import Results, Boxes
# GEMStack
from ...state import AllState,VehicleState,ObjectPose,ObjectFrameEnum,AgentState,AgentEnum,AgentActivityEnum
from .pedestrian_detection_utils import *
from ..interface.gem import GEMInterface
from ..component import Component

def box_to_fake_agent(box):
    """Creates a fake agent state from an (x,y,w,h) bounding box.
    
    The location and size are pretty much meaningless since this is just giving a 2D location.
    """
    x,y,w,h = box
    pose = ObjectPose(t=0,x=x+w/2,y=y+h/2,z=0,yaw=0,pitch=0,roll=0,frame=ObjectFrameEnum.CURRENT)
    dims = (w,h,0)
    return AgentState(pose=pose,dimensions=dims,outline=None,type=AgentEnum.PEDESTRIAN,activity=AgentActivityEnum.MOVING,velocity=(0,0,0),yaw_rate=0)


class PedestrianDetector2D(Component):
    # TODO: Pull params into a JSON/yaml
    # TODO: Convert some lists into np.arrays, vectorize calculations
    # TODO: Implement logging instead of print, cleanup comments
    # TODO: Cleanup funcs + split into separate classes
    # TODO: Decide if we want to name dets "peds" or "objs"/"agents"
    #       Maybe peds for now and Agents in agent_detection.py?
    def __init__(self, vehicle_interface : GEMInterface) -> None:
        self.vehicle_interface = vehicle_interface
        # Publish debug/viz rostopics if true
        self.debug = True
        # Setup variables
        self.bridge = CvBridge()
        # TODO: Wrap detector into GEMDetector?
        self.detector = YOLO(os.getcwd()+'/GEMstack/knowledge/detection/yolov8n.pt')
        # track_id: AgentState
        self.prev_agents = dict()         
        self.current_agents = dict()
        # TODO Implement time
        self.prev_time = 0
        self.curr_time = 1 # Avoid divide by 0 for placebolder, 0
        self.confidence = 0.7
        self.classes_to_detect = 0
        self.ground_threshold = -2.0
        self.max_human_depth = 0.9

        # Load calibration data
        # TODO: Maybe lets add one word or link what R t K are?
        self.R_lidar_to_oak = load_extrinsics(os.getcwd() + '/GEMstack/onboard/perception/calibration/extrinsics/R.npy')
        self.t_lidar_to_oak = load_extrinsics(os.getcwd() + '/GEMstack/onboard/perception/calibration/extrinsics/t.npy')
        self.K = load_intrinsics(os.getcwd() + '/GEMstack/onboard/perception/calibration/camera_intrinsics.json')

        self.R_lidar_to_vehicle = np.array([[0.9995121, 0.02132941, 0.02281911], 
            [-0.02124771, 0.99976695, -0.00381707], 
            [-0.02289521, 0.00333035, 0.9997323]])
        self.t_lidar_to_vehicle = np.array([[0.0], [0.0], [0.35]])

        # Subscribers and sychronizers
        self.rgb_rosbag = message_filters.Subscriber('/oak/rgb/image_raw', Image)
        self.top_lidar_rosbag = message_filters.Subscriber('/ouster/points', PointCloud2)
        self.sync = message_filters.ApproximateTimeSynchronizer([self.rgb_rosbag, self.top_lidar_rosbag], queue_size=10, slop=0.1)
        self.sync.registerCallback(self.ouster_oak_callback)

        # TF listener to get transformation from LiDAR to Camera
        self.tf_listener = tf.TransformListener()

        if self.debug: self.init_debug()
    
    def init_debug(self,) -> None:
         # Debug Publishers
        self.pub_pedestrians_pc2 = rospy.Publisher("/point_cloud/pedestrians", PointCloud2, queue_size=10)
        self.pub_obj_centers_pc2 = rospy.Publisher("/point_cloud/obj_centers", PointCloud2, queue_size=10)
        self.pub_bboxes_markers = rospy.Publisher("/markers/bboxes", MarkerArray, queue_size=10)
        self.pub_image = rospy.Publisher("/camera/image_detection", Image, queue_size=1)

    def update(self, vehicle : VehicleState) -> Dict[str,AgentState]:
        return self.current_agents

    # TODO: Improve Algo Knn, ransac, etc.
    def find_centers(self, clusters: List[List[np.ndarray]]) -> List[np.ndarray]:
        clusters = [np.array(clust) for clust in clusters]
        centers = [np.array(()) if clust.size == 0 else np.mean(clust, axis=0) for clust in clusters]
        return centers
    
    # Beware: AgentState(PhysicalObject) builds bbox from 
    # dims [-l/2,l/2] x [-w/2,w/2] x [0,h], not
    # [-l/2,l/2] x [-w/2,w/2] x [-h/2,h/2]
    def find_dims(self, clusters: List[List[np.ndarray]]) -> List[np.ndarray]:
        # Add some small constant to height to compensate for
        # objects distance to ground we filtered from lidar, 
        # other heuristics to imrpove stability for find_ funcs ?
        clusters = [np.array(clust) for clust in clusters]
        dims = [np.array(()) if clust.size == 0 else np.max(clust, axis= 0) - np.min(clust, axis= 0) for clust in clusters]
        return dims

    # TODO: Slower but cleaner to input self.current_agents dict
    # TODO: Moving Average across last N iterations pos/vel? Less spurious vals
    # TODO Akul: Fix velocity calculation to calculate in ObjectFrameEnum.START
    #            Work towards own tracking class instead of simple YOLO track?
    #            Fix division by time
    # ret: Dict[track_id: vel[x, y, z]]
    def find_vels(self, track_ids: List[int], obj_centers: List[np.ndarray]) -> Dict[int, np.ndarray]:
        # Object not seen -> velocity = None
        track_id_center_map = dict(zip(track_ids, obj_centers))
        vels = defaultdict(lambda: np.array(())) # None is faster, np.array matches other find_ methods.

        for prev_track_id, prev_agent in self.prev_agents.items():
            if prev_track_id in track_ids:
                # TODO: Add prev_agents to memory to avoid None velocity
                # We should only be missing prev pose on first sight of track_id Agent.
                # print("shape 1: ", track_id_center_map[prev_agent.track_id])
                # print("shape 2: ", np.array([prev_agent.pose.x, prev_agent.pose.y, prev_agent.pose.z]))
                # prev can be 3 separate Nones, current is just empty array... make this symmetrical
                if prev_agent.pose.x and prev_agent.pose.y and prev_agent.pose.z and track_id_center_map[prev_agent.track_id].shape == 3:
                    vels[prev_track_id] = (track_id_center_map[prev_track_id] - np.array([prev_agent.pose.x, prev_agent.pose.y, prev_agent.pose.z])) / (self.curr_time - self.prev_time)
        return vels


    # TODO: Separate debug/viz class, bbox and 2d 3d points funcs 
    def viz_object_states(self, cv_image, boxes, extracted_pts_all):
        # Extract 3D pedestrians points in lidar frame
        # ** These are camera frame after transform_lidar_points, right?
        pedestrians_3d_pts = [list(extracted_pts[:, -3:]) for extracted_pts in extracted_pts_all]

        # Object center viz
        obj_3d_obj_centers = list()
        obj_3d_obj_dims = list()
        for track_id, agent in self.current_agents.items():
            if agent.pose.x != None and agent.pose.y != None and agent.pose.z != None:
                obj_3d_obj_centers.append((agent.pose.x, agent.pose.y, agent.pose.z)) # **
            if agent.dimensions != None and agent.dimensions[0] != None and agent.dimensions[1] != None and agent.dimensions[2] != None:
                obj_3d_obj_dims.append(agent.dimensions)
        
        # Extract 2D pedestrians points and bbox in camera frame
        extracted_2d_pts = [list(extracted_pts[:, :2].astype(int)) for extracted_pts in extracted_pts_all]
        flattened_pedestrians_2d_pts = list()
        for pts in extracted_2d_pts:    flattened_pedestrians_2d_pts.extend(pts)

        for ind, bbox in enumerate(boxes):
            xywh = bbox.xywh[0].tolist()
            cv_image = vis_2d_bbox(cv_image, xywh, bbox)
        
        flattened_pedestrians_3d_pts = list() 
        for pts in pedestrians_3d_pts: flattened_pedestrians_3d_pts.extend(pts)
	
        if len(flattened_pedestrians_3d_pts) > 0:
            # Draw projected 2D LiDAR points on the image.
            for pt in flattened_pedestrians_2d_pts:
                cv2.circle(cv_image, pt, 2, (0, 0, 255), -1)
            ros_img = self.bridge.cv2_to_imgmsg(cv_image, 'bgr8')
            self.pub_image.publish(ros_img)  

            # Draw 3D pedestrian pointclouds
            # Tranform 3D pedestrians points to vehicle frame for better visualization. Turn off for performance
            flattened_pedestrians_3d_pts_vehicle = transform_lidar_points(np.array(flattened_pedestrians_3d_pts), self.R_lidar_to_vehicle, self.t_lidar_to_vehicle)
            # Create point cloud from extracted 3D points
            # ros_extracted_pedestrian_pc2 = create_point_cloud(flattened_pedestrians_3d_pts_vehicle)
            ros_extracted_pedestrian_pc2 = create_point_cloud(flattened_pedestrians_3d_pts)
            self.pub_pedestrians_pc2.publish(ros_extracted_pedestrian_pc2)

        # Draw 3D pedestrian centers and dimensions
        if len(obj_3d_obj_centers) > 0 and len(obj_3d_obj_dims) > 0:
            # Draw 3D pedestrian center pointclouds
            ros_obj_3d_obj_centers_pc2 = create_point_cloud(obj_3d_obj_centers, color=(0, 128, 0))
            self.pub_obj_centers_pc2.publish(ros_obj_3d_obj_centers_pc2)

            # Draw 3D pedestrian bboxes markers
            # Create bbox marker from pedestrain dimensions
            ros_delete_bboxes_markers = delete_bbox_marker()
            self.pub_bboxes_markers.publish(ros_delete_bboxes_markers)
            ros_pedestrians_bboxes_markers = create_bbox_marker(obj_3d_obj_centers, obj_3d_obj_dims)
            self.pub_bboxes_markers.publish(ros_pedestrians_bboxes_markers)
        

    def update_object_states(self, track_result: List[Results], extracted_pts_all: List[np.ndarray]) -> None:
        self.prev_agents = self.current_agents.copy()
        self.current_agents.clear()

        # Return if no track results available
        if track_result[0].boxes.id == None:
            return

        # Change pedestrians_3d_pts to dicts matching track_ids
        track_ids = track_result[0].boxes.id.int().cpu().tolist()
        num_objs = len(track_ids)
        boxes = track_result[0].boxes

        # Extract 3D pedestrians points in lidar frame
        # ** These are camera frame after transform_lidar_points, right?
        pedestrians_3d_pts = [list(extracted_pts[:, -3:]) for extracted_pts in extracted_pts_all] 
        if len(pedestrians_3d_pts) != num_objs:
            raise Exception('Perception - Camera detections, points clusters num. mismatch')
        
        # TODO: Slower but cleaner to pass dicts of AgentState
        #       or at least {track_ids: centers/pts/etc}
        # TODO: Combine funcs for efficiency in C.
        #       Separate numpy prob still faster for now
        obj_centers = self.find_centers(pedestrians_3d_pts)
        obj_dims = self.find_dims(pedestrians_3d_pts)
        obj_vels = self.find_vels(track_ids, obj_centers)

        # Transform centers from top_lidar frame to vehicle frame
        # Need to transform the center point one by one since matrix op can't deal with empty points
        obj_centers_vehicle = []
        for obj_center in obj_centers:
            if len(obj_center) > 0:
                obj_center = np.array([obj_center])
                obj_center_vehicle = transform_lidar_points(obj_center, self.R_lidar_to_vehicle, self.t_lidar_to_vehicle)[0]
                obj_centers_vehicle.append(obj_center_vehicle)
            else:
                obj_centers_vehicle.append(np.array(()))
        
        # Update Current AgentStates
        for ind in range(num_objs):
            obj_center = (None, None, None) if obj_centers_vehicle[ind].size == 0 else obj_centers_vehicle[ind]
            obj_dim = (None, None, None) if obj_dims[ind].size == 0 else obj_dims[ind]
            self.current_agents[track_ids[ind]] = (
                AgentState(
                    track_id = track_ids[ind],
                    pose=ObjectPose(t=0, x=obj_center[0], y=obj_center[1], z=obj_center[2] ,yaw=0,pitch=0,roll=0,frame=ObjectFrameEnum.CURRENT),
                    # (l, w, h)
                    # TODO: confirm (z -> l, x -> w, y -> h)
                    dimensions=(obj_dim[0], obj_dim[1], obj_dim[2]),  
                    outline=None,
                    type=AgentEnum.PEDESTRIAN,
                    activity=AgentActivityEnum.MOVING,
                    velocity= None if obj_vels[track_ids[ind]].size == 0 else tuple(obj_vels[track_ids[ind]]),
                    yaw_rate=0
                ))

    def ouster_oak_callback(self, rgb_image_msg: Image, lidar_pc2_msg: PointCloud2):
        # Convert to cv2 image and run detector
        cv_image = self.bridge.imgmsg_to_cv2(rgb_image_msg, "bgr8") 
        track_result = self.detector.track(source=cv_image, classes=self.classes_to_detect, persist=True, conf=self.confidence)

        # Convert 1D PointCloud2 data to x, y, z coords
        lidar_points = convert_pointcloud2_to_xyz(lidar_pc2_msg)
        # print("len lidar_points", len(lidar_points))

        # Downsample xyz point clouds
        downsampled_points = downsample_points(lidar_points)
        # print("len downsampled_points", len(downsampled_points))
        
        # Transform LiDAR points into the camera coordinate frame.
        lidar_in_camera = transform_lidar_points(downsampled_points, self.R_lidar_to_oak, self.t_lidar_to_oak)
        # print("len lidar_in_camera", len(lidar_in_camera))

        # Project the transformed points into the image plane.
        projected_pts = project_points(lidar_in_camera, self.K, downsampled_points)
        # print("projected_pts", len(projected_pts))

        # Process bboxes
        boxes = track_result[0].boxes

        extracted_pts_all = list()

        for ind, bbox in enumerate(boxes):
            xywh = bbox.xywh[0].tolist()

            # Extracting projected pts
            x, y, w, h = xywh
            left_bound = int(x - w / 2)
            right_bound = int(x + w / 2)
            top_bound = int(y - h / 2)
            bottom_bound = int(y + h / 2)

            pts = np.array(projected_pts)
            # Checking projected_pts length is very important
            if len(projected_pts) > 0:
                extracted_pts = pts[(pts[:, 0] > left_bound) &
                                    (pts[:, 0] < right_bound) &
                                    (pts[:, 1] > top_bound) &
                                    (pts[:, 1] < bottom_bound)
                                    ]

                # Apply ground and max distance filter to the extracted 5D points
                extracted_pts = filter_ground_points(extracted_pts, self.ground_threshold)
                extracted_pts = filter_depth_points(extracted_pts, self.max_human_depth)
                extracted_pts_all.append(extracted_pts)
	    else: extracted_pts_all.append(np.array(())
        
        #if len(extracted_pts_all) > 0 and len(track_result) > 0:
        self.update_object_states(track_result, extracted_pts_all)
        if self.debug: self.viz_object_states(cv_image, boxes, extracted_pts_all)


    def rate(self):
        return 4.0
    
    def state_inputs(self):
        return ['vehicle']
    
    def state_outputs(self):
        return ['agents']


class FakePedestrianDetector2D(Component):
    """Triggers a pedestrian detection at some random time ranges"""
    def __init__(self,vehicle_interface : GEMInterface):
        self.vehicle_interface = vehicle_interface
        self.times = [(5.0,20.0),(30.0,35.0)]
        self.t_start = None

    def rate(self):
        return 4.0
    
    def state_inputs(self):
        return ['vehicle']
    
    def state_outputs(self):
        return ['agents']
    
    def update(self, vehicle : VehicleState) -> Dict[str,AgentState]:
        if self.t_start is None:
            self.t_start = self.vehicle_interface.time()
        t = self.vehicle_interface.time() - self.t_start
        res = {}
        for times in self.times:
            if t >= times[0] and t <= times[1]:
                res['pedestrian0'] = box_to_fake_agent((0,0,0,0))
                print("Detected a pedestrian")
        return res
