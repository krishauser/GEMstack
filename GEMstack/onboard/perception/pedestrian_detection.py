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
rosbag play -l ~/rosbags/moving_vehicle_stopping_pedestrian.bag --topics /oak/rgb/camera_info /oak/rgb/image_raw /ouster/points /septentrio_gnss/imu /septentrio_gnss/insnavgeod

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
# from datetime import datetime
from copy import deepcopy
# ROS, CV
import rospy
import tf
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, PointCloud2
from visualization_msgs.msg import MarkerArray
# YOLO
from ultralytics import YOLO
from ultralytics.engine.results import Results, Boxes
# GEMStack
from ...state import AllState,VehicleState,ObjectPose,ObjectFrameEnum,AgentState,AgentEnum,AgentActivityEnum,ObjectFrameEnum
from .pedestrian_detection_utils import *
from ..interface.gem import GEMInterface, GNSSReading
from ..component import Component
from .IdTracker import IdTracker
from scipy.spatial.transform import Rotation as R


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
        self.current_agent_obj_dims = dict()
        # TODO Implement time
        self.confidence = 0.7
        self.classes_to_detect = 0
        self.ground_threshold = -2.0
        self.max_human_depth = 0.9
        self.vehicle_frame = True # Indicate whether pedestrians centroids and point clouds are in the vehicle frame

        # Load calibration data
        # TODO: Maybe lets add one word or link what R t K are?
        self.R_lidar_to_oak = load_extrinsics(os.getcwd() + '/GEMstack/onboard/perception/calibration/extrinsics/R.npy')
        self.t_lidar_to_oak = load_extrinsics(os.getcwd() + '/GEMstack/onboard/perception/calibration/extrinsics/t.npy')
        self.K = load_intrinsics(os.getcwd() + '/GEMstack/onboard/perception/calibration/camera_intrinsics.json')

        self.R_lidar_to_vehicle = np.array([[0.9995121, 0.02132941, 0.02281911], 
            [-0.02124771, 0.99976695, -0.00381707], 
            [-0.02289521, 0.00333035, 0.9997323]])
        self.t_lidar_to_vehicle = np.array([[0.0], [0.0], [0.35]])

        # To calculate vehicle to start frame data
        self.t_start_to_world = None
        self.t_vehicle_to_world = None
        self.t_vehicle_to_start = None

        # GEMStack Subscribers and sychronizers
        # LIDAR Camera fusion
        self.vehicle_interface.subscribe_sensor('sensor_fusion_Lidar_Camera',self.ouster_oak_callback)

        # TF listener to get transformation from LiDAR to Camera
        self.tf_listener = tf.TransformListener()

        if self.debug: self.init_debug()

        self.prev_time = None # Time in seconds since last scan for basic velocity calculation
        self.curr_time = None # Time in seconds of current scan for basic velocity calculation
        self.id_tracker = IdTracker()
        
        # Update function variables
        self.t_start = None # Estimated start frame time
        self.start_pose_abs = None # Get this from GNSS (GLOBAL frame)
        self.current_vehicle_state = None # Current vehicle state from GNSS in GLOBAL frame
        self.previous_vehicle_state = None # Previous vehicle state from GNSS in GLOBAL frame
    
    def init_debug(self,) -> None:
         # Debug Publishers
        self.pub_pedestrians_pc2 = rospy.Publisher("/point_cloud/pedestrians", PointCloud2, queue_size=10)
        self.pub_obj_centers_pc2 = rospy.Publisher("/point_cloud/obj_centers", PointCloud2, queue_size=10)
        self.pub_bboxes_markers = rospy.Publisher("/markers/bboxes", MarkerArray, queue_size=10)
        self.pub_image = rospy.Publisher("/camera/image_detection", Image, queue_size=1)

    def update(self, vehicle : VehicleState) -> Dict[str,AgentState]:
        # Initial vehicle pose data
        if(self.current_vehicle_state == None and self.previous_vehicle_state == None):
            self.current_vehicle_state = vehicle
            # We get vehicle state from GNSS in global state
            # Storing initial pose
            if (self.start_pose_abs == None):
                self.start_pose_abs = vehicle.pose
            if self.t_start == None:
                self.t_start = vehicle.pose.t
            return self.current_agents
        else:
            self.previous_vehicle_state = self.current_vehicle_state
            self.current_vehicle_state = vehicle

        # Update times for basic velocity calculation
        self.prev_time = self.curr_time
        self.curr_time = self.current_vehicle_state.pose.t

        # Edge Cases:

        # edge case to handle no pedestrian data
        if(self.current_agent_obj_dims == {}):
            return {}

        # edge case to handle empty lists:
        if(len(self.current_agent_obj_dims['pose']) == 0 or len(self.current_agent_obj_dims['dims']) == 0):
            return {}

        # Edge case to handle differently sized lists:
        # TODO: Handle different lengths properly
        if len(self.current_agent_obj_dims['pose']) != len(self.current_agent_obj_dims['dims']):
            raise Exception( f"Length of extracted poses ({len(self.current_agent_obj_dims['pose'])}) and dimensions ({len(self.current_agent_obj_dims['dims'])}) are not equal")
        
        # (f"Global state : {vehicle}")

        # Convert pose to start state. Need to use previous_vehicle state as pedestrian info is delayed
        vehicle_start_pose = vehicle.pose.to_frame(ObjectFrameEnum.START,self.previous_vehicle_state.pose,self.start_pose_abs)

        # converting to start frame
        # for dim, pose in zip(self.current_agent_obj_dims['dims'], self.current_agent_obj_dims['pose']):
        #     print("DIM: ", type(dim), dim)
        #     print("POSE: ", type(pose), pose)
        self.current_agent_obj_dims['pose'] = [np.array(vehicle_start_pose.apply(pose)) for pose in self.current_agent_obj_dims['pose']]
        temp_dims = list()
        for dim in self.current_agent_obj_dims['dims']:
            temp_dims.append(np.array([vehicle_start_pose.apply(corner) for corner in dim]))
        self.current_agent_obj_dims['dims'] = temp_dims 

        # Prepare variables for find_vels_and_ids
        self.prev_agents = deepcopy(self.current_agents)
        self.current_agents.clear()
        # Note this below function will probably throw a type error. I think we'll need to index the 
        # tuples by index 0 in the lists but I'm not sure:
        agents = self.create_agent_states(obj_centers=self.current_agent_obj_dims['pose'], obj_dims=self.current_agent_obj_dims['dims'])

        # Calculating the velocity of agents and tracking their ids
        backOrder = self.find_vels_and_ids(agents=agents)

        # Create a dictionary of vehicle frame agents:
        vehicle_agents = self.create_vehicle_dict(agents, backOrder)
        # Create a list containing the ids of the new agents in the vehicle frame agent's order

        return vehicle_agents
    
    def create_vehicle_dict(self, agents: List[AgentState], backOrder: List[int]):
        vehicle_agents = {}

        for idx in backOrder:
            vehicle_agents[backOrder[idx]] = agents[idx]
            vehicle_agents[backOrder[idx]].activity = self.current_agents[backOrder[idx]].activity
            vehicle_agents[backOrder[idx]].velocity = self.current_agents[backOrder[idx]].velocity

        return vehicle_agents

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
        # x, y, z 1 value 
        dims = [np.array(()) if clust.size == 0 else np.max(clust, axis= 0) - np.min(clust, axis= 0) for clust in clusters]
        for i in range(len(dims)):
            if dims[i].size == 0: continue
            else:
                # -dims[i]/2, dims[i]/2
                # 8 point bbox
                dims[i] = np.vstack([[-dims[i][0]/2.0, -dims[i][1]/2.0, -dims[i][2]/2.0],
                                    [-dims[i][0]/2.0, -dims[i][1]/2.0, dims[i][2]/2.0],
                                    [-dims[i][0]/2.0, dims[i][1]/2.0, -dims[i][2]/2.0],
                                    [-dims[i][0]/2.0, dims[i][1]/2.0, dims[i][2]/2.0],
                                    [dims[i][0]/2.0, -dims[i][1]/2.0, -dims[i][2]/2.0],
                                    [dims[i][0]/2.0, -dims[i][1]/2.0, dims[i][2]/2.0],
                                    [dims[i][0]/2.0, dims[i][1]/2.0, -dims[i][2]/2.0],
                                    [dims[i][0]/2.0, dims[i][1]/2.0, dims[i][2]/2.0]]
                                    )

        # Dims are 2 points: [ [min_x, min_y, min_z], [max_x, max_y, max_z] ]
        # Need that for transform
        return dims

    # TODO: Separate debug/viz class, bbox and 2d 3d points funcs 
    def viz_object_states(self, cv_image, boxes, extracted_pts_all):
        # Extract 3D pedestrians points in lidar frame
        # ** These are camera frame after transform_lidar_points, right?
        # ** It was in camera frame before. I fixed it. Now they are in lidar frame!
        pedestrians_3d_pts = [[] if len(extracted_pts) == 0 else list(extracted_pts[:, -3:]) for extracted_pts in extracted_pts_all] 

        # Object center viz
        obj_3d_obj_centers = list()
        obj_3d_obj_dims = list()
        for track_id, agent in self.current_agents.items():
            if agent.pose.x != None and agent.pose.y != None and agent.pose.z != None:
                obj_3d_obj_centers.append((agent.pose.x, agent.pose.y, agent.pose.z)) # **
            if agent.dimensions != None and agent.dimensions[0] != None and agent.dimensions[1] != None and agent.dimensions[2] != None:
                obj_3d_obj_dims.append(agent.dimensions)
        
        # Extract 2D pedestrians points and bbox in camera frame
        extracted_2d_pts = [[] if len(extracted_pts) == 0 else list(extracted_pts[:, :2].astype(int)) for extracted_pts in extracted_pts_all]
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
            if self.vehicle_frame:
                # If in vehicle frame, tranform 3D pedestrians points to vehicle frame for better visualization.
                flattened_pedestrians_3d_pts_vehicle = transform_lidar_points(np.array(flattened_pedestrians_3d_pts), self.R_lidar_to_vehicle, self.t_lidar_to_vehicle)
                flattened_pedestrians_3d_pts = flattened_pedestrians_3d_pts_vehicle

            

            # Create point cloud from extracted 3D points
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
        # self.prev_agents = self.current_agents.copy()
        # self.current_agents.clear()

        # self.current_agent_obj_dims.clear()
        # Return if no track results available
        if track_result[0].boxes.id == None:
            return

        # Change pedestrians_3d_pts to dicts matching track_ids

        # Extract 3D pedestrians points in lidar frame
        # ** These are camera frame after transform_lidar_points, right?
        # ** It was in camera frame before. I fixed it. Now they are in lidar frame!
        pedestrians_3d_pts = [[] if len(extracted_pts) == 0 else list(extracted_pts[:, -3:]) for extracted_pts in extracted_pts_all] 
        if len(pedestrians_3d_pts) != len(track_result[0].boxes):
            raise Exception( f"Length of extracted pedestrian clusters ({len(pedestrians_3d_pts)}) and number of camera bboxes ({len(track_result[0].boxes)}) are not equal")
        

        # TODO: Slower but cleaner to pass dicts of AgentState
        #       or at least {track_ids: centers/pts/etc}
        # TODO: Combine funcs for efficiency in C.
        #       Separate numpy prob still faster for now
        obj_centers = self.find_centers(pedestrians_3d_pts) # Centers are calculated in lidar frame here
        obj_dims = self.find_dims(pedestrians_3d_pts)   # 8 point dims of bounding box
        # Pose is stored in vehicle frame and converted to start frame in the update function 
        obj_centers_vehicle = []
        obj_dims_vehicle = []
        if self.vehicle_frame:
            for obj_center in obj_centers:
                if obj_center.size > 0:
                    obj_center = np.array([obj_center])
                    obj_center_vehicle = transform_lidar_points(obj_center, self.R_lidar_to_vehicle, self.t_lidar_to_vehicle)[0]
                    obj_centers_vehicle.append(obj_center_vehicle)
                else:
                    obj_centers_vehicle.append(np.array(()))
            obj_centers = obj_centers_vehicle

            if(len(obj_center) != 0) and (len(obj_dims) != 0):
                for obj_dim in obj_dims:
                    if len(obj_dim) > 0:
                        # obj_dim_min = np.array([obj_dim[0]])
                        # obj_dim_max = np.array([obj_dim[1]])
                        # obj_dim_min_vehicle = transform_lidar_points(obj_dim_min, self.R_lidar_to_vehicle, self.t_lidar_to_vehicle)[0]
                        # obj_dim_max_vehicle = transform_lidar_points(obj_dim_max, self.R_lidar_to_vehicle, self.t_lidar_to_vehicle)[0]
                        # # Dims are 2 points: [ [min_x, min_y, min_z], [max_x, max_y, max_z] ]
                        # obj_dims_vehicle.append(np.vstack([obj_dim_min_vehicle, obj_dim_max_vehicle]))
                        obj_dim_vehicle = transform_lidar_points(obj_dim, self.R_lidar_to_vehicle, self.t_lidar_to_vehicle)
                        obj_dims_vehicle.append(obj_dim_vehicle)
                    else: obj_dims_vehicle.append(np.array(()))
            obj_dims = obj_dims_vehicle

        self.current_agent_obj_dims["pose"] = obj_centers
        self.current_agent_obj_dims["dims"] = obj_dims


    # TODO: Refactor to make more efficient
    # TODO: Moving Average across last N iterations pos/vel? Less spurious vals
    def find_vels_and_ids(self, agents: List[AgentState]):
        # self.prev_agents was assigned a deepcopy of self.current_agents and then self.current_agents
        # was cleared before this function was called

        # Create a list containing the ids of the new agents in the vehicle frame agent's order
        backOrder = []
        for each in agents:
            backOrder.append(None)
        
        # Nothing was scanned, erase current (for current output) and previous list (for next time through because nothing was scanned)
        if (len(agents) == 0):
            self.current_agents = {}
            return

        assigned = False
        num_agents = len(agents)

        for idx in range(num_agents):
            assigned = False

            # Loop through previous agents backwards
            for prev_id, prev_state in reversed(self.prev_agents.items()):
                # If a scanned agent overlaps with a previous agent, assume that they're the same agent
                if self.agents_overlap(agents[idx], prev_state):
                    assigned = True

                    if self.prev_time == None:
                        # This will be triggered if the very first message has pedestrians in it
                        vel = [0, 0, 0]
                    else:
                        delta_t = self.curr_time - self.prev_time
                        vel = list((np.array([agents[idx].pose.x, agents[idx].pose.y, agents[idx].pose.z]) - np.array([prev_state.pose.x, prev_state.pose.y, prev_state.pose.z])) / delta_t)
                    # print("VELOCITY:")
                    # print(vel)
                    
                    # Fix start frame agent and store in this dict:
                    # TODO: Use np.isclose
                    agents[idx].activity = AgentActivityEnum.STOPPED if (np.all(vel == 0) or len(vel) == 0) else AgentActivityEnum.MOVING
                    agents[idx].velocity = (0, 0, 0) if len(vel) == 0 else tuple(vel)

                    self.current_agents[prev_id] = agents[idx]
                    backOrder[idx] = prev_id
                    del self.prev_agents[prev_id] # Remove previous agent from previous agents so it doesn't get assigned multiple times on accident
                    break
            
            if not assigned:
                # Set velocity to 0 and assign the new agent a new id with IdTracker
                id = self.id_tracker.get_new_id()
                
                # Fix start frame agent and store in this dict:
                agents[idx].activity = AgentActivityEnum.UNDETERMINED
                agents[idx].velocity = (0, 0, 0)
                self.current_agents[id] = agents[idx]
                backOrder[idx] = id
        return backOrder

    # Calculates whether 2 agents overlap. True if they do, false if not
    def agents_overlap(self, curr_agent: AgentState, prev_agent: AgentState) -> bool:
        # Calculate corners of AgentState
        # Beware: AgentState(PhysicalObject) builds bbox from 
        # dims [-l/2,l/2] x [-w/2,w/2] x [0,h], not
        # [-l/2,l/2] x [-w/2,w/2] x [-h/2,h/2]
        # TODO: confirm (z -> l, x -> w, y -> h)

        # Extract the corners of the agents:
        (x1_min, x1_max), (y1_min, y1_max), (z1_min, z1_max) = curr_agent.bounds() # Bounds of current agent
        (x2_min, x2_max), (y2_min, y2_max), (z2_min, z2_max) = prev_agent.bounds() # Bounds of previous agent

        # True if they overlap, false if not
        return (
            ( (x1_min <= x2_min and x2_min <= x1_max) or (x2_min <= x1_min and x1_min <= x2_max) ) and
            ( (y1_min <= y2_min and y2_min <= y1_max) or (y2_min <= y1_min and y1_min <= y2_max) ) and
            ( (z1_min <= z2_min and z2_min <= z1_max) or (z2_min <= z1_min and z1_min <= z2_max) )
        )

    def create_agent_states(self, obj_centers: List[np.ndarray], obj_dims: List[np.ndarray]) -> List[AgentState]:
        # Dims are 2 points: [ [min_x, min_y, min_z], [max_x, max_y, max_z] ]

        # Gate to check whether dims and centers are empty (will happen if no pedestrians are scanned):
        for idx in range(len(obj_dims) -1, -1, -1):
            # print("type:", type(obj_centers[idx]), type(obj_dims[idx]))
            if obj_centers[idx].size == 0 or obj_dims[idx].size == 0:
                del obj_centers[idx]
                del obj_dims[idx]
        if (len(obj_centers) != len(obj_dims)):
            raise Exception(f"Length of object centers ({len(obj_centers)}) and dimensions ({len(obj_dims)}) are not equal")
                
        # Convert from 2 point to 1 point dims
        # obj_dims = [np.abs( dim[0] - dim[1])  for dim in obj_dims]   
       #     elif (obj_centers[idx].size != obj_dims[0].size):
       #         del obj_centers[idx]
       #         del obj_dims[idx]
        # Create list of agent states in current vehicle frame:

        xyz_lengths_start = [np.max(obj_dim, axis=0) - np.min(obj_dim, axis=0) for obj_dim in obj_dims]
        xyz_lengths_start = [xyz_length.astype(float) for xyz_length in  xyz_lengths_start]
        agents = []
        num_pairings = len(obj_centers)
        for idx in range(num_pairings):
            # Create agent in current frame:
            agents.append(AgentState(
                            pose=ObjectPose(t=self.curr_time - self.t_start, x=obj_centers[idx][0], y=obj_centers[idx][1], z=obj_centers[idx][2] - xyz_lengths_start[idx][2]/2.0, yaw=0,pitch=0,roll=0,frame=ObjectFrameEnum.CURRENT),
                            # Beware: AgentState(PhysicalObject) builds bbox from 
                            # dims [-l/2,l/2] x [-w/2,w/2] x [0,h], not
                            # [-l/2,l/2] x [-w/2,w/2] x [-h/2,h/2]
                            # (l, w, h)
                            # TODO: confirm (z -> l, x -> w, y -> h)
                            dimensions=(xyz_lengths_start[idx][0], xyz_lengths_start[idx][1], xyz_lengths_start[idx][2]),
                            outline=None,
                            type=AgentEnum.PEDESTRIAN,
                            activity=AgentActivityEnum.UNDETERMINED, # Temporary
                            velocity=None, # Temporary
                            yaw_rate=0,
                        ))

        return agents
    

    # TODO : Generate Transformation matrix from vehicle to start
    # Just keeping this here incase we need it, at this moment this function will not be used.
    # def generate_vehicle_start_frame(self) -> np.ndarray:
    #     '''
    #     Creates T matrix for WORLD to START Frame and T matrix for WORLD to VEHICLE Frame
    #     Return T VEHICLE to START
    #     '''

    #     # T World to Start 
    #     start_x, start_y, start_z = self.current_vehicle_state.pose.x, self.current_vehicle_state.pose.y, self.current_vehicle_state.pose.z
    #     start_yaw, start_roll, start_pitch =  self.current_vehicle_state.pose.yaw, self.current_vehicle_state.pose.pitch, self.current_vehicle_state.pose.roll
    #     rotation_world_start = R.from_euler('xyz',[start_roll, start_pitch,start_yaw],degrees=False).as_matrix()
    #     self.t_start_to_world = np.eye(4)
    #     self.t_start_to_world[:3,:3] = rotation_world_start
    #     self.t_start_to_world[:3,3] = [start_x, start_y, start_z]


    #     # Converting vehicle_state from START to CURRENT
    #     self.current_vehicle_state.to_frame(frame=ObjectFrameEnum.CURRENT, current_pose=self.current_vehicle_state.pose, start_pose_abs=self.start_pose_abs)

    #     # T World to Vehicle
    #     vehicle_x, vehicle_y, vehicle_z = self.current_vehicle_state.pose.x, self.current_vehicle_state.pose.y, self.current_vehicle_state.pose.z
    #     vehicle_yaw, vehicle_roll, vehicle_pitch =  self.current_vehicle_state.pose.yaw, self.current_vehicle_state.pose.pitch, self.current_vehicle_state.pose.roll
    #     rotation_world_vehicle = R.from_euler('xyz',[vehicle_roll, vehicle_pitch, vehicle_yaw],degrees=False).as_matrix()
    #     self.t_vehicle_to_world = np.eye(4)
    #     self.t_vehicle_to_world[:3,:3] = rotation_world_vehicle
    #     self.t_vehicle_to_world[:3,3] = [vehicle_x, vehicle_y, vehicle_z]

    #     return np.linalg.inv(self.t_start_to_world @ np.linalg.inv(self.t_vehicle_to_world))


    # TODO: Slower but cleaner to input self.current_agents dict
    # TODO: Moving Average across last N iterations pos/vel? Less spurious vals
    # TODO Akul: Fix velocity calculation to calculate in ObjectFrameEnum.START
    #            Work towards own tracking class instead of simple YOLO track?
    #            Fix 1: division by time
    #            Fix 2: Put centers and dims in start frame for velocity calc +  final agentstate in update_object_states
    # ret: Dict[track_id: vel[x, y, z]]
    # def find_vels(self, track_ids: List[int], obj_centers: List[np.ndarray], obj_dims: List[np.ndarray]) -> Dict[int, np.ndarray]:
    #     # Object not seen -> velocity = None
    #     track_id_center_map = dict(zip(track_ids, obj_centers))
    #     vels = defaultdict(lambda: np.array(())) # None is faster, np.array matches other find_ methods.

    #     for prev_track_id, prev_agent in self.prev_agents.items():
    #         if prev_track_id in track_ids:
    #             # TODO: Add prev_agents to memory to avoid None velocity
    #             # We should only be missing prev pose on first sight of track_id Agent.
    #             # print("shape 1: ", track_id_center_map[prev_agent.track_id])
    #             # print("shape 2: ", np.array([prev_agent.pose.x, prev_agent.pose.y, prev_agent.pose.z]))
    #             # prev can be 3 separate Nones, current is just empty array... make this symmetrical
    #             if prev_agent.pose.x and prev_agent.pose.y and prev_agent.pose.z and track_id_center_map[prev_agent.track_id].shape == 3:
    #                 vels[prev_track_id] = (track_id_center_map[prev_track_id] - np.array([prev_agent.pose.x, prev_agent.pose.y, prev_agent.pose.z])) / (self.curr_time - self.prev_time)
    #     return vels

    def ouster_oak_callback(self, cv_image: cv2.Mat, lidar_points: np.ndarray):
        self.cv_image = cv_image
        self.lidar_points = lidar_points

        # Convert to cv2 image and run detector
        # cv_image = self.bridge.imgmsg_to_cv2(rgb_image_msg, "bgr8") 
        track_result = self.detector.track(source=cv_image, classes=self.classes_to_detect, persist=True, conf=self.confidence)

        # Convert 1D PointCloud2 data to x, y, z coords
        # lidar_points = convert_pointcloud2_to_xyz(lidar_pc2_msg)
        # print("len lidar_points", len(lidar_points))

        # Downsample xyz point clouds
        downsampled_points = downsample_points(lidar_points)
        
        # Transform LiDAR points into the camera coordinate frame.
        lidar_in_camera = transform_lidar_points(downsampled_points, self.R_lidar_to_oak, self.t_lidar_to_oak)

        # Project the transformed points into the image plane.
        projected_pts = project_points(lidar_in_camera, self.K, downsampled_points)

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
            else: extracted_pts_all.append(np.array(()))
        
        # Generate Transformation matrix for vehicle to start
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