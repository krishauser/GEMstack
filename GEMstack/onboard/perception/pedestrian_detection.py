from ...state import AllState,VehicleState,ObjectPose,ObjectFrameEnum,AgentState,AgentEnum,AgentActivityEnum #, AllState
from ..interface.gem import GEMInterface
from ..component import Component
from ultralytics import YOLO
import cv2
from typing import Dict
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, PointCloud2
import sensor_msgs.point_cloud2 as pc2
import rospy
import os
from typing import Union
from fusion import Fusion3D


class PedestrianDetector2DShared(Component):
    """
    Detects pedestrians.
    self.vehicle_interface -> GEM car info
    """
    def __init__(self,vehicle_interface : GEMInterface):
        self.vehicle_interface = vehicle_interface

    def rate(self):
        return 4.0

    def state_inputs(self):
        return ['vehicle']

    def state_outputs(self):
        return ['agents']

    def box_to_fake_agent(box) -> AgentState:
        """Creates a fake agent state from an (x,fy,w,h) bounding box.
        
        The location and size are pretty much meaningless since this is just giving a 2D location.
        """
        x,y,w,h = box
        pose = ObjectPose(t=0,x=x+w/2,y=y+h/2,z=0,yaw=0,pitch=0,roll=0,frame=ObjectFrameEnum.CURRENT)
        dims = (w,h,0)
        return AgentState(pose=pose,dimensions=dims,outline=None,type=AgentEnum.PEDESTRIAN,activity=AgentActivityEnum.MOVING,velocity=(0,0,0),yaw_rate=0)


class PedestrianDetector2D(PedestrianDetector2DShared):
    """Detects pedestrians.
    
    self.vehicle -> GEM car info
    self.detector -> pedestrian detection model
    self.last_person_boxes -> used to store boxes detected in every frame of the video
    self.pedestrians -> stores the agentstate (pedestrian) found during the video in a dict, not sure if required can be removed
    self.visualization -> enable this to view pedestrian detected
    self.confidence -> min model confidence level
    self.classes_to_detect -> Classes for the model to detect
    
    """
    def __init__(self,vehicle_interface : GEMInterface):
        self.detector = YOLO(os.getcwd()+'/GEMstack/knowledge/detection/yolov8n.pt') # change to get model value from sys arg
        self.last_person_boxes = [] 
        self.confidence = 0.7 # YOLO Confidence level at which a pedestrian is considered detected
        self.classes_to_detect = 0 # The YOLO class id's that are being detected

        self.fusion = Fusion3D()
    
    def initialize(self):
        """Initializes subscribers and publishers
        
        self.vehicle_interface.subscribe_sensor -> GEM Car camera subscription
        self.rosbag_test -> ROS Bag topic subscription
        self.pub_image -> Publishes Image with detection for visualization

        """
        # Subscribers and sychronizers
        self.rgb_rosbag = message_filters.Subscriber('/oak/rgb/image_raw', Image)
        self.top_lidar_rosbag = message_filters.Subscriber('/ouster/points', PointCloud2)
        self.sync = message_filters.ApproximateTimeSynchronizer([self.rgb_rosbag, self.top_lidar_rosbag], queue_size=2, slop=0.1)
        self.sync.registerCallback(self.detect_pedestrians)

    def detect_pedestrians(self, front_image_msg: Image, ouster_msg: PointCloud2D):
        """Fuses Image and Lidar information to detect pedestrians
        """
        # TODO: make fusion_callback return some data. Call the rest of Lukas' functions and Akul's stuff inside of this function.
        data = self.fusion.fusion_callback(rgb_image_msg=front_image_msg, lidar_pc2_msg=ouster_msg)
        
        # Original plan for how to do this:
        # # combined_point_cloud = self.combine_point_clouds # Removed because we only have transformation for a single lidar (ouster) at the moment. Probably should return np.array

        # point_cloud = np.array(list(pc2.read_points(ouster_msg, skip_nans=True)), dtype=np.float32)[:, :3]

        # image_pedestrians = self.detect_pedestrians_in_image(image=front_image_msg)

        # for id in image_pedestrians:
        #     #### Scrum-20: Calculate and Convert Image Points to Lidar Frame of Reference Task:
        #     (estimated_ped_cloud, flat_center) = self.extract_ped_cloud(point_cloud=point_cloud, image_pedestrian=image_pedestrians[key])

        #     #### Scrum-21 & 39:Calculate Pedestrian Center and Dimensions
        #     # Determine a more exact center and dimensions of the pedestrian by ignoring background point cloud points
        #     (pose, dims) = self.calc_ped_center_dims(estimated_ped_cloud, flat_center)
        #     image_pedestrians[id].pose = pose
        #     image_pedestrians[id].dims = dims

        # #### Scrum-35: Associate and Track Pedestrian Id's
        # self.associate_and_track_peds(image_pedestrians)

    # TODO: refactor into pedestrian_detection.py
    def update(self, vehicle : VehicleState) -> Dict[str,AgentState]:
        self.prev_agents = self.current_agents
        return self.prev_agents

    # clusters: list[ np.array(shape = (N, XYZ) ]
    # TODO: Improve Algo Knn, ransac, etc.
    def find_centers(clusters: list[np.ndarray]): #TODO: comment specifies np.array, type hint specifies np.ndarray
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
                track_id = track_ids[ind],
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
    
    def extract_ped_cloud(point_cloud: np.array, image_pedestrian: AgentState):
        # return (estimated_ped_cloud, flat_center)
        pass

    def calc_ped_center_dims(estimated_ped_cloud: np.array, flat_center: np.array):
        #### Scrum-21 & 39:Calculate Pedestrian Center and Dimensions
        # Determine a more exact center and dimensions of the pedestrian by ignoring background point cloud points
        # return (pose, dims)
        pass

    def associate_and_track_peds(image_pedestrians: dict):
        #### Scrum-35: Associate and Track Pedestrian Id'sSS

    def update(self, vehicle: VehicleState) -> Dict[str, AgentState]:
        res = {}
        for i,b in enumerate(self.last_person_boxes):
            x,y,w,h = b
            res['pedestrian'+str(i)] = self.box_to_fake_agent(b)
        if len(res) > 0:
            print("Detected",len(res),"pedestrians")
        return res


class FakePedestrianDetector2D(PedestrianDetector2DShared):
    """Triggers a pedestrian detection at some random time ranges"""
    def __init__(self, vehicle_interface: GEMInterface):
        self.times = [(5.0,20.0),(30.0,35.0)]
        self.t_start = None
    
    def update(self, vehicle: VehicleState) -> Dict[str, AgentState]:
        if self.t_start is None:
            self.t_start = self.vehicle_interface.time()
        t = self.vehicle_interface.time() - self.t_start
        res = {}
        for times in self.times:
            if t >= times[0] and t <= times[1]:
                res['pedestrian0'] = self.box_to_fake_agent((0,0,0,0))
                print("Detected a pedestrian")
        return res
