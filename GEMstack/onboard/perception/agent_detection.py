from ...state import AllState,VehicleState,ObjectPose,ObjectFrameEnum,AgentState,AgentEnum,AgentActivityEnum
from ..interface.gem import GEMInterface
from ..component import Component
from typing import Dict
import threading
from sklearn.cluster import DBSCAN
import copy
import cv2
import numpy as np
from ultralytics import YOLO
from typing import Dict, Tuple, List
from kalman_tracker import KalmanFilter, KalmanTracker
from pixelwise_3D_lidar_coord_handler import PixelWise3DLidarCoordHandler
try:
    from sensor_msgs.msg import CameraInfo
    from image_geometry import PinholeCameraModel
    import rospy
except ImportError:
    pass

class OmniscientAgentDetector(Component):
    """Obtains agent detections from a simulator"""
    def __init__(self,vehicle_interface : GEMInterface):
        self.vehicle_interface = vehicle_interface
        self.agents = {}
        self.lock = threading.Lock()

    def rate(self):
        return 4.0
    
    def state_inputs(self):
        return []
    
    def state_outputs(self):
        return ['agents']

    def initialize(self):
        self.vehicle_interface.subscribe_sensor('agent_detector',self.agent_callback, AgentState)
    
    def agent_callback(self, name : str, agent : AgentState):
        with self.lock:
            self.agents[name] = agent

    def update(self) -> Dict[str,AgentState]:
        with self.lock:
            return copy.deepcopy(self.agents)
        
#=================================================================================================================================#
colors = []
colors += [(0, 255, i) for i in range(100, 256, 25)]
id_dict = {
    0: "pedestrian",
    2: "car",
    11: "stop sign"
}

def lidar_to_image(point_cloud_lidar: np.ndarray, extrinsic : np.ndarray, intrinsic : np.ndarray):
    
    homo_point_cloud_lidar = np.hstack((point_cloud_lidar, np.ones((point_cloud_lidar.shape[0], 1)))) # (N, 4)
    pointcloud_pixel = (intrinsic @ extrinsic @ (homo_point_cloud_lidar).T) # (3, N)
    pointcloud_pixel = pointcloud_pixel.T # (N, 3)

    # normalize
    pointcloud_pixel[:, 0] /= pointcloud_pixel[:, 2] # normalize
    pointcloud_pixel[:, 1] /= pointcloud_pixel[:, 2] # normalize
    point_cloud_image =  pointcloud_pixel[:,:2] # (N, 2)
    return point_cloud_image

def lidar_to_vehicle(point_cloud_lidar: np.ndarray, T_lidar2_Gem: np.ndarray):
    ones = np.ones((point_cloud_lidar.shape[0], 1))
    pcd_homogeneous = np.hstack((point_cloud_lidar, ones)) # (N, 4)
    pointcloud_trans = np.dot(T_lidar2_Gem, pcd_homogeneous.T) # (4, N)
    pointcloud_trans = pointcloud_trans.T # (N, 4)
    point_cloud_image_world = pointcloud_trans[:, :3] # (N, 3)
    return point_cloud_image_world

def filter_lidar_by_range(point_cloud, xrange: Tuple[float, float], yrange: Tuple[float, float], zrange: Tuple[float, float]):
    xmin, xmax = xrange
    ymin, ymax = yrange
    zmin, zmax = zrange
    idxs = np.where((point_cloud[:, 0] > xmin) & (point_cloud[:, 0] < xmax) &
                    (point_cloud[:, 1] > ymin) & (point_cloud[:, 1] < ymax) &
                     (point_cloud[:, 2] > zmin) & (point_cloud[:, 2] < zmax) )
    return point_cloud[idxs]
        
class MultiObjectDetector(Component):
    """Utilize YOLOv8 to detect multi-objects in each frame"""
    def __init__(self,vehicle_interface : GEMInterface, extrinsic=None):
        self.vehicle_interface = vehicle_interface
        self.detector = YOLO('GEMstack/knowledge/detection/yolov8n.pt')
        self.camera_info_sub = None
        self.camera_info = None
        self.zed_image = None
        self.pedestrian_counter = 0 # Maybe should use dictionary
        self.car_counter = 0
        self.stop_sign_counter = 0
        self.last_agent_states = {}
        self.previous_agents = {} 
        
        # init transformation parameters
        if extrinsic is None:
            extrinsic = np.loadtxt("GEMstack/knowledge/calibration/gem_e4_lidar2oak.txt")
        self.extrinsic = np.array(extrinsic)
            
        intrinsic = np.loadtxt("GEMstack/knowledge/calibration/gem_e4_intrinsic.txt")
        self.intrinsic = np.concatenate([intrinsic, np.zeros((3, 1))], axis=1)

        T_lidar2_Gem = np.loadtxt("GEMstack/knowledge/calibration/gem_e4_lidar2vehicle.txt")
        self.T_lidar2_Gem = np.asarray(T_lidar2_Gem)

        # Hardcode the roi area for agents
        self.xrange = (0, 100)
        self.yrange = (-100, 100)
    
    def test_set_data(self, zed_image, point_cloud, camera_info='dummy'):
        self.zed_image = zed_image
        self.point_cloud = point_cloud
        self.camera_info = camera_info

    def state_inputs(self):
        return ['vehicle']
    
    def state_outputs(self):
        # return ['agents']
        return ["detected_agents"]

    def initialize(self):
        #tell the vehicle to use image_callback whenever 'front_camera' gets a reading, and it expects images of type cv2.Mat
        self.vehicle_interface.subscribe_sensor('front_camera',self.image_callback,cv2.Mat)
        #tell the vehicle to use lidar_callback whenever 'top_lidar' gets a reading, and it expects numpy arrays
        self.vehicle_interface.subscribe_sensor('top_lidar',self.lidar_callback,np.ndarray)
        #subscribe to the Zed CameraInfo topic
        self.camera_info_sub = rospy.Subscriber("/oak/rgb/camera_info", CameraInfo, self.camera_info_callback)


    def image_callback(self, image : cv2.Mat):
        self.zed_image = image

    def camera_info_callback(self, info : CameraInfo):
        self.camera_info = info

    def update(self, vehicle : VehicleState) -> Dict[str,AgentState]:
        if self.zed_image is None:
            #no image data yet
            return {}
        if self.point_cloud is None:
            #no lidar data yet
            return {}
        if self.camera_info is None:
            #no camera info yet
            return {}

        detected_agents = self.detect_agents()
        # current_agent_states = self.track_agents(vehicle,detected_agents)
        # self.last_agent_states = current_agent_states
        # return current_agent_states
        return detected_agents

    def detect_agents(self):
        detection_result = self.detector(self.zed_image,verbose=False)
        
        #TODO: create boxes from detection result
        boxes = []
        target_ids = [0, 2, 11] # 0 for pedestrian, 2 for car, 11 for stop sign
        bbox_ids = []
        for box in detection_result[0].boxes: # only one image, so use index 0 of result
            class_id = int(box.cls[0].item())
            if class_id in target_ids: # class 0 stands for pedestrian
                bbox_ids.append(class_id)
                bbox = box.xywh[0].tolist()
                boxes.append(bbox)
    
        # Only keep lidar point cloud that lies in roi area for agents
        filtered_point_cloud = filter_lidar_by_range(self.point_cloud, 
                                                  self.xrange, 
                                                  self.yrange)
        
        epsilon = 0.09  # Epsilon parameter for DBSCAN

        # Perform DBSCAN clustering
        min_samples = 5  # Minimum number of samples in a cluster
        dbscan = DBSCAN(eps=epsilon, min_samples=min_samples)
        clusters = dbscan.fit_predict(filtered_point_cloud)
        
        # Tansfer lidar point cloud to camera frame
        point_cloud_image = lidar_to_image(filtered_point_cloud, 
                                           self.extrinsic, 
                                           self.intrinsic)
        
        # Tansfer lidar point cloud to vehicle frame
        pc_3D = lidar_to_vehicle(filtered_point_cloud, self.T_lidar2_Gem)

        vis = self.zed_image.copy()
        detected_agents = []
        for i in range(len(boxes)):
            box = boxes[i]
            id = boxes[i]

            if id == 0:
                color = (255, 0, 255)
                self.pedestrian_counter += 1
                print("Pedestrian")
            if id == 2:
                color = (0, 255, 255)
                self.car_counter += 1
                print("Vehicle")
            if id == 11:
                color = (150, 50, 255)
                self.stop_sign_counter += 1
                print("Stop Sign")
            print(box)

            x, y, w, h = box
            xmin, xmax = x - w/2, x + w/2
            ymin, ymax = y - h/2, y + h/2
            l = ymax - ymin

            # Filter. Get the idxs of point cloud that belongs to the agent
            idxs = np.where((point_cloud_image[:, 0] > xmin) & (point_cloud_image[:, 0] < xmax) &
                            (point_cloud_image[:, 1] > ymin) & (point_cloud_image[:, 1] < ymax) )
            agent_image_pc = point_cloud_image[idxs]
            agent_pc_3D = pc_3D[idxs]
            agent_clusters = clusters[idxs]

            # draw bbox
            left_up = (int(x-w/2), int(y-h/2))
            right_bottom = (int(x+w/2), int(y+h/2))
            cv2.rectangle(vis, left_up, right_bottom, color, thickness=2)

            # Get unique elements and their counts
            unique_elements, counts = np.unique(agent_clusters, return_counts=True)
            max_freq = np.max(counts)
            label_cluster = unique_elements[counts == max_freq]
            
            # filter again
            idxs = agent_clusters == label_cluster
            agent_image_pc = agent_image_pc[idxs]
            agent_clusters = agent_clusters[idxs]
            agent_pc_3D = agent_pc_3D[idxs]

            # calulate depth average
            depth = np.mean( (agent_pc_3D[:, 0] ** 2 + agent_pc_3D[:, 1] ** 2) ** 0.5 ) # euclidean dist
            print ('depth:', depth)

            # draw
            text = str(id_dict[id])
            text += f" | depth: {depth:.2f}"
            text_size, baseline = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.4, 1)
            p1 = (left_up[0], left_up[1] - text_size[1])
            cv2.rectangle(vis, (p1[0] - 2 // 2, p1[1] - 2 - baseline), (p1[0] + text_size[0], p1[1] + text_size[1]),
                        color, -1)
            cv2.putText(vis, text, (p1[0], p1[1] + baseline), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1, 8)
            
            # draw point cloud
            # for proj_pt, cluster in zip(agent_image_pc, agent_clusters):
            #     if cluster != label_cluster:
            #         continue
            #     color = colors[cluster % len(colors)]
            #     radius = 1
            #     center = int(proj_pt[0]), int(proj_pt[1])
            #     vis = cv2.circle(vis, center, radius, color, cv2.FILLED)
            detected_agents.append([x, y, w, l])
        return detected_agents

class MultiObjectTracker():
    """Using Kalman Filter to track objects"""
    def __init__(
        self,
        kalman_config_file="GEMstack/onboard/perception/temp_config.py",
        detection_file_name="GEMstack/onboard/prediction/tracking_results.txt",
        test=False,
        write_all=False,
    ):
        self.kalman_tracker = KalmanTracker(config_file_path=kalman_config_file)
        self.tracking_results = {}
        self.current_frame = 0

    """Base class for top-level components in the execution stack."""

    def rate(self) -> float:
        """Returns the rate in Hz at which this component should be updated."""
        return 2.5

    def state_inputs(self) -> List[str]:
        """Returns the list of AllState inputs this component requires."""
        return ["detected_agents"]

    def state_outputs(self) -> List[str]:
        """Returns the list of AllState outputs this component generates."""
        return ["tracking_frames"]

    def healthy(self):
        """Returns True if the element is in a stable state."""
        return True

    def initialize(self):
        """Initialize the component. This is called once before the first
        update."""
        pass

    def cleanup(self):
        """Cleans up resources used by the component. This is called once after
        the last update."""
        pass

    def update(self, detected_agents: List[AgentState]):
        """Update the component."""
        self.track_agents(detected_agents=detected_agents)
        return self.output_tracking_results()

    def debug(self, item, value):
        """Debugs a streaming value within this component"""
        if hasattr(self, "debugger"):
            self.debugger.debug(item, value)

    def debug_event(self, label):
        """Debugs an event within this component"""
        if hasattr(self, "debugger"):
            self.debugger.debug_event(label)

    def track_agents(self, detected_agents: List[AgentState]):
        """Given a list of detected agents, updates the state of the agents using a Kalman Tracker."""
        detections = []
        for agent in detected_agents:
            if agent.type == AgentEnum.PEDESTRIAN:
                x, y, z = agent.pose.x, agent.pose.y, agent.pose.z
                w, h, l = agent.dimensions
                detections.append(np.array([x, y, w, l]))

        kalman_agent_states, matches = self.kalman_tracker.update_pedestrian_tracking(
            detections
        )
        tracking_results = {}
        for pid in kalman_agent_states:
            ag_state = kalman_agent_states[pid]
            tracking_results[pid] = AgentState(
                pose=ObjectPose(
                    t=0,
                    x=ag_state[0],
                    y=ag_state[1],
                    z=0,
                    yaw=0,
                    pitch=0,
                    roll=0,
                    frame=ObjectFrameEnum.CURRENT,
                ),
                dimensions=(ag_state[2], ag_state[3], 1.5),
                velocity=(ag_state[4], ag_state[5], 0),
                type=AgentEnum.PEDESTRIAN,
                activity=AgentActivityEnum.MOVING,
                yaw_rate=0,
                outline=None,
            )
        self.update_track_history(tracking_results)

        
        if self.test:
            return tracking_results, matches

    def update_track_history(
        self, ag_dict: Dict[int, AgentState]
    ):  # Behavior Prediction
        for pid in sorted(ag_dict):
            curr_agent = ag_dict[pid]
            curr_pose = curr_agent.pose
            # 11.0 5.0 Pedestrian -1.0 -1.0 -1.0 -1.0 -1.0 -1.0 -1.0 -1.0 -1.0 -1.0 -1.59 -1.0 0.93 -1.0
            agent_frame_data = f"{float(self.current_frame)} {float(pid)} Pedestrian -1.0 -1.0 -1.0 -1.0 -1.0 -1.0 -1.0 -1.0 -1.0 -1.0 {curr_pose.x} -1.0 {curr_pose.y} -1.0\n"
            if self.current_frame in self.tracking_results:
                self.tracking_results[self.current_frame].append(agent_frame_data)
            else:
                self.tracking_results[self.current_frame] = [agent_frame_data]
        # Do not keep more than 8 frames
        if not self.write_all:
            # Remove 8th oldest frame
            self.tracking_results.pop(self.current_frame - 8, None)
        
        self.current_frame += 1

    # TODO: Write Docstring
    def output_tracking_results(self):  # Behavior Prediction
        tracking_frames = []

        recent_pids = {}
        
        # Dummy frames should be written for these pids
        valid_pids = set()
        with open(self.detection_file_name, "w") as f:
            for frame in sorted(self.tracking_results):
                for line in sorted(self.tracking_results[frame]):
                    
                    # Check whether a dummy frame should be written for this pid
                    curr_pid = line.split(" ")[1]
                    pid_seen_count = recent_pids.get(curr_pid, 0) + 1
                    if pid_seen_count >= 8:
                        valid_pids.add(curr_pid)
                        
                    recent_pids[curr_pid] = pid_seen_count
                        
                        
                    if self.test:
                        f.write(line)
                    tracking_frames.append(line)
            if not self.write_all:
                # Adding dummy frames, because we are only writing the recent frames
                for frame in range(self.current_frame, self.current_frame + 12):
                    for rpid in valid_pids:
                        dummy_frame = f"{float(frame)} {float(rpid)} Pedestrian -1.0 -1.0 -1.0 -1.0 -1.0 -1.0 -1.0 -1.0 -1.0 -1.0 0.0 -1.0 0.0 -1.0\n"
                        if self.test:
                            f.write(dummy_frame)
                        tracking_frames.append(dummy_frame)

        return tracking_frames
    