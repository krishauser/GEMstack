from ...state import AllState,VehicleState,ObjectPose,ObjectFrameEnum,AgentState,AgentEnum,AgentActivityEnum
from ...utils import settings
from ...mathutils import transforms
from ..interface.gem import GEMInterface
from ..component import Component
from ultralytics import YOLO
import cv2
try:
    from sensor_msgs.msg import CameraInfo
    from image_geometry import PinholeCameraModel
    import rospy
except ImportError:
    pass
import numpy as np
from typing import Dict,Tuple, List
import time

def Pmatrix(fx,fy,cx,cy):
    """Returns a projection matrix for a given set of camera intrinsics."""
    return np.array([[fx,0,cx,0],
                     [0,fy,cy,0],
                     [0,0,1,0]])

def project_point_cloud(point_cloud : np.ndarray, P : np.ndarray, xrange : Tuple[int,int], yrange : Tuple[int,int]) -> Tuple[np.ndarray,np.ndarray]:
    """Projects a point cloud into a 2D image using a camera intrinsic projection matrix P.
    
    Returns:
        - point_cloud_image: an Nx2 array of (u,v) visible image coordinates
        - image_indices: an array of N indices of visible points into the original point cloud
    """
    #this is the easy but slow way
    #camera = PinholeCameraModel()
    #camera.fromCameraInfo(self.camera_info)
    # for i,p in enumerate(self.point_cloud_zed):
    #     if p[2] < 0:
    #         continue
    #     u,v = camera.project3dToPixel(p[:3])
    #     if u >= 0 and u < self.camera_info.width and v >= 0 and v < self.camera_info.height:
    #         point_cloud_image.append((u,v,i))
    #point_cloud_image = np.array(point_cloud_image)
    #image_indices = point_cloud_image[:,2].astype(int)
    #this is the hard but fast way

    pc_with_ids = np.hstack((point_cloud,np.arange(len(point_cloud)).reshape(-1,1)))
    pc_fwd = pc_with_ids[pc_with_ids[:,2] > 0]
    pxform = pc_fwd[:,:3].dot(P[:3,:3].T) + P[:3,3]
    uv = (pxform[:,0:2].T/pxform[:,2]).T
    inds = np.logical_and(np.logical_and(uv[:,0] >= xrange[0],uv[:,0] < xrange[1]),
                    np.logical_and(uv[:,1] >= yrange[0],uv[:,1] < yrange[1]))
    point_cloud_image = uv[inds]
    image_indices = pc_fwd[inds,3].astype(int)
    return point_cloud_image, image_indices

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

def filter_lidar_by_range(point_cloud, xrange: Tuple[float, float], yrange: Tuple[float, float]):
    xmin, xmax = xrange
    ymin, ymax = yrange
    idxs = np.where((point_cloud[:, 0] > xmin) & (point_cloud[:, 0] < xmax) &
                    (point_cloud[:, 1] > ymin) & (point_cloud[:, 1] < ymax) )
    return point_cloud[idxs]

class PedestrianDetector(Component):
    """Detects and tracks pedestrians."""
    def __init__(self,vehicle_interface : GEMInterface):
        self.vehicle_interface = vehicle_interface
        self.detector = YOLO(settings.get('pedestrian_detection.model'))
        self.camera_info_sub = None
        self.camera_info = None
        self.zed_image = None
        self.last_person_boxes = []
        self.lidar_translation = np.array(settings.get('vehicle.calibration.top_lidar.position'))
        self.lidar_rotation = np.array(settings.get('vehicle.calibration.top_lidar.rotation'))
        self.zed_translation = np.array(settings.get('vehicle.calibration.front_camera.rgb_position'))
        self.zed_rotation = np.array(settings.get('vehicle.calibration.front_camera.rotation'))
        self.T_lidar = np.eye(4)
        self.T_lidar[:3,:3] = self.lidar_rotation
        self.T_lidar[:3,3] = self.lidar_translation
        self.T_zed = np.eye(4)
        self.T_zed[:3,:3] = self.zed_rotation
        self.T_zed[:3,3] = self.zed_translation
        self.T_lidar_to_zed = np.linalg.inv(self.T_zed) @ self.T_lidar
        self.point_cloud = None
        self.point_cloud_zed = None
        assert(settings.get('vehicle.calibration.top_lidar.reference') == 'rear_axle_center')
        assert(settings.get('vehicle.calibration.front_camera.reference') == 'rear_axle_center')
        self.pedestrian_counter = 0
        self.last_agent_states = {}
        self.previous_agents = {} 
        
        # init transformation parameters
        extrinsic = [[ 0.35282628 , -0.9356864 ,  0.00213977, -1.42526548],
                          [-0.04834961 , -0.02051524, -0.99861977, -0.02062586],
                          [ 0.93443883 ,  0.35223584, -0.05247839, -0.15902421],
                          [ 0.         ,  0.        ,  0.        ,  1.        ]]
        self.extrinsic = np.asarray(extrinsic)
        intrinsic = [527.5779418945312, 0.0, 616.2459716796875, 0.0, 527.5779418945312, 359.2155456542969, 0.0, 0.0, 1.0]
        intrinsic = np.array(intrinsic).reshape((3, 3))
        self.intrinsic = np.concatenate([intrinsic, np.zeros((3, 1))], axis=1)

        T_lidar2_Gem = [[ 0.9988692,  -0.04754282, 0.,       0.81915   ],
                        [0.04754282,  0.9988692,    0.,          0.        ],
                        [ 0.,          0.,          1.,          1.7272    ],
                        [ 0.,          0.,          0.,          1.        ]]
        self.T_lidar2_Gem = np.asarray(T_lidar2_Gem)

        # obtained by GEMstack/offboard/calibration/check_target_lidar_range.py
        # Hardcode the roi area for agents
        self.xrange = (2.3959036, 5.8143473)
        self.yrange = (-2.0247698, 4.0374074)
        
    def rate(self):
        return 4.0
    
    def state_inputs(self):
        return ['vehicle']
    
    def state_outputs(self):
        return ['agents']
    
    def test_set_data(self, zed_image, point_cloud, camera_info='dummy'):
        self.zed_image = zed_image
        self.point_cloud = point_cloud
        self.camera_info = camera_info

    def initialize(self):
        #tell the vehicle to use image_callback whenever 'front_camera' gets a reading, and it expects images of type cv2.Mat
        self.vehicle_interface.subscribe_sensor('front_camera',self.image_callback,cv2.Mat)
        #tell the vehicle to use lidar_callback whenever 'top_lidar' gets a reading, and it expects numpy arrays
        self.vehicle_interface.subscribe_sensor('top_lidar',self.lidar_callback,np.ndarray)
        #subscribe to the Zed CameraInfo topic
        self.camera_info_sub = rospy.Subscriber("/zed2/zed_node/rgb/camera_info", CameraInfo, self.camera_info_callback)


    def image_callback(self, image : cv2.Mat):
        self.zed_image = image

    def camera_info_callback(self, info : CameraInfo):
        self.camera_info = info

    def lidar_callback(self, point_cloud: np.ndarray):
        self.point_cloud = point_cloud
    
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
        
        #debugging
        #self.save_data()

        t1 = time.time()
        detected_agents = self.detect_agents()

        t2 = time.time()
        current_agent_states = self.track_agents(vehicle,detected_agents)
        t3 = time.time()
        print("Detection time",t2-t1,", shape estimation and tracking time",t3-t2)

        self.last_agent_states = current_agent_states
        return current_agent_states

    def box_to_agent(self, box, point_cloud_image, point_cloud_image_world):
        """Creates a 3D agent state from an (x,y,w,h) bounding box.

        TODO: you need to use the image, the camera intrinsics, the lidar
        point cloud, and the calibrated camera / lidar poses to get a good
        estimate of the pedestrian's pose and dimensions.
        """
        # get the idxs of point cloud that belongs to the agent
        x,y,w,h = box
        xmin, xmax = x - w/2, x + w/2
        ymin, ymax = y - h/2, y + h/2
        
        # enlarge bbox in case inaccuracy calibration
        enlarge_factor = 3
        xmin *= enlarge_factor
        xmax *= enlarge_factor
        ymin *= enlarge_factor
        ymax *= enlarge_factor

        agent_world_pc = point_cloud_image_world

        # idxs = np.where((point_cloud_image[:, 0] > xmin) & (point_cloud_image[:, 0] < xmax) &
        #                 (point_cloud_image[:, 1] > ymin) & (point_cloud_image[:, 1] < ymax) )
        
        # agent_image_pc = point_cloud_image[idxs]
        # agent_world_pc = point_cloud_image_world[idxs]
        # print ('# of agent_world_pc:', len(agent_world_pc))
        # if len(agent_world_pc) == 0: # might FP bbox from YOLO
        #     return None

        # Find the point_cloud that is closest to the center of our bounding box
        center_x = x + w / 2
        center_y = y + h / 2
        distances = np.linalg.norm(point_cloud_image - [center_x, center_y], axis=1)
        closest_point_cloud_idx = np.argmin(distances)
        closest_point_cloud = point_cloud_image_world[closest_point_cloud_idx]

        #########################################################################################################
        # Definition of ObjectPose and dimensions:
        # 
        #   Copy from the comment of class PhysicalObject:
        #     The origin is at the object's center in the x-y plane but at the bottom
        #     in the z axis.  I.e., if l,w,h are the dimensions, then the object is
        #     contained in a bounding box [-l/2,l/2] x [-w/2,w/2] x [0,h].
        #   
        #   Copy from the comment of class ObjectFrameEnum(Enum):
        #     ObjectFrameEnum.CURRENT: position / yaw in m / radians relative to current pose of vehicle
        #########################################################################################################
        
        # Specify ObjectPose. Note that The pose's yaw, pitch, and roll are assumed to be 0 for simplicity.
        x, y, _ = closest_point_cloud
        pose = ObjectPose(t=0, x=x, y=y, z=0, yaw=0, pitch=0, roll=0, frame=ObjectFrameEnum.CURRENT)
        
        # Specify AgentState.
        l = np.max(agent_world_pc[:, 0]) - np.min(agent_world_pc[:, 0])
        w = np.max(agent_world_pc[:, 1]) - np.min(agent_world_pc[:, 1])
        h = np.max(agent_world_pc[:, 2]) - np.min(agent_world_pc[:, 2])
        dims = (2, 2, 1.7) 
        return AgentState(pose=pose,dimensions=dims,outline=None,type=AgentEnum.PEDESTRIAN,activity=AgentActivityEnum.MOVING,velocity=(0,0,0),yaw_rate=0)

        
    def detect_agents(self):
        detection_result = self.detector(self.zed_image,verbose=False)
        
        #TODO: create boxes from detection result
        pedestrian_boxes = []
        for box in detection_result[0].boxes: # only one image, so use index 0 of result
           class_id = int(box.cls[0].item())
           if class_id == 0: # class 0 stands for pedestrian
               bbox = box.xywh[0].tolist()
               pedestrian_boxes.append(bbox)
    
        # Only keep lidar point cloud that lies in roi area for agents
        point_cloud_lidar = filter_lidar_by_range(self.point_cloud, self.xrange, self.yrange)
        
        # Tansfer lidar point cloud to camera frame
        point_cloud_image = lidar_to_image(point_cloud_lidar, self.extrinsic, self.intrinsic)
        
        # Tansfer lidar point cloud to vehicle frame
        point_cloud_image_world = lidar_to_vehicle(point_cloud_lidar, self.T_lidar2_Gem)

        # Find agents
        detected_agents = []
        for i,b in enumerate(pedestrian_boxes):
            agent = self.box_to_agent(b, point_cloud_image, point_cloud_image_world)
            if agent is not None:
                detected_agents.append(agent)
        return detected_agents
        
    def estimate_velocity(self, prev_pose, current_pose) -> tuple:
        """Estimate the velocity of an agent based on previous and current positions."""
        delta_time = 1.0  
        velocity_x = (current_pose.x -prev_pose.x) / delta_time
        velocity_y = (current_pose.y -prev_pose.y) / delta_time
        velocity_z = (current_pose.z -prev_pose.z) / delta_time
        return (velocity_x, velocity_y, velocity_z)
    
    def overlaps(self, pose1, pose2) -> bool:
        """Check if two agents overlap (simplified check)."""
        distance_threshold = 0.5
        distance = ((pose1.x - pose2.x) ** 2 + (pose1.y - pose2.y) ** 2 + (pose1.z - pose2.z) ** 2) ** 0.5
        return distance < distance_threshold

    def track_agents(self, vehicle : VehicleState, detected_agents : List[AgentState]):
        """Given a list of detected agents, updates the state of the agents."""
        # TODO: keep track of which pedestrians were detected before u 
        # results = {}

        current_frame_results = {}
        new_pedestrian_counter = 0  

        for current_agent in detected_agents:
            assigned = False  # Flag to check if the current agent is assigned to an existing track

            for ped_id, prev_agent in self.previous_agents.items():
                
                # Car moves between frames. So have to transform the previous agent's pose
                # to align with the current vehicle's pose.
                # prev_agent = prev_agent.to_frame(ObjectFrameEnum.CURRENT, vehicle.pose)
                
                if self.overlaps(current_agent.pose, prev_agent.pose):
                    # If the current agent overlaps with a previous agent, it's the same pedestrian
                    velocity = self.estimate_velocity(prev_agent.pose, current_agent.pose)
                    # Update agent state with new position and calculated velocity
                    updated_agent = AgentState(pose=current_agent.pose, dimensions=current_agent.dimensions,
                                            outline=current_agent.outline, type=current_agent.type,
                                            activity=AgentActivityEnum.MOVING, velocity=velocity, yaw_rate=current_agent.yaw_rate)
                    current_frame_results[ped_id] = updated_agent
                    assigned = True
                    break  

            if not assigned:
                # This is a new pedestrian
                new_id = f"ped{self.pedestrian_counter + new_pedestrian_counter}"
                new_pedestrian_counter += 1
                # For new pedestrians, the velocity is initialized to (0, 0, 0)
                new_agent = AgentState(pose=current_agent.pose, dimensions=current_agent.dimensions,
                                    outline=None, type=AgentEnum.PEDESTRIAN,
                                    activity=AgentActivityEnum.MOVING, velocity=(0, 0, 0), yaw_rate=0)
                current_frame_results[new_id] = new_agent

        # Update the pedestrian counter and previous_agents for the next frame
        self.pedestrian_counter += new_pedestrian_counter
        self.previous_agents = {p_id: agent for p_id, agent in current_frame_results.items()}

        return current_frame_results
        # for i,a in enumerate(detected_agents):
        #     results['pedestrian_'+str(self.pedestrian_counter)] = a
        #     self.pedestrian_counter += 1
        # return results

    def save_data(self, loc=None):
        """This can be used for debugging.  See the provided test."""
        prefix = ''
        if loc is not None:
            prefix = loc + '/'
        cv2.imwrite(prefix+'zed_image.png',self.zed_image)
        np.savez(prefix+'velodyne_point_cloud.npz',self.point_cloud)
        import pickle
        with open(prefix+'zed_camera_info.pkl','wb') as f:
            pickle.dump(self.camera_info,f)

    def load_data(self, loc=None):
        prefix = ''
        if loc is not None:
            prefix = loc + '/'
        self.zed_image = cv2.imread(prefix+'zed_image.png')
        self.point_cloud = np.load(prefix+'velodyne_point_cloud.npz')['arr_0']
        try:
            import pickle
            with open(prefix+'zed_camera_info.pkl','rb') as f:
                self.camera_info = pickle.load(f)
        except ModuleNotFoundError:
            #ros not found?
            from collections import namedtuple
            CameraInfo = namedtuple('CameraInfo',['width','height','P'])
            #TODO: these are guessed parameters
            self.camera_info = CameraInfo(width=1280,height=720,P=[560.0,0,640.0,0,  0,560.0,360,0,  0,0,1,0])
