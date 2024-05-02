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
from numpy.linalg import inv


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

class TDMultiClassDetector(Component):
    """Detects and tracks pedestrians."""
    def __init__(self,vehicle_interface : GEMInterface, extrinsic=None):
        # State Estimation
        self.vehicle_interface = vehicle_interface

        self.detector = YOLO('GEMstack/knowledge/detection/yolov8n.pt')
        self.camera_info_sub = None
        self.camera_info = None
        self.zed_image = None



        self.point_cloud = None
        self.point_cloud_zed = None
        assert(settings.get('vehicle.calibration.top_lidar.reference') == 'rear_axle_center')
        assert(settings.get('vehicle.calibration.front_camera.reference') == 'rear_axle_center')

        
        if extrinsic is None:
            extrinsic = np.loadtxt("GEMstack/knowledge/calibration/gem_e4_lidar2oak.txt")
        self.extrinsic = np.array(extrinsic)
            
        intrinsic = np.loadtxt("GEMstack/knowledge/calibration/gem_e4_intrinsic.txt")
        self.intrinsic = np.concatenate([intrinsic, np.zeros((3, 1))], axis=1)

        T_lidar2_Gem = np.loadtxt("GEMstack/knowledge/calibration/gem_e4_lidar2vehicle.txt")
        self.T_lidar2_Gem = np.asarray(T_lidar2_Gem)

        # Hardcode the roi area for agents
        self.xrange = (1, 20)
        self.yrange = (-10, 10)


    def rate(self):
        return 2.5

    def state_inputs(self):
        return ['vehicle']

    def state_outputs(self):
        return ['detected_agents']

    def test_set_data(self, zed_image, point_cloud, camera_info='dummy'):
        self.zed_image = zed_image
        self.point_cloud = point_cloud
        self.camera_info = camera_info

    def initialize(self):
        print("INITIALIZE PED DETECT")
        # tell the vehicle to use image_callback whenever 'front_camera' gets a reading, and it expects images of type cv2.Mat
        self.vehicle_interface.subscribe_sensor('front_camera',self.image_callback,cv2.Mat)
        # tell the vehicle to use lidar_callback whenever 'top_lidar' gets a reading, and it expects numpy arrays
        self.vehicle_interface.subscribe_sensor('top_lidar',self.lidar_callback,np.ndarray)
        # subscribe to the Zed CameraInfo topic
        self.camera_info_sub = rospy.Subscriber("/oak/rgb/camera_info", CameraInfo, self.camera_info_callback)

    def image_callback(self, image : cv2.Mat):
        self.zed_image = image

    def camera_info_callback(self, info : CameraInfo):
        self.camera_info = info

    def lidar_callback(self, point_cloud: np.ndarray):
        self.point_cloud = point_cloud

    def update(self, vehicle : VehicleState) -> Dict[str,AgentState]:
        print("0PED DETECT UPDATE", self.zed_image is None, self.point_cloud is None, self.camera_info is  None)
        if self.zed_image is None:
            # no image data yet
            print("zed")
            return {}
        if self.point_cloud is None:
            # no lidar data yet
            print("pc")
            return {}
        if self.camera_info is None:
            # no camera info yet
            print("camera")
            return {}

        detected_agents = self.detect_agents()  # TODO: to ask SE to output detected_agents only


        print("Pedestrian Detection detected agents: ", detected_agents)
        return detected_agents


    def box_to_agent(self, box, point_cloud_image, point_cloud_image_world):
        """Creates a 3D agent state from an (x,y,w,h) bounding box.
        """

        # print ('Detect a pedestrian!')

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


        # Find the point_cloud that is closest to the center of our bounding box
        center_x = x + w / 2
        center_y = y + h / 2
        distances = np.linalg.norm(point_cloud_image - [center_x, center_y], axis=1)
        closest_point_cloud_idx = np.argmin(distances)
        closest_point_cloud = point_cloud_image_world[closest_point_cloud_idx]

        x, y, _ = closest_point_cloud
        pose = ObjectPose(t=0, x=x, y=y, z=0, yaw=0, pitch=0, roll=0, frame=ObjectFrameEnum.CURRENT)

        # Specify AgentState.
        l = np.max(agent_world_pc[:, 0]) - np.min(agent_world_pc[:, 0])
        w = np.max(agent_world_pc[:, 1]) - np.min(agent_world_pc[:, 1])
        h = np.max(agent_world_pc[:, 2]) - np.min(agent_world_pc[:, 2])

        dims = (w, h, l) 

        return AgentState(pose=pose,dimensions=dims,outline=None,type=AgentEnum.PEDESTRIAN,activity=AgentActivityEnum.MOVING,velocity=(0,0,0),yaw_rate=0)

    # Behavior Prediction added test argument to return matchings
    def detect_agents(self, test=False):
        detection_result = self.detector(self.zed_image,verbose=False)

        # TODO: create boxes from detection result
        pedestrian_boxes = []
        car_boxes = [] # add car objects

        for box in detection_result[0].boxes: # only one image, so use index 0 of result
            class_id = int(box.cls[0].item())
            if class_id == 0: # class 0 stands for pedestrian
                bbox = box.xywh[0].tolist()
                pedestrian_boxes.append(bbox)

            if class_id == 2: # class 2 stands for car
                bbox = box.xywh[0].tolist()
                car_boxes.append(bbox)


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
            agent.type = AgentEnum.PEDESTRIAN
            if agent is not None:
                detected_agents.append(agent)

        for i,b in enumerate(car_boxes):
            agent = self.box_to_agent(b, point_cloud_image, point_cloud_image_world)
            agent.type = AgentEnum.CAR
            if agent is not None:
                detected_agents.append(agent)

        
        if test: # Behavior Prediction
            return detected_agents, detection_result 
        return detected_agents


    