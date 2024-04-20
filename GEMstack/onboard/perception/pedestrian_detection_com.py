from ...state import AllState,VehicleState,ObjectPose,ObjectFrameEnum,AgentState,AgentEnum,AgentActivityEnum
from ...utils import settings
from ...mathutils import transforms, collisions
from ..interface.gem import GEMInterface
from ..component import Component
from ultralytics import YOLO
import cv2
import rospy
from sensor_msgs.msg import CameraInfo
from image_geometry import PinholeCameraModel

import numpy as np
from typing import Dict,Tuple
import time
import copy

PERSON_INDEX = 0


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
    def __init__(self,vehicle_interface : GEMInterface, extrinsic=None):
        self.vehicle_interface = vehicle_interface
        model_path = settings.get('pedestrian_detection.model')
        self.detector = YOLO(model_path)
        self._rate = settings.get('pedestrian_detection.rate')
        self.camera_info_sub = None
        self.camera_info = None
        self.zed_image = None
        self.last_person_boxes = []
        # self.lidar_translation = np.array([0.998621,  -0.052503,  0.000000] 0.052503, 0.998621, 0.000000, 0.000000,  0.000000,  1.000000])
        # self.lidar_rotation = np.array([1.10, 0.0 , 2.030])
        # self.zed_translation = np.array(settings.get('vehicle.calibration.front_camera.rgb_position'))
        # self.zed_rotation = np.array(settings.get('vehicle.calibration.front_camera.rotation'))
        self.T_lidar = np.array[[0.998621,  -0.052503,  0.000000, 1.10],
                       [0.052503, 0.998621, 0.000000, 0.0],
                       [0.000000,  0.000000,  1.000000, 2.030],
                       [0.0,  0.0,  0.0,   1.0]]
        # self.T_lidar[:3,:3] = self.lidar_rotation
        # self.T_lidar[:3,3] = self.lidar_translation
        self.T_zed = np.array[[ 0.0473086557, -6.72244788e-02,  9.98414035e-01,  1.81744824e+00],
                      [-9.98214961e-01, -5.37729808e-03,  3.33439481e-04,  1.62484518e-01],
                      [ 5.35126620e-03, -9.97733286e-01 -6.72232072e-02,  1.94900475e+00],
                     [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.00000000e+00]]
        # self.T_zed[:3,:3] = self.zed_rotation
        # self.T_zed[:3,3] = self.zed_translation
        # self.T_lidar_to_zed = np.linalg.inv(self.T_zed) @ self.T_lidar
        self.T_lidar_to_zed = np.array[[-0.00519, -0.99997, 0.005352, 0.1627], 
                            [-0.0675, -0.00499, -0.9977, -0.03123], 
                            [0.99771, -0.00554, -0.06743 -0.7284],
                            [0,       0 ,             0 ,      1]]

        self.point_cloud = None 
        self.point_cloud_zed = None
        assert(settings.get('vehicle.calibration.top_lidar.reference') == 'rear_axle_center')
        assert(settings.get('vehicle.calibration.front_camera.reference') == 'rear_axle_center')
        self.pedestrian_counter = 0, 
        self.last_agent_states = {}
        # if extrinsic is None:
        extrinsic = np.loadtxt("GEMstack/knowledge/calibration/gem_e4_oak2lidar.txt")
        self.extrinsic = np.array(extrinsic)
            
        intrinsic = np.loadtxt("GEMstack/knowledge/calibration/gem_e4_intrinsic.txt")
        # intrinsic = [684.8333129882812, 0.0, 573.37109375, 0.0, 684.6096801757812, 363.700927734375, 0.0, 0.0, 1.0] # e4
        # intrinsic = np.array(intrinsic).reshape((3, 3))
        # intrinsic = [527.5779418945312, 0.0, 616.2459716796875, 0.0, 527.5779418945312, 359.2155456542969, 0.0, 0.0, 1.0] #e2
        self.intrinsic = np.concatenate([intrinsic, np.zeros((3, 1))], axis=1)

        T_lidar2_Gem = np.loadtxt("GEMstack/knowledge/calibration/gem_e4_lidar2vehicle.txt")
        self.T_lidar2_Gem = np.asarray(T_lidar2_Gem)

        # obtained by GEMstack/offboard/calibration/check_target_lidar_range.py
        # Hardcode the roi area for agents
        self.xrange = (2.3959036, 100)
        self.yrange = (-200.0247698, 100.0374074)
    def rate(self):
        return self._rate
    
    def state_inputs(self):
        return ['vehicle']
    
    def state_outputs(self):
        return ['agents']
    
    def initialize(self):
        #tell the vehicle to use image_callback whenever 'front_camera' gets a reading, and it expects images of type cv2.Mat
        self.vehicle_interface.subscribe_sensor('front_camera',self.image_callback,cv2.Mat)
        #tell the vehicle to use lidar_callback whenever 'top_lidar' gets a reading, and it expects numpy arrays
        self.vehicle_interface.subscribe_sensor('top_lidar',self.lidar_callback,np.ndarray)
        #subscribe to the Zed CameraInfo topic
        self.camera_info_sub = rospy.Subscriber("/oak/rgb/camera_info", CameraInfo, self.camera_info_callback)
        pass
    
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
        x,y,w,h = box
        # find the point_cloud that is closest to the center of our bounding box
        # center_point_cloud_idx =  np.argmin(np.linalg.norm(point_cloud_image - [x,y],axis=1))
        # x_3d,y_3d,z_3d = point_cloud_image_world[center_point_cloud_idx]

        # the nearest point may be on the ground
        # so we take the 20 nearest points and take the mean of 5 with smallest distance
        center_point_cloud_idx =  np.argsort(np.linalg.norm(point_cloud_image - [x,y],axis=1))
        center_point_cloud_idx = center_point_cloud_idx[:20]
        center_point_cloud = point_cloud_image_world[center_point_cloud_idx]
        close_point_idx = np.argsort(np.linalg.norm(center_point_cloud,axis=1))
        close_point_idx = close_point_idx[:5]
        close_point_cloud = center_point_cloud[close_point_idx]
        x_3d,y_3d,z_3d = np.mean(close_point_cloud,axis=0)

        # we assume the pedestrian is 1.7m tall        # self.lidar_translation = np.array([0.998621,  -0.052503,  0.000000] 0.052503, 0.998621, 0.000000, 0.000000,  0.000000,  1.000000])
        # self.lidar_rotation = np.array([1.10, 0.0 , 2.030])
        height = 1.7
        # the bbox is [-w/2,w/2] x [-h/2,h/2] x [0,height], see PhysicalObject.py
        z_3d -= height/2
        pose = ObjectPose(t=0,x=x_3d,y=y_3d,z=z_3d,yaw=0,pitch=0,roll=0,frame=ObjectFrameEnum.CURRENT)
        dims = [1,1,height]
        return AgentState(pose=pose,dimensions=dims,outline=None,type=AgentEnum.PEDESTRIAN,activity=AgentActivityEnum.MOVING,velocity=(0,0,0),yaw_rate=0)

    def detect_agents(self):
        detection_result = self.detector(self.zed_image,verbose=False)[0]
        boxes = detection_result.boxes
        self.last_person_boxes = []
        for i, class_idx in enumerate(boxes.cls):
            if class_idx == PERSON_INDEX:
                self.last_person_boxes.append(boxes.xywh[i].tolist())
        #TODO: create point clouds in image frame and world frame
        # in order to get pmatrix, we first need the camera intrinsics(fx,fy,cx,cy)
        # we can get these from the camera_info message but we first need to convert the camera info into a 
        image_geometry = PinholeCameraModel()
        image_geometry.fromCameraInfo(self.camera_info)
        fx, fy, cx, cy = image_geometry.fx(), image_geometry.fy(), image_geometry.cx(), image_geometry.cy()
        p_matrix = Pmatrix(fx, fy, cx, cy)
        # we also need the range of the image
        xrange = (0, self.camera_info.width)
        yrange = (0, self.camera_info.height)
        # we might need to convert the point cloud into the camera frame we can use the T_lidar_to_zed matrix to convert the point cloud into the camera frame
        # however I'm not sure when/if we need to do this
        point_cloud = np.concatenate((self.point_cloud[:,:3],np.ones((self.point_cloud.shape[0],1))),axis=1)
        point_cloud = (np.dot(self.T_lidar_to_zed, point_cloud.T).T)[:,:3]
        point_cloud_image, image_indices =  project_point_cloud(point_cloud, p_matrix, xrange, yrange)
        point_cloud_image_world = point_cloud[image_indices]
        point_cloud_image_world = np.concatenate((point_cloud_image_world,np.ones((point_cloud_image_world.shape[0],1))),axis=1)
        point_cloud_image_world = (np.dot(self.T_zed, point_cloud_image_world.T).T)[:,:3]

        self._point_cloud_image = point_cloud_image
        self._point_cloud_image_world = point_cloud_image_world

        detected_agents = []
        for i,b in enumerate(self.last_person_boxes):
            agent = self.box_to_agent(b, point_cloud_image, point_cloud_image_world)
            detected_agents.append(agent)
        return detected_agents
    
    def track_agents(self, vehicle : VehicleState, detected_agents):
        """Given a list of detected agents, updates the state of the agents."""
        # TODO: keep track of which pedestrians were detected before using last_agent_states.
        # use these to assign their ids and estimate velocities.
        results = {}
        for detection in detected_agents:
            detected = False
            for previous_detection_name in self.last_agent_states:
                start_frame = self.last_agent_states[previous_detection_name]
                # start_frame = start_frame.to_frame(1, VehicleState.pose)
                current_frame = copy.deepcopy(detection)
                current_frame.pose.x += vehicle.v * self.rate()
                if collisions.polygon_intersects_polygon_2d(current_frame.polygon_parent(), start_frame.polygon_parent()):
                    results[previous_detection_name] = detection
                    velocity = ((current_frame.pose.x - start_frame.pose.x) * self.rate(),
                                (current_frame.pose.y - start_frame.pose.y) * self.rate(),
                                (current_frame.pose.z - start_frame.pose.z) * self.rate())
                    detection.velocity = velocity
                    detected = True
            if not detected:
                detection.velocity = (0,0,0)
                results['ped'+str(self.pedestrian_counter)] = detection
                self.pedestrian_counter += 1
        return results

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
        self.detect_agents()

    def point_cloud_image(self):
        return self._point_cloud_image, self._point_cloud_image_world