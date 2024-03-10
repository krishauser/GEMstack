from ...state import AllState,VehicleState,ObjectPose,ObjectFrameEnum,AgentState,AgentEnum,AgentActivityEnum
from ...utils import settings
from ...mathutils import transforms, collisions
from ..interface.gem import GEMInterface
from ..component import Component
from ultralytics import YOLO
import cv2, copy
try:
    from sensor_msgs.msg import CameraInfo
    from image_geometry import PinholeCameraModel
    import rospy
except ImportError:
    pass
import numpy as np
from typing import Dict,Tuple,List
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


class PedestrianDetector(Component):
    """Detects and tracks pedestrians."""
    def __init__(self,vehicle_interface : GEMInterface):
        self.vehicle_interface = vehicle_interface
        self.detector = YOLO('GEMstack/knowledge/detection/yolov8n.pt')
        self._rate = settings.get('pedestrian_detection.rate')
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
        self.point_cloud_lidar = None
        self.point_cloud_zed = None
        assert(settings.get('vehicle.calibration.top_lidar.reference') == 'rear_axle_center')
        assert(settings.get('vehicle.calibration.front_camera.reference') == 'rear_axle_center')
        self.pedestrian_counter = 0
        self.last_agent_states = {}
        self.persons_id = []

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
        self.camera_info_sub = rospy.Subscriber("/zed2/zed_node/rgb/camera_info", CameraInfo, self.camera_info_callback)
        
    
    def image_callback(self, image : cv2.Mat):
        self.zed_image = image

    def camera_info_callback(self, info : CameraInfo):
        self.camera_info = info

    def lidar_callback(self, point_cloud: np.ndarray):
        self.point_cloud_lidar = point_cloud
    
    def update(self, vehicle : VehicleState) -> Dict[str,AgentState]:
        if self.zed_image is None:
            #no image data yet
            return {}
        if self.point_cloud_lidar is None:
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

    def box_to_agent(self, box, point_cloud_image, point_cloud_image_world, image_indices):
        """Creates a 3D agent state from an (x,y,w,h) bounding box.
        
        TODO: you need to use the image, the camera intrinsics, the lidar
        point cloud, and the calibrated camera / lidar poses to get a good
        estimate of the pedestrian's pose and dimensions.
        """
        # point_cloud_image #uv, depth stereo i wrote this

        x,y,w,h = box
        u=x+w/2
        v=y+h/2

        mask_indices = (np.argsort(np.linalg.norm(point_cloud_image - [x,y],axis=1)))[:20]
        agent_pcd = point_cloud_image_world[mask_indices]
        agent_close_idx = np.argsort(np.linalg.norm(agent_pcd, axis=1))[:5]
        agent_close_pcd =  agent_pcd[agent_close_idx]
        x_3d, y_3d,z_3d = np.mean(agent_close_pcd, axis=0)
        
        height = 1.7
        z_3d -= height/2
        pose = ObjectPose(t=0,x=x_3d,y=y_3d,z=z_3d,yaw=0,pitch=0,roll=0,frame=ObjectFrameEnum.CURRENT)
        dims = [1,1,1.7]
        
        return AgentState(pose=pose,dimensions=dims,outline=None,type=AgentEnum.PEDESTRIAN,activity=AgentActivityEnum.MOVING,velocity=(0,0,0),yaw_rate=0)

    def detect_agents(self):
        detection_result = self.detector(self.zed_image,verbose=False)[0]
        #TODO: create boxes from detection result
        latest_boxes = detection_result.boxes
        self.last_person_boxes = []

        for i, b in enumerate(latest_boxes.cls):
            if b == 0:
                x,y,w,h = latest_boxes.xywh[i].tolist()
                self.last_person_boxes.append((x, y, w, h))

        image_geometry = PinholeCameraModel()
        image_geometry.fromCameraInfo(self.camera_info)
        fx, fy, cx, cy = image_geometry.fx(), image_geometry.fy(), image_geometry.cx(), image_geometry.cy()
        p_matrix = Pmatrix(fx, fy, cx, cy)
        xrange = (0, self.camera_info.width)
        yrange = (0, self.camera_info.height)

        #TODO: create point clouds in image frame and world frame
        homo_point_cloud_lidar = np.concatenate((self.point_cloud_lidar[:,:3], np.ones((self.point_cloud_lidar.shape[0],1))),axis=1)
        homo_point_cloud_lidar = (np.dot(self.T_lidar_to_zed, homo_point_cloud_lidar.T).T)[:,:3] #nx4

        point_cloud_image, image_indices = project_point_cloud(homo_point_cloud_lidar, p_matrix, xrange, yrange)
        point_cloud_image_world = homo_point_cloud_lidar[image_indices] #nx4
        point_cloud_image_world = np.concatenate((point_cloud_image_world, np.ones((point_cloud_image_world.shape[0], 1))),axis=1)
        point_cloud_image_world = (self.T_zed @ (point_cloud_image_world.T)).T[:,:3]
        
        self._point_cloud_image = point_cloud_image
        self._point_cloud_image_world = point_cloud_image_world
        
        detected_agents = []
        for i,b in enumerate(self.last_person_boxes):
            agent = self.box_to_agent(b, point_cloud_image, point_cloud_image_world, image_indices)
            detected_agents.append(agent)

        return detected_agents
    
    def track_agents(self, vehicle : VehicleState, detected_agents : List[AgentState]):
        """Given a list of detected agents, updates the state of the agents."""
        # TODO: keep track of which pedestrians were detected before using last_agent_states.
        # use these to assign their ids and estimate velocities.

        results = {}
        for agent in detected_agents:
            detected = False
            for previous_agent_name in self.last_agent_states:
                frame_zero = self.last_agent_states[previous_agent_name]
                # frame_zero = frame_zero.to_frame(1, VehicleState.pose)
                frame_curr = copy.deepcopy(agent)
                frame_curr.pose.x += vehicle.v * self.rate()
                if collisions.polygon_intersects_polygon_2d(frame_curr.polygon_parent(), frame_zero.polygon_parent()):
                    results[previous_agent_name] = agent
                    velocity = ((frame_curr.pose.x - frame_zero.pose.x) * self.rate(),
                                (frame_curr.pose.y - frame_zero.pose.y) * self.rate(),
                                (frame_curr.pose.z - frame_zero.pose.z) * self.rate())
                    agent.velocity = velocity
                    detected = True
            if not detected:
                agent.velocity = (0,0,0)
                results['ped'+str(self.pedestrian_counter)] = agent
                self.pedestrian_counter += 1
        return results

    def save_data(self, loc=None):
        """This can be used for debugging.  See the provided test."""
        prefix = ''
        if loc is not None:
            prefix = loc + '/'
        cv2.imwrite(prefix+'zed_image.png',self.zed_image)
        np.savez(prefix+'velodyne_point_cloud.npz',self.point_cloud_lidar)
        import pickle
        with open(prefix+'zed_camera_info.pkl','wb') as f:
            pickle.dump(self.camera_info,f)

    def load_data(self, loc=None):
        prefix = ''
        if loc is not None:
            prefix = loc + '/'
        self.zed_image = cv2.imread(prefix+'zed_image.png')
        self.point_cloud_lidar = np.load(prefix+'velodyne_point_cloud.npz')['arr_0']
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
    
    def point_cloud_image(self):
        return self._point_cloud_image, self._point_cloud_image_world
