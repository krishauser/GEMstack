from ...state import AllState,VehicleState,ObjectPose,ObjectFrameEnum,AgentState,AgentEnum,AgentActivityEnum
from ...utils import settings
from ...mathutils import transforms, collisions
from ..interface.gem import GEMInterface
from ..component import Component
from ultralytics import YOLO
import cv2
import pickle
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
    print("uv:", uv)
    print("inds:", inds)
    print(np.sum(inds))
    point_cloud_image = uv[inds]
    image_indices = pc_fwd[inds,3].astype(int)
    return point_cloud_image, image_indices


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

    def rate(self):
        return 4.0
    
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
        x, y, w, h = box

        print('point_cloud_image:', point_cloud_image)

        indices_in_box = []
        for i in range(len(point_cloud_image)):
            if x - w / 2 <= point_cloud_image[i][0] <= x + w / 2 and y - w / 2 <= point_cloud_image[i][1] <= y + h / 2:
                indices_in_box.append(i)

        points_in_box = [point_cloud_image_world[i] for i in indices_in_box]
        print('points_in_box:', points_in_box)

        position = np.mean(points_in_box, axis=0)
        print('position:', position)

        pose = ObjectPose(t=0, x=position[0], y=position[1], z=position[2], yaw=0, pitch=0,
                          roll=0, frame=ObjectFrameEnum.CURRENT)

        dims = np.max(points_in_box, axis=0) - np.min(points_in_box, axis=0)

        return AgentState(pose=pose, dimensions=dims, outline=None,
                          type=AgentEnum.PEDESTRIAN, activity=AgentActivityEnum.MOVING, velocity=(0, 0, 0), yaw_rate=0)

    def get_point_cloud_image_world(self, image_indices, point_cloud):
        point_cloud_camera_frame = point_cloud[image_indices]

        # Change camera frame to world frame
        point_cloud_camera = np.ones((4, len(point_cloud_camera_frame)))
        point_cloud_camera[:3, :] = point_cloud_camera_frame.transpose()
        point_cloud_image_world = (self.T_zed @ point_cloud_camera).T[:, :3]

        return point_cloud_image_world


    def detect_agents(self):
        detection_result = self.detector(self.zed_image,verbose=False, classes=0)
        self.last_person_boxes = []

        # Create boxes from detection result
        people = detection_result[0]
        for box in people.boxes:
            self.last_person_boxes.append(box.xywh[0].tolist())

        # TODO: create point clouds in image frame and world frame
        from collections import namedtuple
        CameraInfo = namedtuple('CameraInfo',['width','height','P'])
        #TODO: these are guessed parameters
        #with open("GEMstack/knowledge/calibration/values.pickle", 'rb') as f:
        #    camera_info = pickle.load(f)
        camera_info = settings.get('camera_info')
        zed_intrinsics = [camera_info['fx'], camera_info['fy'], camera_info['cx'], camera_info['cy']]
        zed_w = camera_info['width']
        zed_h = camera_info['height']
        P_M = Pmatrix(camera_info['fx'],camera_info['fy'],camera_info['cx'],camera_info['cy'])
        self.camera_info = CameraInfo(width=zed_w, height=zed_h, P=P_M)
        print(self.camera_info.P)
        print(self.point_cloud)

        point_cloud_homogeneous = np.hstack((self.point_cloud, np.ones((self.point_cloud.shape[0], 1))))
        point_cloud_camera_frame = (self.T_lidar_to_zed @ point_cloud_homogeneous.T).T[:, :3]


        point_cloud_image, image_indices = project_point_cloud(point_cloud_camera_frame,
                                                               self.camera_info.P, (0, self.camera_info.width),
                                                               (0, self.camera_info.height))
        print(point_cloud_image)
        point_cloud_image_world = self.get_point_cloud_image_world(image_indices, point_cloud_camera_frame)
        detected_agents = []
        for i, b in enumerate(self.last_person_boxes):
            agent = self.box_to_agent(b, point_cloud_image, point_cloud_image_world)
            detected_agents.append(agent)
            
        return detected_agents
    
    def track_agents(self, vehicle : VehicleState, detected_agents : List[AgentState]):
        """Given a list of detected agents, updates the state of the agents."""
        # TODO: keep track of which pedestrians were detected before using last_agent_states.
        # use these to assign their ids and estimate velocities.
        results = {}
        delta_time = 1.0
        for agent in detected_agents:
            tracked = False
            #agent.pose.x += vehicle.v * delta_time
            for id, last_agent in self.last_agent_states.items():
                if collisions.polygon_intersects_polygon_2d(agent.polygon_parent(), last_agent.polygon_parent()):
                    velocity_x = (agent.pose.x - last_agent.pose.x) / delta_time
                    velocity_y = (agent.pose.y - last_agent.pose.y) / delta_time
                    velocity_z = (agent.pose.z - last_agent.pose.z) / delta_time
                    if velocity_x != 0 or velocity_y != 0 or velocity_z != 0:
                        activity = AgentActivityEnum.MOVING
                    else:
                        activity = AgentActivityEnum.STOPPED

                    new_agent = AgentState(pose=agent.pose, dimensions=agent.dimensions,
                                           outline=agent.outline, type=agent.type,
                                           activity=activity, velocity=(velocity_x, velocity_y, velocity_z),
                                           yaw_rate=agent.yaw_rate)
                    results[id] = new_agent
                    tracked = True
                    break

            if not tracked:
                self.pedestrian_counter += 1
                results['ped'+str(self.pedestrian_counter)] = agent
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
        print('##############################')
        print('load data called')
        prefix = ''
        if loc is not None:
            prefix = loc + '/'
        self.zed_image = cv2.imread(prefix+'zed_image.png')
        self.point_cloud = np.load(prefix+'velodyne_point_cloud.npz')['arr_0']
        try:
            import pickle
            with open(prefix+'zed_camera_info.pkl','rb') as f:
                self.camera_info = pickle.load(f)
            print('##############################')
            print('In try segment')
            print('##############################')
            print(self.camera_info)
        except ModuleNotFoundError:
            print('##############################')
            print("In load fail")
            print('##############################')
            #ros not found?
            from collections import namedtuple
            CameraInfo = namedtuple('CameraInfo',['width','height','P'])
            #TODO: these are guessed parameters
            #with open("GEMstack/knowledge/calibration/values.pickle", 'rb') as f:
            #    camera_info = pickle.load(f)
            camera_info = settings.get('camera_info')
            zed_intrinsics = [camera_info['fx'], camera_info['fy'], camera_info['cx'], camera_info['cy']]
            zed_w = camera_info['width']
            zed_h = camera_info['height']
            P_M = self.Pmatrix(camera_info['fx'],camera_info['fy'],camera_info['cx'],camera_info['cy'])
            self.camera_info = CameraInfo(width=zed_w, height=zed_h, P=P_M)

            with open(prefix+'zed_camera_info.pkl','wb') as f:
                pickle.dump(self.camera_info)
