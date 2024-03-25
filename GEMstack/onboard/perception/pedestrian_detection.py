from ...state import AllState,VehicleState,ObjectPose,ObjectFrameEnum,AgentState,AgentEnum,AgentActivityEnum

from ...utils import settings

from ...mathutils import transforms, collisions

from ..interface.gem import GEMInterface

from ..component import Component

from ultralytics import YOLO

import object_tracking

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

    return np.array([[fx, 0,  cx, 0],

                     [0,  fy, cy, 0],

                     [0,  0,  1,  0]])


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

    
def lidar2pixel (intrinsic, extrinsic, point_cloud):
    point_cloud = np.array(point_cloud)
    homo_matrix = np.concatenate((point_cloud, np.ones((point_cloud.shape[0], 1))), axis=1) # N,4

    lidar_pixel = (intrinsic @ extrinsic @ (homo_matrix).T) # Lidar to pixel coordinate

    lidar_pixel = lidar_pixel.T

    lidar_pixel = lidar_pixel / lidar_pixel[:, [2]]  # Normalize x and y

    lidar_pixel = lidar_pixel[:, :2]

    return lidar_pixel

def lidar2world (lidar_vehicle_matrix, point_cloud):
    point_cloud = np.array(point_cloud)[0]
    homo_matrix = np.concatenate((point_cloud, np.ones((point_cloud.shape[0], 1))), axis=1) # N,4

    pc_world = (lidar_vehicle_matrix @ homo_matrix.T) # 4,N

    pc_world = pc_world.T

    pc_world = pc_world[:, :3]

    return pc_world

class PedestrianDetector(Component):

    """Detects and tracks pedestrians."""

    def __init__(self,vehicle_interface : GEMInterface):

        self.vehicle_interface = vehicle_interface

        self.detector = YOLO(settings.get('pedestrian_detection.model'))

        # self.detector = YOLO('GEMstack/knowledge/detection/yolov8n.pt')

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

        self.prev_agents = {}

        # extrinsic = settings.get('vehicle.calibration.front_camera.extrinsic')
        # [[ 0.35282628 , -0.9356864 ,  0.00213977, -1.42526548],

        #              [-0.04834961 , -0.02051524, -0.99861977, -0.02062586],

        #              [ 0.93443883 ,  0.35223584, -0.05247839, -0.15902421],

        #              [ 0.         ,  0.        ,  0.        ,  1.        ]]

        self.extrinsic = self.T_lidar_to_zed # np.asarray(extrinsic)

        self.fx = 527.5779418945312

        self.cx = 616.2459716796875

        self.fy = 527.5779418945312

        self.cy = 359.2155456542969

        self.P = Pmatrix(self.fx, self.fy, self.cx, self.cy)

        self.intrinsic = self.P 
        print(self.intrinsic)

        T_lidar_to_vehicle = [[ 1,  0,  0., 0.85],
                              [ 0,  1,  0., 0.  ],
                              [ 0., 0., 1., 1.68],
                              [ 0., 0., 0., 1.  ]]

        self.T_lidar_to_vehicle = np.asarray(T_lidar_to_vehicle)

    def rate(self):

        return 4.0

    def state_inputs(self):

        return ['vehicle']

    def state_outputs(self):

        return ['agents']

    def initialize(self):

        #tell the vehicle to use image_callback whenever 'front_camera' gets a reading, and it expects images of type cv2.Mat

        self.vehicle_interface.subscribe_sensor('front_camera',self.image_callback,cv2.Mat)

        # tell the vehicle to use lidar_callback whenever 'top_lidar' gets a reading, and it expects numpy arrays

        self.vehicle_interface.subscribe_sensor('top_lidar',self.lidar_callback,np.ndarray)

        # subscribe to the Zed CameraInfo topic

        self.camera_info_sub = rospy.Subscriber("/zed2/zed_node/rgb/camera_info", CameraInfo, self.camera_info_callback)

    def image_callback(self, image : cv2.Mat):

        self.zed_image = image

    def camera_info_callback(self, info : CameraInfo):

        self.camera_info = info

    def lidar_callback(self, point_cloud: np.ndarray):

        self.point_cloud = point_cloud

    def update(self, vehicle : VehicleState) -> Dict[str,AgentState]:

        print('start')

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

        self.save_data()

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
        # The box boundary
        center_x = x 

        center_y = y 

        b_left, b_right = x - w/2, x + w/2

        b_top, b_bottom = y - h/2, y + h/2

        # The point could in the box

        index = np.where((point_cloud_image[:, 0] >= b_left) & (point_cloud_image[:, 0] <= b_right) 

                            & (point_cloud_image[:, 1] > b_top) & (point_cloud_image[:, 1] < b_bottom))

        point_cloud_agent = point_cloud_image[index]
        
        point_cloud_world_agent = point_cloud_image_world[index]
        idx = np.argsort(point_cloud_world_agent[:,0], axis=0)[:point_cloud_world_agent.shape[0]//3]
        point_cloud_world_agent = point_cloud_world_agent[idx]

        x_3d = point_cloud_world_agent[:, 0].mean()
        y_3d = point_cloud_world_agent[:, 1].mean()
        z_3d = 0

        # The distance from center point to any point

        pose = ObjectPose(t=0,x=x_3d,y=y_3d,z=z_3d,yaw=0,pitch=0,roll=0,frame=ObjectFrameEnum.CURRENT)

        dims = [1, 1, 1.7] # [w,h,l]

        return AgentState(pose=pose,dimensions=dims,outline=None,type=AgentEnum.PEDESTRIAN,activity=AgentActivityEnum.MOVING,velocity=(0,0,0),yaw_rate=0)

    def detect_agents(self):
        detected_object = object_tracking.tracking(self.zed_image)

        detection_result = self.detector(self.zed_image,verbose=False)

        person_boxes = []

        boxes = detection_result[0].boxes

        image_w = 1280

        image_h = 720

        #TODO: create boxes from detection result

        for i in range(len(detection_result[0].boxes)):

            if (int(boxes.cls[i].item()) == 0):

                x, y, w, h = boxes[i].xywh[0].cpu().detach().numpy().tolist()

                print(f'xywh{x, y, w, h}')

                person_boxes.append((x,y,w,h))       

        point_cloud_in_range = []
        filetered_point_cloud = np.where(self.point_cloud[:, 0] > 0)
        self.point_cloud = self.point_cloud[filetered_point_cloud]
        point_cloud_image = lidar2pixel(self.intrinsic, self.extrinsic, self.point_cloud)

        indexs = np.where((point_cloud_image[:, 0] > 0) & (point_cloud_image[:, 0] < image_w) & (point_cloud_image[:, 1] > 0) & (point_cloud_image[:, 1] < image_h))

        for index in indexs:

            point_cloud_in_range.append(self.point_cloud[index])
        
        point_cloud_image = point_cloud_image[indexs]
        point_cloud_image_world = lidar2world(self.T_lidar_to_vehicle, point_cloud_in_range)

        #TODO: create point clouds in image frame and world frame

        detected_agents = []

        for i,b in enumerate(person_boxes):

            agent = self.box_to_agent(b, point_cloud_image, point_cloud_image_world)

            if agent is not None:

                detected_agents.append(agent)

        return detected_agents, detected_object

    def track_agents(self, vehicle : VehicleState, detected_agents : List[AgentState]):

        """Given a list of detected agents, updates the state of the agents."""

        # TODO: keep track of which pedestrians were detected before using last_agent_states.

        # use these to assign their ids and estimate velocities.

        results = {}

        self.pedestrian_counter = 0

        dt = 1.0

        for agent in detected_agents:

            state = False
            for id, prev_agent in self.last_agent_states.items():


                # prev_agent = prev_agent.to_frame(ObjectFrameEnum.CURRENT, vehicle.pose)  # Using the current vehicle coornidate

                prev_poly = prev_agent.polygon_parent()

                poly = agent.polygon_parent()

                if collisions.polygon_intersects_polygon_2d(prev_poly, poly):

                    vx = (agent.pose.x - prev_agent.pose.x) / dt

                    vy = (agent.pose.y - prev_agent.pose.y) / dt

                    vz = (agent.pose.z - prev_agent.pose.z) / dt

                    v = (vx, vy, vz)

                    update_agent = AgentState(pose=agent.pose, dimensions=agent.dimensions,outline=None, type=AgentEnum.PEDESTRIAN,

                                            activity=AgentActivityEnum.MOVING, velocity=v, yaw_rate=agent.yaw_rate)

                    results[id] = update_agent

                    state = True

                    break

            if state == False: # No overlap => new pedestrian

                self.pedestrian_counter += 1

                pedestrian_number = f'pedestrian_{self.pedestrian_counter}'

                new_agent = AgentState(pose=agent.pose, dimensions=agent.dimensions,outline=agent.outline, type=agent.type,

                                            activity=AgentActivityEnum.MOVING, velocity=(0, 0, 0), yaw_rate=0)

                results[pedestrian_number] = new_agent

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

            self.camera_info = CameraInfo(width=1280,height=720,P=[527.5779418945312, 0.0, 616.2459716796875, 0.0, 0.0, 527.5779418945312, 359.2155456542969, 0.0, 0.0, 0.0, 1.0, 0.0])

            # self.camera_info = CameraInfo(width=1280,height=720,P=[560.0,0,640.0,0,  0,560.0,360,0,  0,0,1,0])
