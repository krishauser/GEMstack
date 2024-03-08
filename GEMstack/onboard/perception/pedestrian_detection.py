from ...state import AllState,VehicleState,ObjectPose,ObjectFrameEnum,AgentState,AgentEnum,AgentActivityEnum
from ...utils import settings
from ...mathutils import transforms, collisions
from ..interface.gem import GEMInterface
from ..component import Component
from ultralytics import YOLO
import cv2
from sensor_msgs.msg import CameraInfo
from image_geometry import PinholeCameraModel
import rospy
import numpy as np
from typing import Dict,Tuple, List
import time, copy

MAX_HUMAN_FATNESS_IN_M = 0.75

'''
lidar to zed = lidar frame to camera frame
lidar = lidar to world
zed = zed to world
'''
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
        #self.T_lidar_to_zed = np.linalg.inv(self.T_zed) @ self.T_lidar
        #todo load from file?
        self.T_lidar_to_zed = np.array([[-0.00115108, -0.99996643,  0.00811196, -0.02388523], [-0.0053156,  -0.00810573, -0.99995302,  0.01848758], [ 0.99998521, -0.00119414, -0.00530609, -0.01805329],[ 0.,0.,0.,1.]])
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
    
    def image_callback(self, image : cv2.Mat):
        self.zed_image = image
        # self.update(self.vehicle_interface.get_state('vehicle'))

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

    def box_to_agent(self, box, point_cloud_image, point_cloud_image_world): # TODO
        """Creates a 3D agent state from an (x,y,w,h) bounding box.
        
        TODO: you need to use the image, the camera intrinsics, the lidar
        point cloud, and the calibrated camera / lidar poses to get a good
        estimate of the pedestrian's pose and dimensions.
        """
        # yeah idk what to do here
        x,y,w,h = box
        # camera intrinsics
        camera = PinholeCameraModel()
        camera.fromCameraInfo(self.camera_info)
        fx = camera.fx()
        fy = camera.fy()
        cx = camera.cx()
        cy = camera.cy()

        p_matrix = Pmatrix(fx, fy, cx, cy)
        xrange = (0, self.camera_info.width)
        yrange = (0, self.camera_info.height)

        # assume x, y is the top left of bbx?
        # goal: change camera frame box to world frame box?
        # and then get the z correct

        # # 4 corners of the box
        # corners = [[x, y, 1, 1], [x+w, y, 1, 1], [x+w, y+h, 1, 1], [x, y+h, 1, 1]]
        # corners = np.array(corners)

        # the points that are going to be on simon are the nearest points to his stomach

        # center_of_guy 

        # our_point_indices = []
        person_lidar_points = []
        for i in range(point_cloud_image.shape[0]):
            point = point_cloud_image[i, :]
            if point[0] >= x - (w/2) and point[0] <= x+(w/2) and point[1] >= y -(h/2)and point[1] <= y+(h/2):
                # our_point_indices.append(i)
                try:
                    person_lidar_points.append(point_cloud_image_world[i])
                except:
                    pass

        person_lidar_points = np.array(person_lidar_points)

        # PErson is at the front of the img
        new_x = np.min(person_lidar_points[:, 0])
        new_y = np.min(person_lidar_points[:, 1])
        new_z = np.min(person_lidar_points[:, 2])
        new_depth = MAX_HUMAN_FATNESS_IN_M
        new_width = MAX_HUMAN_FATNESS_IN_M
        new_height = 1.7 # given constant
        

        # x is forward in the object's frame
        # y is left in the object's frame
        # z is up in the object's frame
        pose = ObjectPose(t=0,x=new_x,y=new_y,z=new_z,yaw=0,pitch=0,roll=0,frame=ObjectFrameEnum.CURRENT)
        dims = [new_depth, new_width, new_height]
        return AgentState(pose=pose,dimensions=dims,outline=None,type=AgentEnum.PEDESTRIAN,activity=AgentActivityEnum.MOVING,velocity=(0,0,0),yaw_rate=0)

    def detect_agents(self): # TODO
        print(self.T_lidar_to_zed)
        detection_result = self.detector(self.zed_image,verbose=False)
        self.last_person_boxes = []
        #create boxes from detection result, copied from hw2
        boxes = detection_result[0].boxes
        for i in range(len(boxes)):
            if(boxes.cls[i] == 0):
                self.last_person_boxes.append(boxes.xywh[i].tolist())

        #create point clouds in image frame and world frame
        camera = PinholeCameraModel()
        camera.fromCameraInfo(self.camera_info)

        fx = camera.fx()
        fy = camera.fy()
        cx = camera.cx()
        cy = camera.cy()

        p_matrix = Pmatrix(fx, fy, cx, cy)
        xrange = (0, self.camera_info.width)
        yrange = (0, self.camera_info.height)

        point_cloud = np.concatenate((self.point_cloud[:,:3],np.ones((self.point_cloud.shape[0],1))),axis=1)
        point_cloud = (np.dot(self.T_lidar_to_zed, point_cloud.T).T)[:,:3] # move the pc to image frame

        point_cloud_image, image_indices = project_point_cloud(point_cloud, p_matrix, xrange, yrange)


        # lecture 7 slide 24 says pciw[i, j, k] should be (x, y, 1). 
        # copilot wrote the below lines, gotta test if they work
        point_cloud_image_world = point_cloud[image_indices] # just the pc that's in the img
        ones = np.ones((point_cloud_image_world.shape[0], 1))
        point_cloud_image_world = np.hstack((point_cloud_image_world, ones)) 
        point_cloud_image_world = (np.dot(self.T_zed, point_cloud_image_world.T).T)[:,:3]


        detected_agents = []
        for i,b in enumerate(self.last_person_boxes):
            agent = self.box_to_agent(b, point_cloud_image, point_cloud_image_world)
            detected_agents.append(agent)


        # set some variables for l8r
        self._point_cloud_image = point_cloud_image
        self._point_cloud_image_world = point_cloud_image_world

        return detected_agents
    
    def track_agents(self, vehicle : VehicleState, detected_agents : List[AgentState]):
        """Given a list of detected agents, updates the state of the agents."""
        # keep track of which pedestrians were detected before using last_agent_states.
        # use these to assign their ids and estimate velocities.
        
        dt = self.rate()
        results = {}
        for agent in detected_agents:
            # agent has pose and dimensions
            new_agent = True
            # figure out if the agent is new or previously detected
            for s in self.last_agent_states:
                name, past_agent = s, self.last_agent_states[s]
                previous_pose = past_agent
                current_pose = copy.deepcopy(agent)
                current_pose.pose.x += vehicle.v * dt

                prev_polygon = past_agent.polygon_parent()
                current_polygon = agent.polygon_parent()
                if collisions.polygon_intersects_polygon_2d(prev_polygon, current_polygon):
                    results[name] = agent
                    v = ((current_pose.pose.x - previous_pose.pose.x) * dt,
                                (current_pose.pose.y - previous_pose.pose.y) * dt,
                                (current_pose.pose.z - previous_pose.pose.z) * dt)
                    agent.velocity = v
                    new_agent = False           

            if new_agent:
                results['pedestrian_'+str(self.pedestrian_counter)] = agent
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

    def point_cloud_image(self):
        return self._point_cloud_image, self._point_cloud_image_world