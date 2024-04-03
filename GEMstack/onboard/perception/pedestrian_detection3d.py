from ...state import AllState,VehicleState,ObjectPose,ObjectFrameEnum,AgentState,AgentEnum,AgentActivityEnum
from ..interface.gem import GEMInterface
from ..component import Component
from ultralytics import YOLO
import cv2
from typing import Dict
import numpy as np
from sensor_msgs.msg import CameraInfo
from ...utils import settings
import copy
import uuid
from ...mathutils import collisions
index_person = 0

class PedestrianDetector3D(Component):
    """Detects pedestrians."""
    def __init__(self,vehicle_interface : GEMInterface):
        self.vehicle_interface = vehicle_interface
        self.detector = YOLO('./GEMstack/knowledge/detection/yolov8n.pt')
        self.camera_info = None
        self.image = None
        self.detected_pedestrians = None
        self.camera_info = None
        self.camera_range = ()
        self.person_count = 0
        global SETTINGS_LOC
        global SENSOR_FRONT_CAMERA
        global SENSOR_LIDAR
        global HUMAN_HEIGHT
        HUMAN_HEIGHT =1.95
        SETTINGS_LOC = 'vehicle.calibration'
        SENSOR_LIDAR = 'top_lidar'
        SENSOR_FRONT_CAMERA = 'front_camera'
        self.lidar_points = None
        self.point_cloud_zed = None
        self.ped_state = {}
        self.make_camera_info()
        self.make_transformation_matrices()


    def box_to_agent(self,box, pc2d, pc3d):
        """Creates an agent state from an (x,y,w,h) bounding box.
        
        The location and size are pretty much meaningless since this is just giving a 2D location.
        """
        x,y,w,h = box

        dist_2 = np.sum((pc2d - [x+(w/2), y+(h/2)])**2, axis=1)
        closest_ind = np.argmin(dist_2)
        closest_point = pc3d[closest_ind]
        pose = ObjectPose(t=0,x=closest_point[0],y=closest_point[1],z= 0,yaw=0,pitch=0,roll=0,frame=ObjectFrameEnum.CURRENT)
        dims = (1,1,HUMAN_HEIGHT)
        return AgentState(pose=pose,dimensions=dims,outline=None,type=AgentEnum.PEDESTRIAN,activity=AgentActivityEnum.MOVING,velocity=(0,0,0),yaw_rate=0)


    def make_transformation_matrices(self) -> None : 
        self.lidar_translation = np.array(settings.get(SETTINGS_LOC + '.' + SENSOR_LIDAR + '.position'))
        self.lidar_rotation = np.array(settings.get(SETTINGS_LOC + '.' + SENSOR_LIDAR + '.rotation'))
        self.zed_translation = np.array(settings.get(SETTINGS_LOC + '.' + SENSOR_FRONT_CAMERA + '.center_position'))
        self.zed_rotation = np.array(settings.get(SETTINGS_LOC + '.' + SENSOR_FRONT_CAMERA + '.rotation'))
        self.lidar_transformation= np.hstack((self.lidar_rotation, self.lidar_translation.reshape(-1, 1)))
        self.lidar_transformation= np.vstack((self.lidar_transformation, np.array([0, 0, 0, 1])))
        self.zed_tranformation= np.hstack((self.zed_rotation, self.zed_translation.reshape(-1, 1)))
        self.zed_tranformation= np.vstack((self.zed_tranformation, np.array([0, 0, 0, 1])))
        self.lidar_to_zed = np.linalg.inv(self.zed_tranformation) @ self.lidar_transformation # essentially ICP matrix

    def make_camera_info(self) -> None:
        fx = settings.get(SETTINGS_LOC + '.' + SENSOR_FRONT_CAMERA + ".rgb_calibration.fx")
        fy = settings.get(SETTINGS_LOC + '.' + SENSOR_FRONT_CAMERA + ".rgb_calibration.fy")
        cx = settings.get(SETTINGS_LOC + '.' + SENSOR_FRONT_CAMERA + ".rgb_calibration.cx")
        cy = settings.get(SETTINGS_LOC + '.' + SENSOR_FRONT_CAMERA + ".rgb_calibration.cy")
        w = settings.get(SETTINGS_LOC + '.' + SENSOR_FRONT_CAMERA + ".rgb_image.width")
        h = settings.get(SETTINGS_LOC + '.' + SENSOR_FRONT_CAMERA + ".rgb_image.height")
        self.camera_info = np.array([[fx, 0, cx, 0],[0, fy, cy, 0],[0, 0, 1, 0]])
        self.camera_range = (w, h)

    def rate(self):
        return settings.get('longitudinal_planning.rate')
    
    def state_inputs(self):
        return ['vehicle']
    
    def state_outputs(self):
        return ['agents']
    
    def initialize(self):
        
        self.vehicle_interface.subscribe_sensor('front_camera',self.image_callback,cv2.Mat) # subscribe to zed
        self.vehicle_interface.subscribe_sensor('top_lidar',self.lidar_callback,np.ndarray) # subscribe to lidar
        
        pass
    
    def image_callback(self, image : cv2.Mat):
        self.image = image
    
    def lidar_callback(self, lidar_pts: np.ndarray) :
        self.lidar_points = lidar_pts

    
    def update(self, vehicle : VehicleState) -> Dict[str,AgentState]:
        if self.image is None and self.lidar_points is None:
            return dict()

        detected_agents = self.detect_agents()

        current_agent_states = {}
        for agent in detected_agents:
            polygon_collision = False
            for previous_detection_name in self.ped_state:
                last_agent = self.ped_state[previous_detection_name]
                # start_frame = start_frame.to_frame(1, VehicleState.pose)
                current_agent = copy.deepcopy(agent)
                current_agent.pose.x += vehicle.v * self.rate()
                if collisions.polygon_intersects_polygon_2d(current_agent.polygon_parent(), last_agent.polygon_parent()):
                    current_agent_states[previous_detection_name] = agent
                    velocity = ((current_agent.pose.x - last_agent.pose.x) * self.rate(),\
                                (current_agent.pose.y - last_agent.pose.y) * self.rate(),\
                                (current_agent.pose.z - last_agent.pose.z) * self.rate())
                    agent.velocity = velocity
                    polygon_collision = True
            if not polygon_collision:
                agent.velocity = (0,0,0)
                current_agent_states['person-'+str(self.person_count)] = agent

        self.ped_state = current_agent_states
        return current_agent_states
    
    def detect_agents(self):

        # detect pedestrians

        detection_result = self.detector(self.image,verbose=False)[0]
        boxes = detection_result.boxes
        self.detected_pedestrians = []
        for i, class_idx in enumerate(boxes.cls):
            if class_idx == 0:
                self.detected_pedestrians.append(boxes.xywh[i].tolist())

        image_coor_x = (0, self.camera_range[0])
        image_coor_y = (0, self.camera_range[1])

        # prepare point cloud points for calibration
        point_cloud = np.hstack(\
            (self.lidar_points[:,:3],\
            np.array([ 1 for x in range(self.lidar_points.shape[0])]).reshape(-1, 1)))
        
        # calibrate lidar points to zed
        point_cloud_as_zed = (np.dot(self.lidar_to_zed, point_cloud.T).T)[:,:3]

        # assign ids to indentify lidar points
        pc_with_ids_from_lidar = np.hstack((\
            point_cloud_as_zed,\
            np.array([ [x] for x in range(point_cloud_as_zed.shape[0])])\
            ))
        
        # crop off lidar points behind the camera hemisphere
        pc_zed_hemisphere = pc_with_ids_from_lidar[pc_with_ids_from_lidar[:,2] > 0] # in front of the car
        
        # magnify lidar (x,y,z) to zed dimensions
        zed_points = pc_zed_hemisphere[:,:3].dot(self.camera_info[:3,:3].T)

        # flatten zed points to form 2d image
        zed_points_depth_magnified = (zed_points[:,0:2].T/zed_points[:,2]).T

        # select points in zed image range
        inds = np.logical_and(\
            np.logical_and(zed_points_depth_magnified[:,0] >= image_coor_x[0],zed_points_depth_magnified[:,0] < image_coor_x[1]),\
            np.logical_and(zed_points_depth_magnified[:,1] >= image_coor_y[0],zed_points_depth_magnified[:,1] < image_coor_y[1]))
        
        point_cloud_image_2d = zed_points_depth_magnified[inds]
        ids_on_lidar = pc_zed_hemisphere[inds,3].astype(int)
        
        point_cloud_mapped_in_zed_3d = point_cloud_as_zed[ids_on_lidar]

        point_cloud_mapped_in_zed_3d = \
            np.concatenate(\
                (point_cloud_mapped_in_zed_3d,\
                 np.array([ 1 for x in range(point_cloud_mapped_in_zed_3d.shape[0])]).reshape(-1, 1)),axis=1)
        point_cloud_mapped_in_zed_3d = (np.dot(self.zed_tranformation, point_cloud_mapped_in_zed_3d.T).T)[:,:3]

        self.point_cloud_image_2d = point_cloud_image_2d
        self.point_cloud_image_2d = point_cloud_mapped_in_zed_3d

        detected_agents = []
        for i,box in enumerate(self.detected_pedestrians):
            agent = self.box_to_agent(box, point_cloud_image_2d, point_cloud_mapped_in_zed_3d)
            detected_agents.append(agent)
        return detected_agents

class FakePedestrianDetector2D(Component):
    """Triggers a pedestrian detection at some random time ranges"""
    def __init__(self,vehicle_interface : GEMInterface):
        self.vehicle_interface = vehicle_interface
        self.times = [(5.0,20.0),(30.0,35.0)]
        self.t_start = None

    def rate(self):
        return settings.get("longitudinal_planning.rate")
    
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
                # res['pedestrian0'] = box_to_fake_agent((0,0,0,0))
                print("Detected a pedestrian")
        return res