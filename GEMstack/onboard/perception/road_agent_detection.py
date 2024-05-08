from ...state import AllState,VehicleState,ObjectPose,ObjectFrameEnum,AgentState,AgentEnum,AgentActivityEnum
from ...utils import settings
from ..interface.gem import GEMInterface
from ..component import Component
from .object_detection import ObjectDetector

from ultralytics import YOLO
from typing import Dict, List
import threading
import copy
import cv2
import numpy as np
import timeit

agent_dict = {
    '0': AgentEnum.PEDESTRIAN,
    '1': AgentEnum.BICYCLIST,
    '2': AgentEnum.CAR,
    '7': lambda z : AgentEnum.MEDIUM_TRUCK if z <= 2.0 else AgentEnum.LARGE_TRUCK
}


class AgentDetector(Component):
    """Detects other agents (pedestrians and vehicles)."""

    def __init__(self, vehicle_interface : GEMInterface):
        self.vehicle_interface = vehicle_interface

        self.camera_image = None
        self.lidar_point_cloud = None
        
        self.detector = YOLO(settings.get('perception.agent_detection.model'))
        
        self.object_detector = None
        self.counter = np.zeros(5, dtype=int)

    def rate(self):
        return settings.get('perception.agent_detection.rate')
    
    def state_inputs(self):
        return ['vehicle']
    
    def state_outputs(self):
        return ['agents']

    def initialize(self):
        # use image_callback whenever 'front_camera' gets a reading, and it expects images of type cv2.Mat
        self.vehicle_interface.subscribe_sensor('front_camera', self.image_callback, cv2.Mat)
        
        # use lidar_callback whenever 'top_lidar' gets a reading, and it expects numpy arrays
        self.vehicle_interface.subscribe_sensor('top_lidar', self.lidar_callback, np.ndarray)
    
    def image_callback(self, image : cv2.Mat):
        self.camera_image = image

    def lidar_callback(self, point_cloud: np.ndarray):
        self.lidar_point_cloud = point_cloud
    
    def update(self, vehicle : VehicleState) -> Dict[str,AgentState]:
        if self.camera_image is None or self.lidar_point_cloud is None:
            return {}   # no image data or lidar data yet
        
        # debugging
        # self.save_data()

        self.object_detector = ObjectDetector(vehicle, self.camera_image, self.lidar_point_cloud, self.detector)

        t1 = timeit.default_timer()

        detected_agents, names = self.detect_agents()
        
        t2 = timeit.default_timer()
        print('Agent detection time: {:.6f} s'.format(t2 - t1))

        return dict(zip(names, detected_agents))

    def object_to_agent(self, detected_object, bbox_cls):
        """Creates a 3D agent state from a PhysicalObject."""
        
        dims = detected_object.dimensions
        agent_type = agent_dict[str(bbox_cls)]
        if bbox_cls == 7:
            agent_type = type(dims[2])

        # set agent type based on class id
        agent = AgentState(pose=detected_object.pose, dimensions=dims, outline=None, 
                           type=agent_type, activity=AgentActivityEnum.STOPPED, 
                           velocity=(0,0,0), yaw_rate=0, attributes=None)

        return agent, agent_type

    def detect_agents(self):
        """Creates a list of AgentState objects."""
        yolo_class_ids = list(map(int, agent_dict.keys()))
        detected_objects, bbox_classes = self.object_detector.detect_objects(yolo_class_ids)

        detected_agents = []
        names = []
        for i in range(len(detected_objects)):
            cls = int(bbox_classes[i])

            agent, agent_type = self.object_to_agent(detected_objects[i], cls)
            detected_agents.append(agent)

            c_idx = list(agent_dict.values()).index(agent_type)
            self.counter[c_idx] += 1
            names.append(AgentEnum(agent_type).name.lower() + str(self.counter[c_idx]))

        return detected_agents, names


class OmniscientAgentDetector(Component):
    """Obtains agent detections from a simulator"""
    def __init__(self,vehicle_interface : GEMInterface):
        self.vehicle_interface = vehicle_interface
        self.agents = {}
        self.lock = threading.Lock()

    def rate(self):
        return settings.get('perception.agent_detection.rate')

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
