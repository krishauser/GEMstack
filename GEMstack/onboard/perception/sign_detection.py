from ...state import VehicleState,Roadgraph,ObjectPose,SignState,SignEnum,SignalLightEnum,SignalLightState,Sign
from ...utils import settings
from ...mathutils import collisions
from ..interface.gem import GEMInterface
from ..component import Component
from .object_detection import ObjectDetector

from ultralytics import YOLO
import cv2
import numpy as np
from typing import Dict
import threading
import copy
import timeit

sign_dict = dict([
    (3, SignEnum.NO_LEFT_TURN),
    (4, SignEnum.NO_RIGHT_TURN),
    (5, SignEnum.NO_U_TURN),
    (9, SignEnum.NO_PARKING),
    (11, SignEnum.PEDESTRIAN_CROSSING),
    (13, SignEnum.RAILROAD_CROSSING),
    (15, SignEnum.STOP_SIGN)
])

signal_dict = dict([
    (7, SignalLightEnum.GREEN),
    (14, SignalLightEnum.RED),
    (20, SignalLightEnum.YELLOW)
])


class SignDetector(Component):
    """Detects road signs and traffic signals."""

    def __init__(self, vehicle_interface : GEMInterface):
        self.vehicle_interface = vehicle_interface

        self.camera_image = None
        self.lidar_point_cloud = None
        
        self.detector = YOLO(settings.get('perception.sign_detection.model'))
        
        self.object_detector = None
        self.counter = np.zeros(len(sign_dict) + len(signal_dict), dtype=int)

    def rate(self):
        return settings.get('perception.sign_detection.rate')
    
    def state_inputs(self):
        return ['vehicle', 'roadgraph']
    
    def state_outputs(self):
        return ['detected_signs']

    def initialize(self):
        # use image_callback whenever 'front_camera' gets a reading, and it expects images of type cv2.Mat
        self.vehicle_interface.subscribe_sensor('front_camera', self.image_callback, cv2.Mat)
        
        # use lidar_callback whenever 'top_lidar' gets a reading, and it expects numpy arrays
        self.vehicle_interface.subscribe_sensor('top_lidar', self.lidar_callback, np.ndarray)
    
    def image_callback(self, image : cv2.Mat):
        self.camera_image = image

    def lidar_callback(self, point_cloud: np.ndarray):
        self.lidar_point_cloud = point_cloud
    
    def update(self, vehicle : VehicleState, roadgraph : Roadgraph) -> Dict[str,Sign]:
        if self.camera_image is None or self.lidar_point_cloud is None:
            return {}   # no image data or lidar data yet
        
        # debugging
        # self.save_data()

        self.object_detector = ObjectDetector(vehicle, self.camera_image, self.lidar_point_cloud, self.detector)

        t1 = timeit.default_timer()

        detected_signs, names = self.detect_signs()
        
        t2 = timeit.default_timer()
        print('Sign detection time: {:.6f} s'.format(t2 - t1))

        return dict(zip(names, detected_signs))
    
    def object_to_sign(self, detected_object, bbox_cls):
        type = sign_dict[bbox_cls]

        sign = Sign(pose=detected_object.pose, dimensions=detected_object.dimensions, outline=None, 
                    type=type, entities=[])

        return sign, type

    def object_to_signal(self, detected_object, bbox_cls):
        type = signal_dict[bbox_cls]

        signal_state = SignalLightState(state=type, duration=1/self.rate())
        sign = Sign(pose=detected_object.pose, dimensions=detected_object.dimensions, outline=None, 
                    type=SignEnum.STOP_LIGHT, entities=[], state=SignState(signal_state=signal_state))
        
        return sign, type

    def detect_signs(self):
        class_ids = list(sign_dict.keys()) + list(signal_dict.keys())
        detected_objects, bbox_classes = self.object_detector.detect_objects(class_ids)

        detected_signs = []
        names = []
        for i in range(len(detected_objects)):
            cls = int(bbox_classes[i])
            
            if cls in list(sign_dict.keys()): # road sign
                sign, type = self.object_to_sign(detected_objects[i], cls)
                detected_signs.append(sign)

                c_idx = list(sign_dict.values()).index(type)
                self.counter[c_idx] += 1
                names.append(SignEnum(type).name.lower() + str(self.counter[c_idx]))
            
            else: # traffic signal
                signal, type = self.object_to_signal(detected_objects[i], cls)
                detected_signs.append(signal)

                c_idx = list(signal_dict.values()).index(type) + len(sign_dict)
                self.counter[c_idx] += 1
                names.append(SignalLightEnum(type).name.lower() + str(self.counter[c_idx]))
        
        return detected_signs, names


class OmniscientSignDetector(Component):
    """Obtains road sign & traffic signal detections from a simulator"""
    def __init__(self,vehicle_interface : GEMInterface):
        self.vehicle_interface = vehicle_interface
        self.signs = {}
        self.lock = threading.Lock()

    def rate(self):
        return settings.get('perception.sign_detection.rate')
    
    def state_inputs(self):
        return []
    
    def state_outputs(self):
        return ['detected_signs']

    def initialize(self):
        self.vehicle_interface.subscribe_sensor('sign_detector',self.sign_callback, Sign)
    
    def sign_callback(self, name : str, sign : Sign):
        with self.lock:
            self.signs[name] = sign

    def update(self) -> Dict[str,Sign]:
        with self.lock:
            return copy.deepcopy(self.signs)
