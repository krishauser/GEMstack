from ...state import AllState,VehicleState,ObjectPose,SignState,SignEnum,SignalLightEnum,SignalLightState,Sign
from ...utils import settings
from ...mathutils import collisions
from ..interface.gem import GEMInterface
from ..component import Component
from .object_detection import ObjectDetector

from ultralytics import YOLO
import numpy as np
from typing import Dict
import threading
import copy

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


class SignDetector(ObjectDetector):
    """Detects road signs and traffic signals."""

    def __init__(self, vehicle : VehicleState, camera_image, lidar_point_cloud):
        detector = YOLO(settings.get('perception.sign_detection.model'))
        super().__init__(vehicle, camera_image, lidar_point_cloud, detector)
    
    def object_to_sign(self, detected_object, bbox_cls):
        return Sign(pose=detected_object.pose, dimensions=detected_object.dimensions, outline=None, 
                    type=sign_dict[bbox_cls], entities=[])

    def object_to_signal(self, detected_object, bbox_cls):
        signal_state = SignalLightState(state=signal_dict[bbox_cls], duration=60) # actual duration?
        
        return Sign(pose=detected_object.pose, dimensions=detected_object.dimensions, outline=None, 
                    type=SignEnum.STOP_LIGHT, entities=[], state=SignState(signal_state))

    def detect_signs(self):
        class_ids = list(sign_dict.keys()) + list(signal_dict.keys())
        detected_objects, bbox_classes = super().detect_objects(class_ids)

        detected_signs = []
        for i in range(len(detected_objects)):
            cls = int(bbox_classes[i])
            
            if cls in list(sign_dict.keys()): # road sign
                sign = self.object_to_sign(detected_objects[i], cls)
                detected_signs.append(sign)
            else: # traffic signal
                signal = self.object_to_signal(detected_objects[i], cls)
                detected_signs.append(signal)
        
        return detected_signs


class OmniscientSignDetector(Component):
    """Obtains road sign & traffic signal detections from a simulator"""
    def __init__(self,vehicle_interface : GEMInterface):
        self.vehicle_interface = vehicle_interface
        self.signs = {}
        self.lock = threading.Lock()

    def rate(self):
        return 4.0
    
    def state_inputs(self):
        return []
    
    def state_outputs(self):
        return ['signs']

    def initialize(self):
        self.vehicle_interface.subscribe_sensor('sign_detector',self.sign_callback, Sign)
    
    def sign_callback(self, name : str, sign : Sign):
        with self.lock:
            self.signs[name] = sign

    def update(self) -> Dict[str,Sign]:
        with self.lock:
            return copy.deepcopy(self.signs)
