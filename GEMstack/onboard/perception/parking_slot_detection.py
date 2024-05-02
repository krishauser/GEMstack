from ...state import AllState,VehicleState,ObjectPose,ObjectFrameEnum,AgentState,AgentEnum,AgentActivityEnum
from ..interface.gem import GEMInterface
from ..component import Component
from typing import Dict
import threading
import copy
import cv2
import numpy as np

# Template for a parking slot detector
class ParkingSlotDetector(Component):
    """Obtains agent detections from a simulator"""
    def __init__(self, vehicle_interface : GEMInterface):
        self.vehicle_interface = vehicle_interface
        self.front_image = None
        self.parking_slot = None

    def rate(self):
        return 4.0
    
    # def state_inputs(self):
    #     return ['...'] # or something from perception, if add anything, also change the computation graph
    
    def state_outputs(self):
        return ['parking_slot']

    def initialize(self):
        # ADD SENSOR SUBSCRIPTIONS HERE
        self.vehicle_interface.subscribe_sensor('front_camera',self.image_callback,cv2.Mat)
    
    # ADD OTHER CALLBACKS HERE
    def image_callback(self, image : cv2.Mat):
        self.front_image = image
    
    def parking_slot_detection(self):
        # DETECT PARKING SLOTS HERE

        # Some test code
        x, y = 15.0, 17.0
        yaw = 0.0
        # simulate noise
        x += np.random.normal(0, 0.1)
        y += np.random.normal(0, 0.1)
        yaw += np.random.normal(0, 0.1)
        slot = ObjectPose(t=0, x=x, y=y, yaw=yaw, frame=ObjectFrameEnum.START)

        # If the slot is from detection, probably want to use ObjectFrameEnum.CURRENT
        # slot = ObjectPose(t=0, x=x, y=y, yaw=yaw, frame=ObjectFrameEnum.CURRENT)

        return slot

    def update(self):
        self.parking_slot = self.parking_slot_detection()
        return copy.deepcopy(self.parking_slot)
