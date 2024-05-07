from ...state import AllState,VehicleState,ObjectPose,ObjectFrameEnum,AgentState,AgentEnum,AgentActivityEnum
from ..interface.gem import GEMInterface
from ..component import Component
from typing import Dict
import threading
import copy
import cv2
import numpy as np

# Template for a normal road detector
class LaneGoalDetector(Component):
    """Obtains agent detections from a simulator"""
    def __init__(self, vehicle_interface : GEMInterface):
        self.vehicle_interface = vehicle_interface
        self.front_image = None
        self.lane_goal = None

    def rate(self):
        return 4.0
    
    # def state_inputs(self):
    #     return ['...'] # or something from perception, if add anything, also change the computation graph
    
    def state_outputs(self):
        return ['lane_goal']

    def initialize(self):
        # ADD SENSOR SUBSCRIPTIONS HERE
        self.vehicle_interface.subscribe_sensor('front_camera',self.image_callback,cv2.Mat)
    
    # ADD OTHER CALLBACKS HERE
    def image_callback(self, image : cv2.Mat):
        self.front_image = image
    
    def lane_goal_detection(self):
        # DETECT PARKING SLOTS HERE

        # Some test code
        x, y = 30.0, 0.0
        # simulate noise
        x += np.random.normal(0, 0.1)
        y += np.random.normal(0, 0.1)
        slot = ObjectPose(t=0, x=x, y=y, frame=ObjectFrameEnum.START)

        # If the slot is from detection, probably want to use ObjectFrameEnum.CURRENT
        # slot = ObjectPose(t=0, x=x, y=y, frame=ObjectFrameEnum.CURRENT)

        return slot

    def update(self):
        self.lane_goal = self.lane_goal_detection()
        return copy.deepcopy(self.lane_goal)


class LaneBoundaryDetector(Component):
    """Obtains agent detections from a simulator"""
    def __init__(self, vehicle_interface : GEMInterface):
        self.vehicle_interface = vehicle_interface
        self.front_image = None
        self.lane_bound = None

    def rate(self):
        return 4.0
    
    # def state_inputs(self):
    #     return ['...'] # or something from perception, if add anything, also change the computation graph
    
    def state_outputs(self):
        return ['lane_bound']

    def initialize(self):
        # ADD SENSOR SUBSCRIPTIONS HERE
        self.vehicle_interface.subscribe_sensor('front_camera',self.image_callback,cv2.Mat)
    
    # ADD OTHER CALLBACKS HERE
    def image_callback(self, image : cv2.Mat):
        self.front_image = image
    
    def lane_bound_detection(self):
        # DETECT PARKING SLOTS HERE

        # Some test code
        LeftBound = -1.0
        RightBound = 1.0

        lane_bound = [LeftBound, RightBound]

        return lane_bound

    def update(self):
        self.lane_bound = self.lane_bound_detection()
        return copy.deepcopy(self.lane_bound)