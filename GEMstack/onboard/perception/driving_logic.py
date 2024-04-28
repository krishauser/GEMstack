from typing import List, Tuple
from ..component import Component
from ...utils import serialization, settings
from ...state import AllState,VehicleState,Route,ObjectFrameEnum,Roadmap,Roadgraph
from ...mathutils import collisions
from ...mathutils.transforms import normalize_vector
from ...state.intent import VehicleIntent,VehicleIntentEnum
import os
import copy
from time import time
from dataclasses import replace
from queue import PriorityQueue
import numpy as np


class DrivingLogicIntent(Component):
    """
        Component that sets the vehicle intent. 

        TODO: Create intent based on mission. Move hardcoded logic to a config file. 
    """
    def __init__(self):
        self.t_start = None

    def state_inputs(self):
        return ['vehicle']

    def state_outputs(self) -> List[str]:
        return ['intent']

    def rate(self):
        return 1.0

    def update(self, vehicle_state):
        
        current_t = vehicle_state.pose.t
        
        if(self.t_start is None):
            self.t_start = current_t
        
        elapsed_t = current_t - self.t_start


        intent = None

        if(elapsed_t < 1.0):
            intent = VehicleIntent(intent=VehicleIntentEnum.DRIVING,entity='')
        elif(elapsed_t < 7.0):
            intent = VehicleIntent(intent=VehicleIntentEnum.PULL_OVER,entity='')
        else:
            intent = VehicleIntent(intent=VehicleIntentEnum.IDLE,entity='')

        return intent

