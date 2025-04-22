from typing import List
from ..component import Component
from ...utils import serialization
from ...state import Route,ObjectFrameEnum, AllState, PlannerEnum, MissionPlan
import os
import numpy as np
import time

class ParkingSim(Component):
    def __init__(self):
        self.start_time = time.time()
        pass

    def state_inputs(self):
        return ["all"]

    def rate(self):
        return 10.0

    def state_outputs(self)-> List[str]:
        return ["mission_plan"]
    
    def update(self, state: AllState):
        # Calculate elapsed time since initialization.
        elapsed_time = time.time() - self.start_time
        
        # After 4 seconds, change the mission plan to use PARKING.
        if elapsed_time >= 4.0:
            print("Entering parking mode")
            mission_plan = MissionPlan(1, 6, 0, PlannerEnum.PARKING)
        else:
            print("Entering RRT mode")
            mission_plan = MissionPlan(1, 6, 0, PlannerEnum.RRT_STAR)

        print("ParkingSim update with state:",mission_plan)
        return mission_plan