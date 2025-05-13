from typing import List
from ..component import Component
from ...utils import serialization
from ...state import Route, ObjectFrameEnum, AllState, PlannerEnum, MissionObjective
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

        # After a goal is detected, change the mission plan to use PARKING.
        if state.parking_goal:
            print("\n Parking goal available. Entering PARKING mode......")
            print("Trying to Route to ", state.parking_goal)
            mission_plan = MissionObjective(PlannerEnum.PARKING, state.parking_goal)
        else:
            print("\n Entering SCANNING mode......")
            mission_plan = MissionObjective(PlannerEnum.SCANNING)

        print(f"\n ParkingSim update with state: {mission_plan} \n")
        return mission_plan