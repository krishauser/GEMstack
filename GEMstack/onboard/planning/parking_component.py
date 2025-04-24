from typing import List
from ..component import Component
from ...utils import serialization
from ...state import Route, ObjectFrameEnum, AllState, PlannerEnum, MissionPlan
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

        # Reading goal from state
        # print(f"\n AllState (parking goal): {state.goal} \n")
        # print(f"AllState (parking obstacles): {state.obstacles} \n")

        # After a goal is detected, change the mission plan to use PARKING.
        if state.goal:
            print("\n Parking goal available. Entering parking mode......")
            goal_x = state.goal.x
            goal_y = state.goal.y
            yaw = state.goal.yaw
            mission_plan = MissionPlan(goal_x, goal_y, yaw, PlannerEnum.PARKING)
        else:
            print("\n Entering RRT mode......")
            mission_plan = MissionPlan(1, 6, 0, PlannerEnum.RRT_STAR)

        print(f"\n ParkingSim update with state: {mission_plan} \n")
        return mission_plan