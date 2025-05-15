from typing import List

from GEMstack.utils import settings
from ..component import Component
from ...utils import serialization
from ...state import Route,ObjectFrameEnum, AllState, PlannerEnum, MissionPlan, ObjectPose
import os
import numpy as np
import time
import math

DEBUG = True

class PlanningComponentExample(Component):
    def __init__(self):
        self.start_time = time.time()
        self.goal_start_pose = None
        self.mode = settings.get("run.mode")
        pass

    def state_inputs(self):
        return ["all"]

    def rate(self):
        return 10.0

    def state_outputs(self)-> List[str]:
        return ["mission_plan"]
    
    def update(self, state: AllState):
        if DEBUG:
            print("PARKING COMPONENT: Entering RRT mode")
        if self.goal_start_pose == None:
            self.goal_start_pose = ObjectPose(
                frame=ObjectFrameEnum.START,
                t=state.start_vehicle_pose.t,
                x=state.vehicle.pose.x + 35,
                y=state.vehicle.pose.y,
                yaw=state.vehicle.pose.yaw + math.pi,
            )

        mission_plan = MissionPlan(PlannerEnum.RRT_STAR, self.goal_start_pose, state.start_vehicle_pose)

        if DEBUG:
            print("Planning component update with state:",mission_plan)

        return mission_plan
