from ...state import AllState,VehicleState,ObjectPose,ObjectFrameEnum,AgentState,AgentEnum,AgentActivityEnum
from ..interface.gem import GEMInterface
from ..component import Component
from typing import Dict
import threading
import copy

import numpy as np

class OmniscientAgentDetector(Component):
    """Obtains agent detections from a simulator"""

    def __init__(self, vehicle_interface: GEMInterface):
        self.vehicle_interface = vehicle_interface
        self.agents = {}
        self.lock = threading.Lock()

        self.start_pose = None

    def rate(self):
        return 4.0

    def state_inputs(self):
        return ['vehicle']

    def state_outputs(self):
        return ['agents']

    def initialize(self):
        self.vehicle_interface.subscribe_sensor('agent_detector', self.agent_callback, AgentState)

    def agent_callback(self, name: str, agent: AgentState):
        with self.lock:
            self.agents[name] = agent

    def update(self, vehicle : VehicleState) -> Dict[str, AgentState]:
        with self.lock:
            res = {}
            ped_num = 0

            if self.start_pose is None:
                self.start_pose = vehicle.pose
            # print("\nVehicle start state self.start_pose:", self.start_pose)
            # print("\nVehicle current state:", vehicle.pose, vehicle.v)

            for n, a in self.agents.items():
                # print("\nBefore to_frame: Agent:", n, a.pose, a.velocity)
                a = a.to_frame(ObjectFrameEnum.START, current_pose = a.pose, start_pose_abs = self.start_pose)
                # print("\nAfter to_frame START: Agent:", n, a.pose, a.velocity)
                # print('==============', a.pose.frame==ObjectFrameEnum.START)
                res[n] = a
                if a.type == AgentEnum.PEDESTRIAN:
                    ped_num += 1
            if ped_num > 0:
                print("\nDetected", ped_num, "pedestrians")
            return res
