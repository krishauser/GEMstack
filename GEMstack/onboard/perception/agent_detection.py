from ...state import AllState,VehicleState,ObjectPose,ObjectFrameEnum,AgentState,AgentEnum,AgentActivityEnum
from ..interface.gem import GEMInterface
from ..component import Component
from typing import Dict
import threading
import copy

class OmniscientAgentDetector(Component):
    """Obtains agent detections from a simulator"""
    def __init__(self,vehicle_interface : GEMInterface):
        self.vehicle_interface = vehicle_interface
        self.agents = {}
        self.lock = threading.Lock()

    def rate(self):
        return 4.0
    
    def state_inputs(self):
        return []
    
    def state_outputs(self):
        return ['agents']

    def initialize(self):
        self.vehicle_interface.subscribe_sensor('agent_detector',self.agent_callback, AgentState)
    
    def agent_callback(self, name : str, agent : AgentState):
        with self.lock:
            self.agents[name] = agent

    def update(self) -> Dict[str,AgentState]:
        with self.lock:
            return copy.deepcopy(self.agents)
