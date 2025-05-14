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


class GazeboAgentDetector(OmniscientAgentDetector):
    """Obtains agent detections from the Gazebo simulator using model_states topic"""
    def __init__(self, vehicle_interface : GEMInterface, tracked_model_prefixes=None):
        super().__init__(vehicle_interface)
        
        # If specific model prefixes are provided, configure the interface to track them
        if tracked_model_prefixes is not None:
            # Check if our interface has the tracked_model_prefixes attribute (is a GazeboInterface)
            if hasattr(vehicle_interface, 'tracked_model_prefixes'):
                vehicle_interface.tracked_model_prefixes = tracked_model_prefixes
                print(f"Configured GazeboAgentDetector to track models with prefixes: {tracked_model_prefixes}")
            else:
                print("Warning: vehicle_interface doesn't support tracked_model_prefixes configuration")
                
    def initialize(self):
        # Use the same agent_detector sensor as OmniscientAgentDetector
        # The GazeboInterface implements this with model_states subscription
        super().initialize()
        print("GazeboAgentDetector initialized and subscribed to model_states")
