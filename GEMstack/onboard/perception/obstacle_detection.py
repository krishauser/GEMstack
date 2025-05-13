
import threading
import copy
from typing import Dict
from ..component import Component

from ..interface.gem import GEMInterface
from ...state.obstacle import Obstacle

class OmniscientObstacleDetector(Component):
    """Obtains agent detections from a simulator"""
    def __init__(self,vehicle_interface : GEMInterface):
        self.vehicle_interface = vehicle_interface
        self.obstacles = {}
        self.lock = threading.Lock()

    def rate(self):
        return 15.0

    def state_inputs(self):
        return []

    def state_outputs(self):
        return ['obstacles']

    def initialize(self):
        self.vehicle_interface.subscribe_sensor('obstacle_detector',self.obstacle_callback, Obstacle)

    def obstacle_callback(self, name : str, obstacle : Obstacle):
        with self.lock:
            self.obstacles[name] = obstacle

    def update(self) -> Dict[str,Obstacle]:
        with self.lock:
            return copy.deepcopy(self.obstacles)


class GazeboObstacleDetector(OmniscientObstacleDetector):
    """Obtains agent detections from the Gazebo simulator using model_states topic"""
    def __init__(self, vehicle_interface : GEMInterface, tracked_obstacle_prefixes=None):
        super().__init__(vehicle_interface)

        # If specific model prefixes are provided, configure the interface to track them
        if tracked_obstacle_prefixes is not None:
            # Check if our interface has the tracked_model_prefixes attribute (is a GazeboInterface)
            if hasattr(vehicle_interface, 'tracked_obstacle_prefixes'):
                vehicle_interface.tracked_obstacle_prefixes = tracked_obstacle_prefixes
                print(f"Configured GazeboObstacleDetector to track obstacles with prefixes: {tracked_obstacle_prefixes}")
            else:
                print("Warning: vehicle_interface doesn't support tracked_obstacle_prefixes configuration")

    def initialize(self):
        # Use the same agent_detector sensor as OmniscientAgentDetector
        # The GazeboInterface implements this with model_states subscription
        super().initialize()
        print("GazeboObstacleDetector initialized and subscribed to model_states")