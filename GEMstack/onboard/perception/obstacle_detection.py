from ...state import ObstacleState
from ..interface.gem import GEMInterface
from ..component import Component
from typing import Dict
import threading
import copy

class OmniscientObstacleDetector(Component):
    """Obtains obstacle detections from a simulator"""
    def __init__(self,vehicle_interface : GEMInterface):
        self.vehicle_interface = vehicle_interface
        self.obstacles = {}
        self.lock = threading.Lock()

    def rate(self):
        return 4.0
    
    def state_inputs(self):
        return []
    
    def state_outputs(self):
        return ['obstacles']

    def initialize(self):
        self.vehicle_interface.subscribe_sensor('obstacle_detector',self.obstacle_callback, ObstacleState)
    
    def obstacle_callback(self, name : str, obstacle : ObstacleState):
        with self.lock:
            self.obstacles[name] = obstacle

    def update(self) -> Dict[str,ObstacleState]:
        with self.lock:
            return copy.deepcopy(self.obstacles)