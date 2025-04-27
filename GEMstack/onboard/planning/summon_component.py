from typing import List
from ..component import Component
from ...utils import serialization
from ...state import Route,ObjectFrameEnum, AllState, PlannerEnum
import os
import numpy as np

class SummonSim(Component):
    """Simulates a summoning team component that receives an AllState object and outputs a route object."""
    def __init__(self):
        self.counter = 0

    def state_inputs(self):
        return ["all"]

    def state_outputs(self) -> List[str]:
        return ["route"]

    def rate(self):
        return 10.0

    def update(self, state: AllState):      
        print("SummonSim update with state:", state)
        base_path = os.path.dirname(__file__)
        file_path = os.path.join(base_path, "../../knowledge/routes/forward_15m.csv")
    
        waypoints = np.loadtxt(file_path, delimiter=',', dtype=float)
        if waypoints.shape[1] == 3:
                waypoints = waypoints[:,:2]
        print("waypoints", waypoints)
        route = Route(frame=ObjectFrameEnum.START,points=waypoints.tolist())
        print("route", route)
        return route
    
    
    
    