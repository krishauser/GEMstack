from ...state import Roadgraph, Roadmap
from ..component import Component
from ...utils import serialization

class StaticRoadgraphUpdater(Component):
    def __init__(self, mapfn : str, vehicle_interface = None):
        self.mapfn = mapfn
        with open(mapfn,'r') as f:
            self.roadmap = serialization.load(f)
        if not isinstance(self.roadmap,(Roadmap,Roadgraph)):
            raise ValueError("Invalid roadmap file "+mapfn)
    
    def rate(self):
        return 10.0
    
    def state_inputs(self):
        return ['vehicle']
    
    def state_outputs(self):
        return ['roadgraph']
    
    def update(self, vehicle):
        return self.roadmap
    
