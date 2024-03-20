from typing import List, Tuple
from ..component import Component
from ...utils import serialization
from ...state import AllState,VehicleState,Route,ObjectFrameEnum,Roadmap,Roadgraph
import os
import numpy as np

class StaticRoutePlanner(Component):
    """Reads a route from disk and returns it as the desired route."""
    def __init__(self, routefn : str, frame : str = 'start'):
        self.routefn = routefn
        base, ext = os.path.splitext(routefn)
        if ext in ['.json','.yml','.yaml']:
            with open(routefn,'r') as f:
                self.route = serialization.load(f)
        elif ext == '.csv':
            waypoints = np.loadtxt(routefn,delimiter=',',dtype=float)
            if waypoints.shape[1] == 3:
                waypoints = waypoints[:,:2]
            if frame == 'start':
                self.route = Route(frame=ObjectFrameEnum.START,points=waypoints.tolist())
            elif frame == 'global':
                self.route = Route(frame=ObjectFrameEnum.GLOBAL,points=waypoints.tolist())
            elif frame == 'cartesian':
                self.route = Route(frame=ObjectFrameEnum.ABSOLUTE_CARTESIAN,points=waypoints.tolist())
            else:
                raise ValueError("Unknown route frame {} must be start, global, or cartesian".format(frame))
        else:
            raise ValueError("Unknown route file extension",ext)

    def state_inputs(self):
        return []

    def state_outputs(self) -> List[str]:
        return ['route']

    def rate(self):
        return 1.0

    def update(self):
        return self.route
    
class NavigationRoutePlanner(Component):
    """Returns a straight line as the desired route."""
    def __init__(self, start : List[float], end : List[float]):
        # start and end are [x, y, yaw, speed, steer] in the START frame
        self.start = start
        self.end = end
        print("NavigationRoutePlanner: start",start)
        print("NavigationRoutePlanner: end",end)

    def state_inputs(self):
        # return ['vehicle', 'roadgraph']
        return ['all'] # Temporary for collision detection, should be replaced by the roadgraph

    def state_outputs(self) -> List[str]:
        return ['route']

    def rate(self):
        return 1.0

    # def update(self, vehicle : VehicleState, roadgraph : Tuple[Roadmap,Roadgraph]):
    #     # TODO: Figure out what is a Roadgraph
    #     roadmap, roadgraph = roadgraph

    #     # TODO: make this a real route
    #     route = Route(frame=ObjectFrameEnum.START,points=[self.start[:2],self.end[:2]])
    #     return route
    
    def update(self, state : AllState):
        # We can use agents to detect collisions, just like hw3
        # TODO: Shoule be replaced by the roadgraph to follow the GEMstack decision-making graph
        agents = state.agents

        # TODO: make this a real route

        
        route = Route(frame=ObjectFrameEnum.START,points=[self.start[:2],self.end[:2]])
        return route