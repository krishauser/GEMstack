from dataclasses import dataclass, field
from ..utils.serialization import register
from .vehicle import VehicleState
from .agent import AgentState
from .roadgraph import Roadgraph
from .environment import EnvironmentState
from .obstacle import Obstacle
from typing import List,Dict,Optional,Any

@dataclass
@register
class SceneState:
    t : float                                     #time, either simulation time or real time since epoch
    vehicle : VehicleState                        #ego-vehicle state
    roadgraph : Roadgraph                         #local roadgraph
    environment : EnvironmentState                #environmental conditions
    vehicle_lane : Optional[str]                  #lane in which the vehicle is located, if known
    agents : Dict[str,AgentState]                 #moving agents
    obstacles : Dict[str,Obstacle]                #dynamically determined obstacles
    
    def get_entity(self, name : str) -> Any:
        if name =='':
            return self.vehicle
        elif name in self.agents:
            return self.agents[name]
        elif name in self.obstacles:
            return self.obstacles[name]
        else:
            return self.roadgraph.get_entity(name)
    
    def entity_names(self) -> List[str]:
        keys = set([''] + self.roadgraph.entity_names())
        keys.update(self.agents.keys())
        keys.update(self.obstacles.keys())
        return list(keys)

    @staticmethod
    def zero():
        return SceneState(0.0,VehicleState.zero(),Roadgraph.zero(),EnvironmentState(),None,{},{})
