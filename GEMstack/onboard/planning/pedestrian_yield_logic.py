from ...state import AgentState,AgentEnum,EntityRelation,EntityRelationEnum
from ..component import Component
from typing import List,Dict

class PedestrianYielder(Component):
    """Yields for all pedestrians in the scene.
    
    Result is stored in the relations graph.
    """
    def rate(self):
        return None
    def state_inputs(self):
        return ['agents']
    def state_outputs(self):
        return ['relations']
    def update(self,agents : Dict[str,AgentState]) -> List[EntityRelation]:
        res = []
        for n,a in agents.items():
            if a.type == AgentEnum.PEDESTRIAN:
                #relation: ego-vehicle yields to pedestrian
                res.append(EntityRelation(type=EntityRelationEnum.YIELDING,obj1='',obj2=n))
        return res
