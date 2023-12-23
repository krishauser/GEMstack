from .predicate import PredicateBase
from ...state import AllState

class AgentCountPredicate(PredicateBase):
    """Counts the number of agents in the scene.
    
    Args:
        agent_type: If not None, only counts agents of this type.
    """
    def __init__(self,agent_type = None):
        self.agent_type = agent_type

    @classmethod
    def name(cls):
        return "AgentCount"
    
    def args(self):
        return [self.agent_type]

    def value_type(self):
        return int
    
    def value(self, state : AllState):
        if self.agent_type is None:
            return len(state.agents)
        else:
            return len([a for a in state.agents.values() if a.type == self.agent_type])
