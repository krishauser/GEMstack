# something to store current id
# that current id will intentionally overflow to increment back down to 0

# GEM imports:
from ...state import AgentState

class LostAgent():
    def __init__(self, last_id: int, last_state: AgentState):
        self.time_since_seen: float = 0.0 # Time since the agent was last seen in seconds
        self.last_id: int = last_id
        self.last_state: AgentState = last_state

    def update_time(time: float):
        """Updates the time since the agent was last seen
        """
        if time <= 0:
            # TODO: log error here
            print("UPDATE TIME FOR LOST AGENT WAS LESS THAN OR EQUAL TO 0")
        else:
            self.time_since_seen += time