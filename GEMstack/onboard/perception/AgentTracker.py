# assigns pedestrians the same id 
# calculates velocity

class AgentTracker():
    """Associates and tracks AgentState agents.
    """
    def __init__(self):
        self.__lost_agents = []     # Stores LostAgent objects that are no longer detected
        self.prev_agents = []       # dict{id: agent} is more efficient, but list for
        self.current_agents = []    # simplicity to match update() output to start
        self.drop_agent_t: float = 1.0     # The maximum length of time a lost agent is stored before it is dropped in seconds
    
    def assign_ids(self, agents: list) -> Dict[str,AgentState]:
        """ 
        """
        agents = {}
    
    def __convert_to_start_frame(self):
        """Converts a list of AgentState agents from ouster Lidar frame of 
        reference (which is in reference to the current frame) to start 
        frame frame of reference
        """
        pass
        
    def __agents_overlap(ped1, ped2) -> bool:
        """Determines if two AgentState objects overlap.
        
        returns true if they overlap, false if not
        """
        pass

    def __calculate_velocity(self):
        """
        """
        pass
