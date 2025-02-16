from typing import Dict


class AgentTracker():
    """Associates and tracks AgentState agents.
    """
    def __init__(self):
        self.prev_agents = []       # dict{id: agent} is more efficient, but list for simplicity to match update() output to start
        self.current_agents = []    # Stores PrevAgent objects that were in previous frames
        self.drop_agent_t: float = 1.0     # The maximum length of time a lost agent is stored before it is dropped in seconds
    
    def assign_ids(self, agents: list) -> Dict[str,AgentState]:
        """ 
        """
        agents = {}
        # Act with the assumption that you are being sent a list of AgentState objects and you need to use the object fields to keep track of them for your task
        # Further act on the assumption that we will decide the id's of the pedestrians by assuming that 2 pedestrians are the same pedestrian if a
        # previously stored AgentState pose and dimensions overlap with a newly passed in AgentState

        # Act on the assumption that the AgentState objects are all in reference to the start frame of the vehicle

        # some helper functions in this class, LostAgent.py, and IdTracker.py have been created to try to help you out with your task.

        # Assume that the output returned from this function will be a dictionary of AgentState objects with the key corresponding to their id
        pass

    def __convert_to_start_frame(self):
        """Converts a list of AgentState agents from ouster Lidar frame of 
        reference (which is in reference to the current frame) to start 
        frame frame of reference
        ""
        # you can ignore this function akul
        pass
        
    def __agents_overlap(self, ped1, ped2) -> bool:
        """Determines if two AgentState objects overlap.
        
        returns true if they overlap, false if not
        """
        pass

    def __calculate_velocity(self):
        """
        """
        pass
