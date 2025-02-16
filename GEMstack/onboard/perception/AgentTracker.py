import math
from typing import Dict, List

from GEMstack.onboard.perception.IdTracker import IdTracker
from GEMstack.onboard.perception.PrevAgent import PrevAgent
from GEMstack.state.agent import AgentState


class AgentTracker():
    """Associates and tracks AgentState agents.
    """
    def __init__(self):
        # List of PrevAgent objects (each keeps track of the last seen state and time since seen)
        self.prev_agents: List[PrevAgent] = []
        # List of currently visible AgentState objects
        self.current_agents: List[AgentState] = []
        # Maximum time (in seconds) to keep a lost agent before dropping it.
        self.drop_agent_t: float = 1.0
        # Id tracker for creating new unique pedestrian IDs.
        self.id_tracker = IdTracker()
    
    def assign_ids(self, agents: list) -> Dict[str,AgentState]:
        # Act with the assumption that you are being sent a list of AgentState objects and you need to use the object fields to keep track of them for your task
        # Further act on the assumption that we will decide the id's of the pedestrians by assuming that 2 pedestrians are the same pedestrian if a
        # previously stored AgentState pose and dimensions overlap with a newly passed in AgentState
        # Act on the assumption that the AgentState objects are all in reference to the start frame of the vehicle
        # some helper functions in this class, LostAgent.py, and IdTracker.py have been created to try to help you out with your task.
        # Assume that the output returned from this function will be a dictionary of AgentState objects with the key corresponding to their id
        """
        Associates new AgentState objects with existing tracked agents based on overlap.
        If an agent does not match any previously tracked agent, a new unique id is assigned.
        Also updates the “lost” time for agents that are not matched in this frame.

        Parameters:
            agents (list): List of AgentState objects for the current frame 
                           (already converted to the start frame of reference).

        Returns:
            Dict[str, AgentState]: Dictionary mapping agent id (as a string) to AgentState.
        """
        dt = 1.0  # Assuming a fixed time-step of 1 second (for simplicity)
        output_agents: Dict[str, AgentState] = {}
        matched_ids = set()
        updated_prev_agents: List[PrevAgent] = []

        # Process each new agent from the current frame.
        for new_agent in agents:
            found_match = None
            # Look for a previously tracked agent whose bounding box overlaps.
            for prev in self.prev_agents:
                if prev.last_id in matched_ids:
                    continue  # Already matched with another new agent.
                if self.__agents_overlap(prev.last_state, new_agent):
                    found_match = prev
                    break

            if found_match is not None:
                # update velocity using the change in position.
                if hasattr(new_agent, 'velocity'):
                    new_agent.velocity = self.__calculate_velocity(found_match.last_state, new_agent, dt)

                # Update the matched agent’s state and reset its lost-time counter.
                found_match.last_state = new_agent
                found_match.time_since_seen = 0.0
                matched_ids.add(found_match.last_id)
                output_agents[str(found_match.last_id)] = new_agent
                updated_prev_agents.append(found_match)
            else:
                # No match found – assign a new unique id.
                new_id = self.id_tracker.get_new_ped_id()
                new_prev = PrevAgent(new_id, new_agent)
                # initialize velocity to 0.
                if hasattr(new_agent, 'velocity'):
                    new_agent.velocity = 0.0
                output_agents[str(new_id)] = new_agent
                updated_prev_agents.append(new_prev)

        # For all previously tracked agents that were not matched this frame,
        # update their time-since-seen and only keep them if they have not timed out.
        for prev in self.prev_agents:
            if prev.last_id not in matched_ids:
                prev.update_time(dt)
                if prev.time_since_seen < self.drop_agent_t:
                    updated_prev_agents.append(prev)

        # Save the updated list of tracked agents.
        self.prev_agents = updated_prev_agents
        # update current_agents to include only those seen in the current frame.
        self.current_agents = list(output_agents.values())
        return output_agents

    def __convert_to_start_frame(self):
        """Converts a list of AgentState agents from ouster Lidar frame of 
        reference (which is in reference to the current frame) to start 
        frame frame of reference
        """
        # you can ignore this function akul
        pass
        
    def __agents_overlap(self, ped1, ped2) -> bool:
        """
        Determines if two AgentState objects overlap based on their pose and dimensions.

        Assumes each AgentState has:
            - pose with attributes x and y.
            - dimensions with attributes width and height.

        Returns:
            bool: True if they overlap; False otherwise.
        """
        # Get first agent's properties.
        x1, y1 = ped1.pose.x, ped1.pose.y
        w1, h1 = ped1.dimensions.width, ped1.dimensions.height

        # Get second agent's properties.
        x2, y2 = ped2.pose.x, ped2.pose.y
        w2, h2 = ped2.dimensions.width, ped2.dimensions.height

        # Compute bounding boxes (assuming (x, y) is the center).
        left1, right1 = x1 - w1 / 2, x1 + w1 / 2
        top1, bottom1 = y1 - h1 / 2, y1 + h1 / 2

        left2, right2 = x2 - w2 / 2, x2 + w2 / 2
        top2, bottom2 = y2 - h2 / 2, y2 + h2 / 2

        # Check for overlap between the bounding boxes.
        overlap = not (right1 < left2 or left1 > right2 or bottom1 < top2 or top1 > bottom2)
        return overlap

    def __calculate_velocity(self, old_state: AgentState, new_state: AgentState, dt: float) -> float:
        """
        Calculates the velocity based on the change in pose over time.

        Parameters:
            old_state (AgentState): The previous state.
            new_state (AgentState): The current state.
            dt (float): Time difference between the two states.

        Returns:
            float: The computed velocity.
        """
        dx = new_state.pose.x - old_state.pose.x
        dy = new_state.pose.y - old_state.pose.y
        return math.sqrt(dx * dx + dy * dy) / dt if dt > 0 else 0.0
