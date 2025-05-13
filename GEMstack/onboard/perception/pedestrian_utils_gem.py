from ...state import ObjectPose, AgentState
from typing import Dict
import numpy as np

def match_existing_pedestrian(
        new_center: np.ndarray,
        new_dims: tuple,
        existing_agents: Dict[str, AgentState],
        distance_threshold: float = 1.0
) -> str:
    """
    Find the closest existing pedestrian agent within a specified distance threshold.
    """
    best_agent_id = None
    best_dist = float('inf')
    for agent_id, agent_state in existing_agents.items():
        old_center = np.array([agent_state.pose.x, agent_state.pose.y, agent_state.pose.z])
        dist = np.linalg.norm(new_center - old_center)
        if dist < distance_threshold and dist < best_dist:
            best_dist = dist
            best_agent_id = agent_id
    return best_agent_id


def compute_velocity(old_pose: ObjectPose, new_pose: ObjectPose, dt: float) -> tuple:
    """
    Compute the (vx, vy, vz) velocity based on change in pose over time.
    """
    if dt <= 0:
        return (0, 0, 0)
    vx = (new_pose.x - old_pose.x) / dt
    vy = (new_pose.y - old_pose.y) / dt
    vz = (new_pose.z - old_pose.z) / dt
    return (vx, vy, vz)