from dataclasses import dataclass
from ..utils.serialization import register
from .route import PlannerEnum
from typing import Optional
from GEMstack.state.physical_object import ObjectPose

@dataclass
@register
class MissionPlan:
    planner_type : PlannerEnum = PlannerEnum.RRT_STAR
    goal_vehicle_pose : Optional[ObjectPose] = None
    start_vehicle_pose : Optional[ObjectPose] = None
    # other mission-specific parameters can be added here

class ModeEnum:
    """Enum for different modes of operation."""
    HARDWARE = "hardware"
    SIMULATION = "simulation"
