from dataclasses import dataclass
from ..utils.serialization import register
from .route import PlannerEnum
from typing import Optional
from .physical_object import ObjectPose

@dataclass
@register
class MissionPlan:
    goal_x: Optional[float] = None
    goal_y: Optional[float] = None
    goal_orientation: Optional[float] = None
    planner_type : PlannerEnum = PlannerEnum.IDLE
    # other mission-specific parameters can be added here
    goal_pose: Optional[ObjectPose] = None
    start_pose: Optional[ObjectPose] = None


    