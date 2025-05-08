from dataclasses import dataclass
from ..utils.serialization import register
from .route import PlannerEnum

@dataclass
@register
class MissionPlan:
    goal_x: float
    goal_y: float
    goal_orientation: float
    planner_type : PlannerEnum = PlannerEnum.SCANNING
    # other mission-specific parameters can be added here


    