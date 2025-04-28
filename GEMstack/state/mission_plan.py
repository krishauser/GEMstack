from dataclasses import dataclass
from ..utils.serialization import register
from .route import PlannerEnum

@dataclass
@register
class MissionPlan:
    goal_x: float
    goal_y: float
    goal_orientation: float
    planner_type : PlannerEnum = PlannerEnum.RRT_STAR
    goal_vehicle_pose : Optional[ObjectPose] = None
    start_vehicle_pose : Optional[ObjectPose] = None
    # other mission-specific parameters can be added here


    