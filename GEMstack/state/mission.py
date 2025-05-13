from dataclasses import dataclass
from typing import Optional
from . import ObjectPose
from ..utils.serialization import register
from enum import Enum
from .route import PlannerEnum


class MissionEnum(Enum):
    IDLE = 0            # not driving, no mission
    DRIVE = 1           # normal driving with routing
    DRIVE_ROUTE = 2     # normal driving with a fixed route
    TELEOP = 3          # manual teleop control
    RECOVERY_STOP = 4   # abnormal condition detected, must stop now
    ESTOP = 5           # estop pressed, must stop now

    SUMMONING_DRIVE = 6
    PARALLEL_PARKING = 7

@dataclass
@register
class MissionObjective:
    type : MissionEnum = MissionEnum.IDLE
    goal_pose: Optional[ObjectPose] = None
    