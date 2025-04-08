from dataclasses import dataclass, field, field
from ..utils.serialization import register
from enum import Enum
from typing import List

class MissionEnum(Enum):
    IDLE = 0            # not driving, no mission
    DRIVE = 1           # normal driving with routing
    DRIVE_ROUTE = 2     # normal driving with a fixed route
    TELEOP = 3          # manual teleop control
    RECOVERY_STOP = 4   # abnormal condition detected, must stop now
    ESTOP = 5           # estop pressed, must stop now

@dataclass
@register
class MissionObjective:
    type : MissionEnum = MissionEnum.IDLE
    state_list :list = field(default_factory=lambda: [MissionEnum.IDLE, MissionEnum.DRIVE, MissionEnum.RECOVERY_STOP])
    index: int = 0

    @staticmethod
    def zero():
        return MissionObjective(MissionEnum.IDLE, [MissionEnum.IDLE, MissionEnum.DRIVE, MissionEnum.RECOVERY_STOP], 0)
    