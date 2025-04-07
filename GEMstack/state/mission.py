from dataclasses import dataclass
from ..utils.serialization import register
from enum import Enum

class MissionEnum(Enum):
    IDLE = 0            # not driving, no mission
    DRIVE = 1           # normal driving with routing
    DRIVE_ROUTE = 2     # normal driving with a fixed route
    TELEOP = 3          # manual teleop control
    RECOVERY_STOP = 4   # abnormal condition detected, must stop now
    ESTOP = 5           # estop pressed, must stop now

    # added by summoning team
    PLAN = 6  # planning for the route
    PARK = 7  # parking
    UNPARK = 8  # leave parking
    WAIT = 9  # waiting at the target location
    INTERSECT = 10  # (optional) performing some activity at the objection location


@dataclass
@register
class MissionObjective:
    type : MissionEnum = MissionEnum.IDLE
    