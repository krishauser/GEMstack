from dataclasses import dataclass
from ..utils.serialization import register
from .physical_object import PhysicalObject
from enum import Enum
from typing import Tuple

class AgentEnum(Enum):
    CAR = 0
    MEDIUM_TRUCK = 1
    LARGE_TRUCK = 2
    PEDESTRIAN = 3
    BICYCLIST = 4


class AgentActivityEnum(Enum):
    STOPPED = 0         # standing pedestrians, parked cars, etc. No need to predict motion.
    MOVING = 1          # standard motion.  Predictions will be used here
    FAST = 2            # indicates faster than usual motion, e.g., runners.
    UNDETERMINED = 3    # unknown activity


@dataclass
@register
class AgentState(PhysicalObject):
    type : AgentEnum
    activity : AgentActivityEnum
    velocity : Tuple[float,float,float]     #estimated velocity in x,y,z, m/s
    yaw_rate : float                        #estimated yaw rate, in radians/s
    