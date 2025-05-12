from __future__ import annotations
from dataclasses import dataclass, replace
from ..utils.serialization import register
from .physical_object import ObjectFrameEnum,ObjectPose,PhysicalObject,convert_vector
from enum import Enum
from typing import Tuple

class ObstacleMaterialEnum(Enum):
    UNKNOWN = 0
    LEAVES = 1
    BRANCHES = 2
    LITTER = 3
    ROCKS = 4
    TRAFFIC_CONE = 5
    BARRIER = 6
    MAILBOX = 7
    SIGN = 8
    SMALL_ANIMAL = 9
    ROADKILL = 10

class ObstacleStateEnum(Enum):
    STOPPED = 0         # standing pedestrians, parked cars, etc. No need to predict motion.
    MOVING = 1          # standard motion.  Predictions will be used here
    FAST = 2            # indicates faster than usual motion
    UNDETERMINED = 3    # unknown activity
    STANDING = 4        # standing cone
    LEFT = 5            # flipped cone facing left
    RIGHT = 6           # flipped cone facing right



@dataclass
@register
class Obstacle(PhysicalObject):
    material : ObstacleMaterialEnum
    collidable : bool
    state: ObstacleStateEnum
