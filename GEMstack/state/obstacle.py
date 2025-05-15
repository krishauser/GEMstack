from __future__ import annotations
from dataclasses import dataclass, replace
from ..utils.serialization import register
from .physical_object import ObjectFrameEnum,PhysicalObject #,convert_vector

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
    UNDETERMINED = 0    # unknown activity
    STANDING = 1        # standing cone
    LEFT = 2            # flipped cone facing left
    RIGHT = 3           # flipped cone facing right

@dataclass
@register
class Obstacle(PhysicalObject):
    material : ObstacleMaterialEnum
    collidable : bool
    state: ObstacleStateEnum = ObstacleStateEnum.UNDETERMINED

    def to_frame(self, frame: ObjectFrameEnum, current_pose=None, start_pose_abs=None) -> Obstacle:
        newpose = self.pose.to_frame(frame, current_pose, start_pose_abs)
        return replace(self, pose=newpose)

