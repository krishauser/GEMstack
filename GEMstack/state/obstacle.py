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


@dataclass
@register
class ObstacleState(PhysicalObject):
    type: ObstacleMaterialEnum
    activity: ObstacleStateEnum
    velocity: Tuple[float, float, float]  # estimated velocity in x,y,z, m/s and in agent's local frame
    yaw_rate: float  # estimated yaw rate, in radians/s

    def velocity_local(self) -> Tuple[float, float, float]:
        """Returns velocity in m/s in the agent's local frame."""
        return self.velocity

    def velocity_parent(self) -> Tuple[float, float, float]:
        """Returns velocity in m/s in the agent pose's parent frame.
        I.e., if the pose frame is CURRENT, then will return the velocity in
        the CURRENT frame."""
        return self.pose.rotation().dot(self.velocity).tolist()

    def to_frame(self, frame: ObjectFrameEnum, current_pose=None, start_pose_abs=None) -> ObstacleState:
        newpose = self.pose.to_frame(frame, current_pose, start_pose_abs)
        newvelocity = convert_vector(self.velocity, self.pose.frame, frame, current_pose, start_pose_abs)
        return replace(self, pose=newpose, velocity=newvelocity)