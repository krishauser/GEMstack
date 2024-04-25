from __future__ import annotations
from dataclasses import dataclass, replace
from ..utils.serialization import register
from .physical_object import ObjectFrameEnum,ObjectPose,PhysicalObject,convert_vector
from enum import Enum
from typing import Tuple
from enum import Flag, auto

class AgentEnum(Enum):
    CAR = 0
    MEDIUM_TRUCK = 1
    LARGE_TRUCK = 2
    PEDESTRIAN = 3
    BICYCLIST = 4


class AgentAttributesFlag(Flag):
    DEFAULT = 0
    WAVING = auto()

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
    velocity : Tuple[float,float,float]     #estimated velocity in x,y,z, m/s and in agent's local frame
    yaw_rate : float                        #estimated yaw rate, in radians/s
    attributes: AgentAttributesFlag

    def velocity_local(self) -> Tuple[float,float,float]:
        """Returns velocity in m/s in the agent's local frame."""
        return self.velocity
    
    def velocity_parent(self) -> Tuple[float,float,float]:
        """Returns velocity in m/s in the agent pose's parent frame.
        I.e., if the pose frame is CURRENT, then will return the velocity in
        the CURRENT frame."""
        return self.pose.rotation().dot(self.velocity).tolist()

    def to_frame(self, frame : ObjectFrameEnum, current_pose = None, start_pose_abs = None) -> AgentState:
        newpose = self.pose.to_frame(frame,current_pose,start_pose_abs)
        newvelocity = convert_vector(self.velocity,self.pose.frame,frame,current_pose,start_pose_abs)
        return replace(self,pose=newpose,velocity=newvelocity)