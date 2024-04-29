from __future__ import annotations
from dataclasses import dataclass
from ..utils.serialization import register
from enum import Enum
from .physical_object import ObjectFrameEnum, convert_point
from dataclasses import dataclass, replace, field, asdict

class VehicleIntentEnum(Enum):
    IDLE = 0            # stopped, parked, or waiting
    DRIVING = 1         # normal driving
    HALTING = 2         # terminating driving
    WAIT_AT_SIGN = 3    # normal driving, deciding to wait at sign, e.g., stop sign, yellow light, or yield
    PROCEED_THROUGH_SIGN = 4    # normal driving, deciding to proceed through sign, e.g., stop sign, yellow light, or yield
    LANE_CHANGE_LEFT = 5        # normal driving, deciding to execute a lane change to the left
    LANE_CHANGE_RIGHT = 6       # normal driving, deciding to execute a lane change to the right
    MERGING = 7         # normal driving, merging into lane
    PARKING = 8         # normal driving, executing parking behavior 
    LEAVING_PARKING = 9        # normal driving, leaving a parking spot
    U_TURN = 10         # normal driving, executing U-turn outside of dedicated lane

    PULL_OVER = 11      # normal driving, deciding to pull over to the curbside
    PULL_OVER_PICKUP = 12     # normal driving, deciding to pull over to the curbside for a pickup
    


@dataclass
@register
class VehicleIntent:
    intent : VehicleIntentEnum = VehicleIntentEnum.IDLE
    entity : str = ''    # a scene entity referred to by the X_SIGN, LANE_CHANGE_X, MERGING, and XPARKING intents

    frame : ObjectFrameEnum = None
    pullover_target : list = None # a target location for the intent, e.g., for pullover

    def to_frame(self, frame: ObjectFrameEnum, current_pose = None, start_pose_abs = None) -> VehicleIntent:
        return replace(self, pullover_target=convert_point(self.pullover_target, self.frame, frame, current_pose, start_pose_abs) if self.pullover_target is not None else None)
