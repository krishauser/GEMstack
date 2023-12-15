from dataclasses import dataclass
from ..utils.serialization import register
from enum import Enum

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


@dataclass
@register
class VehicleIntent:
    intent : VehicleIntentEnum = VehicleIntentEnum.IDLE
    entity : str = ''    # a scene entity referred to by the X_SIGN, LANE_CHANGE_X, MERGING, and XPARKING intents
