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

    # added by summoning team
    HEAD_IN_PARKING = 11            # parking, deciding to execute head in parking
    LEAVING_HEAD_IN_PARKING = 12    # leave parking, deciding to leave from head in parking
    BACK_IN_PARKING = 13            # parking, deciding to execute back in parking
    LEAVING_BACK_IN_PARKING = 14    # leave parking, deciding to leave from back in parking
    PARALLEL_PARKING = 15           # parking, deciding to execute parallel parking
    LEAVING_PARALLEL_PARKING = 16   # leave parking, deciding to leave from parallel parking


@dataclass
@register
class VehicleIntent:
    intent : VehicleIntentEnum = VehicleIntentEnum.IDLE
    entity : str = ''    # a scene entity referred to by the X_SIGN, LANE_CHANGE_X, MERGING, and XPARKING intents
