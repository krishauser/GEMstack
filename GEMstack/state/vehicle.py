from dataclasses import dataclass
from ..utils.serialization import register
from .physical_object import ObjectFrameEnum, ObjectPose
from enum import Enum


class VehicleGearEnum(Enum):
    PARK = -2
    REVERSE = -1
    NEUTRAL = 0
    FIRST = 1
    SECOND = 2
    THIRD = 3
    FOURTH = 4
    FIFTH = 5
    SIXTH = 6
    

@dataclass
@register
class VehicleState:
    """Represents the state of the ego-vehicle."""
    pose : ObjectPose                       #pose of the vehicle
    v : float                               #forward velocity in m/s
    acceleration : float                    #current acceleration / deceleration in m/s^2
    steering_wheel_angle : float            #angle of the steering wheel, in radians
    front_wheel_angle : float               #angle of the front wheels, in radians.   Related to steering_wheel_angle by a fixed transform
    heading_rate : float                    #the rate at which the vehicle is turning, in radians/s.  Related to v and front_wheel_angle by a fixed transform
    gear : VehicleGearEnum                  #the current gear
    left_turn_indicator : bool = False      #whether left turn indicator is on
    right_turn_indicator : bool = False     #whether right turn indicator is on
    horn_on : bool = False                  #whether horn is on
    wiper_level : int = 0                   #whether wipers are on

    @staticmethod
    def zero():
        return VehicleState(ObjectPose(ObjectFrameEnum.START,0,0,0),0,0,0,0,0,VehicleGearEnum.PARK,False,False,False,0)
    