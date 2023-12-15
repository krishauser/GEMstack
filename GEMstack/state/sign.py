from dataclasses import dataclass
from ..utils.serialization import register
from .physical_object import PhysicalObject
from enum import Enum
from typing import List,Optional

class SignEnum(Enum):
    #longitudinal signals
    STOP_SIGN = 0
    STOP_LIGHT = 1
    SPEED_LIMIT = 2
    SPEED_ADVISORY = 3
    YIELD = 4
    #lane control
    MERGE = 10
    LANE_ENDS = 11
    WRONG_WAY = 12
    DEAD_END = 13
    ONE_WAY = 14
    #crossings
    PEDESTRIAN_CROSSING = 20
    ANIMAL_CROSSING = 21
    RAILROAD_CROSSING = 22
    YIELD_TO_PEDESTRIANS = 23
    #turning permission
    NO_LEFT_TURN = 30
    NO_RIGHT_TURN = 31
    NO_LEFT_TURN_ON_RED = 32
    NO_RIGHT_TURN_ON_RED = 33
    NO_U_TURN = 34
    LEFT_TURN_ONLY = 35
    STRAIGHT_ONLY = 36
    RIGHT_TURN_ONLY = 37
    U_TURN_ONLY = 38
    EXIT_ONLY = 39
    #informational
    DIVIDED_ROADWAY = 50
    CROSS_TRAFFIC_DOES_NOT_STOP = 51
    NO_PASSING = 52
    NO_PARKING = 53
    #construction 
    BEGIN_CONSTRUCTION_ZONE = 100
    END_CONSTRUCTION_ZONE = 101
    EXIT_CLOSED = 102
    LANE_CLOSED = 103
    SHOULDER_CLOSED = 104


class SignalLightEnum(Enum):
    GREEN = 0
    YELLOW = 1
    RED = 2
    RED_FLASHING = 3
    YELLOW_FLASHING = 4


class CrossingGateEnum(Enum):
    OPEN = 0
    FLASHING = 1
    CLOSING = 2
    CLOSED = 3
    OPENING = 4


@dataclass
@register
class SignalLightState:
    state : SignalLightEnum
    duration : float


@dataclass
@register
class CrossingGateState:
    state : CrossingGateEnum
    duration : float


@dataclass
@register
class SignState:
    signal_state : Optional[SignalLightState] = None
    left_turn_signal_state : Optional[SignalLightState] = None
    right_turn_signal_state : Optional[SignalLightState] = None
    crossing_gate_state : Optional[CrossingGateState] = None


@dataclass
@register
class Sign(PhysicalObject):
    type : SignEnum
    entities : List[str]                        #the named entities to which this sign is attached, e.g., lane, intersection, onramp
    speed : Optional[int] = None                #for speed limit and advisory signs
    state : Optional[SignState]  = None         #for stop lights and optional for railroad crossings

