from dataclasses import dataclass
from ..utils.serialization import register
from .physical_object import PhysicalObject
from enum import Enum

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


@dataclass
@register
class Obstacle(PhysicalObject):
    material : ObstacleMaterialEnum
    collidable : bool

