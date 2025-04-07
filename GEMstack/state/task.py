from dataclasses import dataclass
from ..utils.serialization import register
from enum import Enum


class TaskEnum(Enum):
    IDLE = 0            # no task
    START = 1           # a task is given
    NAVIGATING = 2      # driving to the target location
    ARRIVED = 3         # arrive the target location


@dataclass
@register
class TaskPhase:
    phase : TaskEnum = TaskEnum.IDLE