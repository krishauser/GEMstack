from dataclasses import dataclass, field
from ..utils.serialization import register
from .physical_object import ObjectFrameEnum, convert_point
from .trajectory import Path
from typing import List, Tuple, Optional

from enum import Enum


class PlannerEnum(Enum):
    RRT_STAR = 0  # position / yaw in m / radians relative to starting pose of vehicle
    HYBRID_A_STAR = (
        1  # position / yaw in m / radians relative to current pose of vehicle
    )
    PARKING = 2  # position in longitude / latitude, yaw=heading in radians with respect to true north (used in GNSS)


from enum import Enum


class PlannerEnum(Enum):
    RRT_STAR = 0                #position / yaw in m / radians relative to starting pose of vehicle
    HYBRID_A_STAR = 1           #position / yaw in m / radians relative to current pose of vehicle
    PARKING = 2                 #position in longitude / latitude, yaw=heading in radians with respect to true north (used in GNSS)
    LEAVE_PARKING = 3

    IDLE = 4                    # no mission, no driving
    SUMMON_DRIVING = 5          # route planning with lanes
    PARALLEL_PARKING = 6        # route planning for parallel parking

@dataclass
@register
class Route(Path):
    """A sequence of waypoints and lanes that the motion planner will attempt
    to follow.  Usually the path connects the centerlines of the given lanes.

    Unlike a Path, for the planner's convenience, the route should also extract
    out the wait lines (stop lines, crossings) from the roadgraph.
    """

    lanes: List[str] = field(default_factory=list)
    wait_lines: List[str] = field(default_factory=list)
