from dataclasses import dataclass, field
from ..utils.serialization import register
from .physical_object import ObjectFrameEnum, convert_point
from .trajectory import Path
from typing import List,Tuple,Optional

@dataclass
@register
class Route(Path):
    """A sequence of waypoints and lanes that the motion planner will attempt
    to follow.  Usually the path connects the centerlines of the given lanes.
    
    Unlike a Path, for the planner's convenience, the route should also extract
    out the wait lines (stop lines, crossings) from the roadgraph.
    """
    lanes : List[str] = field(default_factory=list)
    wait_lines : List[str] = field(default_factory=list)
    yaws : Optional[List[float]] = None

