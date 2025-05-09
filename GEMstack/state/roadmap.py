from dataclasses import dataclass
from ..utils.serialization import register
from .roadgraph import Roadgraph

@dataclass
@register
class Roadmap:
    name : str
    author : str
    authored_date : str
    graph : Roadgraph
    