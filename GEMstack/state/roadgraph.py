from __future__ import annotations
from dataclasses import dataclass, replace, field
from ..utils.serialization import register
from .physical_object import ObjectFrameEnum, convert_point
from .obstacle import Obstacle
from .sign import Sign
from enum import Enum
from typing import List,Tuple,Any,Optional,Dict


class RoadgraphCurveEnum(Enum):
    LANE_BOUNDARY = 0
    STOP_LINE = 1
    YIELD_LINE = 2
    CROSSING_BOUNDARY = 3
    PARKING_SPOT_BOUNDARY = 4
    CURB = 5
    WALL = 6
    OBSTACLES = 7
    OVERPASS_BOUNDARY = 8
    UNDERPASS_BOUNDARY = 9


class RoadgraphLaneEnum(Enum):
    LANE = 0
    SHOULDER = 1
    ONRAMP = 2
    OFFRAMP = 3
    DRIVEWAY = 4
    PARKING_SPOT = 5
    PARKING_ZONE = 6
    OVERPASS = 7
    UNDERPASS = 8
    CROSSING = 9


class RoadgraphSurfaceEnum(Enum):
    PAVEMENT = 0
    DIRT = 1
    GRAVEL = 2
    GRASS = 3


class RoadgraphRegionEnum(Enum):
    VIRTUAL = 0             # just a region designation, e.g., geofence
    CLOSED_COURSE = 1       # open space, can drive anywhere
    PARKING_LOT = 2         # parking lot, should drive at low speed
    INTERSECTION = 3        # intersection, should not stop in middle
    

class RoadgraphConnectionEnum(Enum):
    CONTINUES = 0           # after lane1 is complete it converts to lane2
    ADJACENT = 1            # lane1 and lane2 are going the same way, allows a lane change
    ONCOMING = 2            # lane1 and lane2 are adjacent but going opposite ways
    CROSSING = 3            # lane2 or curve2 crosses over lane1
    MERGE = 4               # lane1 merges into lane2 
    DIVERGE = 5             # lane2 diverges from lane1
    BORDERING = 6           # lane1 is bordered by curve2
    RIGHT_TURN = 7          # lane1 passes through lane2 to perform right turn onto another lane (lane2 must have a CONTINUES relationship with another lane)
    LEFT_TURN = 8           # lane1 passes through lane2 to perform left turn onto another lane (lane2 must have a CONTINUES relationship with another lane)
    U_TURN = 9              # lane1 passes through lane2 to perform U-turn onto another lane (lane2 must have a CONTINUES relationship with another lane)
    

@dataclass
@register
class RoadgraphCurve:
    type : RoadgraphCurveEnum
    segments : List[List[Tuple[float,float,float]]]     #Polyline representation of the curve. List of lists of 3D positions.  Last element is height above road surface, usually 0
    crossable : bool = True
    elevation : Optional[float] = None                   #for CURB, WALL, OBSTACLES, OVERPASS_BOUNDARY, UNDERPASS_BOUNDARY, elevation over road surface, in m
    height : Optional[float] = None                     #for CURB, WALL, OBSTACLES, OVERPASS_BOUNDARY, UNDERPASS_BOUNDARY, height in m

    def to_frame(self, orig_frame : ObjectFrameEnum, new_frame : ObjectFrameEnum,
                 current_origin = None, global_origin = None) -> RoadgraphCurve:
        return replace(self,segments=[[convert_point(p,orig_frame,new_frame,current_origin,global_origin) for p in seg] for seg in self.segments])


@dataclass
@register
class RoadgraphLane:
    type : RoadgraphLaneEnum = RoadgraphLaneEnum.LANE
    surface : RoadgraphSurfaceEnum = RoadgraphSurfaceEnum.PAVEMENT
    left : Optional[RoadgraphCurve] = None
    right : Optional[RoadgraphCurve] = None
    center : Optional[RoadgraphCurve] = None
    begin : Optional[RoadgraphCurve] = None
    end : Optional[RoadgraphCurve] = None

    def to_frame(self, orig_frame : ObjectFrameEnum, new_frame : ObjectFrameEnum,
                 current_origin = None, global_origin = None) -> RoadgraphCurve:
        return replace(self,left=self.left.to_frame(orig_frame,new_frame,current_origin,global_origin) if self.left is not None else None,
                        right=self.right.to_frame(orig_frame,new_frame,current_origin,global_origin) if self.right is not None else None,
                        center=self.center.to_frame(orig_frame,new_frame,current_origin,global_origin) if self.center is not None else None,
                        begin=self.begin.to_frame(orig_frame,new_frame,current_origin,global_origin) if self.begin is not None else None,
                        end=self.end.to_frame(orig_frame,new_frame,current_origin,global_origin) if self.end is not None else None)


@dataclass
@register
class RoadgraphRegion:
    type : RoadgraphRegionEnum
    outline : List[Tuple[float,float]]
    crossable : bool = True
    
    def to_frame(self, orig_frame : ObjectFrameEnum, new_frame : ObjectFrameEnum,
                 current_origin = None, global_origin = None) -> RoadgraphCurve:
        return replace(self,outline=[convert_point(p,orig_frame,new_frame,current_origin,global_origin) for p in self.outline])


@dataclass
@register
class RoadgraphConnection:
    type : RoadgraphConnectionEnum
    lane1 : str                                 # the primary entity in this connection
    lane2 : Optional[str] = None                # for lane-lane connections, the second entity
    curve2 : Optional[str] = None               # for lane-curve connections (e.g., stopline crossing), the second entity
    region2 : Optional[str] = None              # for lane-region connections (e.g, intersection), the second entity
    location : List[Tuple[float,float]] = field(default_factory=list)         # a polyline defining when this transition occurs

    def to_frame(self, orig_frame : ObjectFrameEnum, new_frame : ObjectFrameEnum,
                 current_origin = None, global_origin = None) -> RoadgraphCurve:
        return replace(self,location=[convert_point(p,orig_frame,new_frame,current_origin,global_origin) for p in self.location])


@dataclass
@register
class Roadgraph:
    frame : ObjectFrameEnum                     # frame of reference for the roadgraph
    curves : Dict[str,RoadgraphCurve] = field(default_factory=dict)          # all non-lane-bordering curves in the roadgraph, e.g., stoplines, yieldlines, parking spot boundaries, curbs, walls, obstacles, overpass boundaries, underpass boundaries
    lanes : Dict[str,RoadgraphLane] = field(default_factory=dict)            # all lanes in the roadgraph
    regions : Dict[str,RoadgraphRegion] = field(default_factory=dict)        # all regions in the roadgraph
    signs : Dict[str,Sign] = field(default_factory=dict)                     # all signs in the roadgraph
    static_obstacles : Dict[str,Obstacle] = field(default_factory=dict)      # all static obstacles in the roadgraph
    connections : List[RoadgraphConnection] = field(default_factory=list)    # all connections between lanes, curves, and regions

    @staticmethod
    def zero():
        return Roadgraph(ObjectFrameEnum.GLOBAL)

    def is_valid(self) -> bool:
        keys = set()
        keys.update(self.curves.keys())
        for k in self.lanes.keys():
            if k in keys:
                return False
            keys.add(k)
        for k in self.regions.keys():
            if k in keys:
                return False
            keys.add(k)
        for k in self.signs.keys():
            if k in keys:
                return False
            if self.signs[k].frame != self.frame:
                return False
            keys.add(k)
        for k in self.static_obstacles.keys():
            if k in keys:
                return False
            if self.static_obstacles[k].frame != self.frame:
                return False
            keys.add(k)
        for c in self.connections:
            if c.lane1 not in self.lanes:
                return False
            if c.lane2 is not None and c.lane2 not in self.lanes:
                return False
            if c.curve2 is not None and c.curve2 not in self.curves:
                return False
            if c.region2 is not None and c.region2 not in self.regions:
                return False
        return True

    def get_entity(self, name : str) -> Any:
        if name in self.curves:
            return self.curves[name]
        if name in self.lanes:
            return self.lanes[name]
        if name in self.regions:
            return self.regions[name]
        if name in self.signs:
            return self.signs[name]
        if name in self.signs:
            return self.signs[name]
        raise KeyError("Entity {} not found".format(name))
    
    def entity_names(self) -> List[str]:
        keys = set()
        keys.update(self.curves.keys())
        keys.update(self.lanes.keys())
        keys.update(self.regions.keys())
        keys.update(self.signs.keys())
        keys.update(self.static_obstacles.keys())
        return list(keys)
    
    def to_frame(self, frame : ObjectFrameEnum, current_origin = None, global_origin = None) -> Roadgraph:
        newcurves = dict()
        for (k,c) in self.curves.items():
            newc = c.to_frame(self.frame,frame,current_origin,global_origin)
            newcurves[k] = newc
        newlanes = dict()
        for (k,l) in self.lanes.items():
            newl = l.to_frame(self.frame,frame,current_origin,global_origin)
            newlanes[k] = newl
        newregions = dict()
        for (k,r) in self.regions.items():
            newr = r.to_frame(self.frame,frame,current_origin,global_origin)
            newregions[k] = newr
        newsigns = dict()
        for (k,s) in self.signs.items():
            news = s.to_frame(frame,current_origin,global_origin)
            newsigns[k] = news
        newstatic_obstacles = dict()
        for (k,o) in self.static_obstacles.items():
            newo = o.to_frame(frame,current_origin,global_origin)
            newstatic_obstacles[k] = newo
        newconnections = []
        for c in self.connections:
            newc = c.to_frame(self.frame,frame,current_origin,global_origin)
            newconnections.append(newc)
        return replace(self, frame = frame, curves = newcurves, lanes = newlanes, regions = newregions, signs = newsigns, static_obstacles = newstatic_obstacles, connections=newconnections)