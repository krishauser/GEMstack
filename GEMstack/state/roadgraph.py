from __future__ import annotations
from ..utils.serialization import register
from .physical_object import ObjectFrameEnum, convert_point
from .obstacle import Obstacle
from .sign import Sign
from enum import Enum
from collections import defaultdict
from dataclasses import dataclass, replace, field, asdict
import itertools
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
    CLIFF = 8
    OVERPASS_BOUNDARY = 9
    UNDERPASS_BOUNDARY = 10


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
    CURB_SIDE = 4           # region along the curb, i.e.., areas to pull over. 
    

class RoadgraphConnectionEnum(Enum):
    CONTINUES = 0           # after lane1 is complete it converts to lane2
    ADJACENT = 1            # lane2 is to the left of lane1 and are going the same way, allows a lane change
    ONCOMING = 2            # lane1 and lane2 are adjacent but going opposite ways
    CROSSING = 3            # lane1 is crossed by lane2, curve2, or region2
    MERGE = 4               # lane1 merges into lane2 
    DIVERGE = 5             # lane2 diverges from lane1
    BORDERING = 6           # lane1 is bordered by curve2
    RIGHT_TURN = 7          # lane1 passes through lane2 to perform right turn onto another lane (lane2 must have a CONTINUES relationship with another lane)
    LEFT_TURN = 8           # lane1 passes through lane2 to perform left turn onto another lane (lane2 must have a CONTINUES relationship with another lane)
    U_TURN = 9              # lane1 passes through lane2 to perform U-turn onto another lane (lane2 must have a CONTINUES relationship with another lane)
    

@dataclass
@register
class RoadgraphCurve:
    """Any curve in the roadgraph, whether a stopline, lane boundary, crossing,
    wall, etc.

    A curve can consist of multiple segments which may include gaps, e.g. a curb
    broken by driveways.  These proceed in back to forward order.
    """
    type : RoadgraphCurveEnum
    segments : List[List[Tuple[float,float,float]]]     #Polyline representation of the curve. List of lists of 3D positions.  3rd element is height above road surface, usually 0
    crossable : bool = True                             #Whether the curve is crossable by ego vehicle
    elevation : Optional[float] = None                  #for CURB, WALL, OBSTACLES, OVERPASS_BOUNDARY, UNDERPASS_BOUNDARY, elevation over road surface, in m
    height : Optional[float] = None                     #for CURB, WALL, OBSTACLES, OVERPASS_BOUNDARY, UNDERPASS_BOUNDARY, height in m

    def to_frame(self, orig_frame : ObjectFrameEnum, new_frame : ObjectFrameEnum,
                 current_origin = None, global_origin = None) -> RoadgraphCurve:
        return replace(self,segments=[[convert_point(p,orig_frame,new_frame,current_origin,global_origin) for p in seg] for seg in self.segments])

    def polyline(self) -> List[Tuple[float,float,float]]:
        """Returns a contiguous polyline representation of the curve."""
        return sum(self.segments,[])


@dataclass
@register
class RoadgraphLane:
    """A lane in the roadgraph.

    By convention, the left and right boundaries are oriented in back to
    forward order.  The end boundary is from right to left, and the start
    boundary is from left to right.
    """
    type : RoadgraphLaneEnum = RoadgraphLaneEnum.LANE               # type of lane
    surface : RoadgraphSurfaceEnum = RoadgraphSurfaceEnum.PAVEMENT  # surface of lane
    route_name : str = ''                                           # name of the route (e.g., street name) that this lane is on
    left : Optional[RoadgraphCurve] = None                          # left boundary of lane
    right : Optional[RoadgraphCurve] = None                         # right boundary of lane
    center : Optional[RoadgraphCurve] = None                        # centerline of lane
    begin : Optional[RoadgraphCurve] = None                         # Optional curve that begins at the start of the lane
    end : Optional[RoadgraphCurve] = None                           # Optional curve that ends at the end of the lane

    def to_frame(self, orig_frame : ObjectFrameEnum, new_frame : ObjectFrameEnum,
                 current_origin = None, global_origin = None) -> RoadgraphCurve:
        return replace(self,left=self.left.to_frame(orig_frame,new_frame,current_origin,global_origin) if self.left is not None else None,
                        right=self.right.to_frame(orig_frame,new_frame,current_origin,global_origin) if self.right is not None else None,
                        center=self.center.to_frame(orig_frame,new_frame,current_origin,global_origin) if self.center is not None else None,
                        begin=self.begin.to_frame(orig_frame,new_frame,current_origin,global_origin) if self.begin is not None else None,
                        end=self.end.to_frame(orig_frame,new_frame,current_origin,global_origin) if self.end is not None else None)

    def outline(self) -> List[Tuple[float,float,float]]:
        """Produces a 2D outline of the lane, including elevation.

        The first point is the beginning-right of the lane, and the points
        proceed CCW around the lane.

        If the begin and end are None, then straight lines are used to connect
        the left and right boundaries.  Otherwise, the begin and end curves
        are used to connect the boundaries.
        """
        if self.left is None or self.right is None:
            raise RuntimeError("Cannot produce outline of lane with missing left or right boundary")
        points = self.right.polyline()
        if self.end is not None:
            points += self.end.polyline()
        points += self.left.polyline()[::-1]
        if self.begin is not None:
            points += self.begin.polyline()[::-1]
        return [key for key, _group in itertools.groupby(points)]
        



@dataclass
@register
class RoadgraphRegion:
    type : RoadgraphRegionEnum
    outline : List[Tuple[float,float]]     # a list of 2D points defining the outline of the region in CCW order
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
        """Returns an empty roadgraph."""
        return Roadgraph(ObjectFrameEnum.GLOBAL)

    def is_valid(self) -> bool:
        """Returns true if the roadgraph is valid, i.e., all entities are in
        the roadgraph, all keys are unique, and all connections are valid."""
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
        """Returns a named entity in the roadgraph."""
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
        """Returns names of all entities in the roadgraph."""
        keys = set()
        keys.update(self.curves.keys())
        keys.update(self.lanes.keys())
        keys.update(self.regions.keys())
        keys.update(self.signs.keys())
        keys.update(self.static_obstacles.keys())
        return list(keys)
    
    def to_frame(self, frame : ObjectFrameEnum, current_pose = None, start_pose_abs = None) -> Roadgraph:
        newcurves = dict()
        for (k,c) in self.curves.items():
            newc = c.to_frame(self.frame,frame,current_pose,start_pose_abs)
            newcurves[k] = newc
        newlanes = dict()
        for (k,l) in self.lanes.items():
            newl = l.to_frame(self.frame,frame,current_pose,start_pose_abs)
            newlanes[k] = newl
        newregions = dict()
        for (k,r) in self.regions.items():
            newr = r.to_frame(self.frame,frame,current_pose,start_pose_abs)
            newregions[k] = newr
        newsigns = dict()
        for (k,s) in self.signs.items():
            news = s.to_frame(frame,current_pose,start_pose_abs)
            newsigns[k] = news
        newstatic_obstacles = dict()
        for (k,o) in self.static_obstacles.items():
            newo = o.to_frame(frame,current_pose,start_pose_abs)
            newstatic_obstacles[k] = newo
        newconnections = []
        for c in self.connections:
            newc = c.to_frame(self.frame,frame,current_pose,start_pose_abs)
            newconnections.append(newc)
        return replace(self, frame = frame, curves = newcurves, lanes = newlanes, regions = newregions, signs = newsigns, static_obstacles = newstatic_obstacles, connections=newconnections)



class RoadgraphNetwork(Roadgraph):
    """Stores all of the items within a Roadgraph but also allows for fast
    lookup of connections. This is used in routing.
    
    NOT TESTED YET.
    """
    def __init__(self, roadgraph = None):
        if roadgraph is not None:
            Roadgraph.__init__(self,**asdict(roadgraph))
        else:
            Roadgraph.__init__(self,ObjectFrameEnum.GLOBAL)
        
        self.connections_by_name = defaultdict(list)
        self.update_network()
    
    def update_network(self):
        self.connections_by_name = defaultdict(list)
        for c in self.connections:
            self.connections_by_name[c.lane1].append(c)
            if c.lane2 is not None:
                self.connections_by_name[c.lane2].append(c)
            if c.curve2 is not None:
                self.connections_by_name[c.curve2].append(c)
            if c.region2 is not None:
                self.connections_by_name[c.region2].append(c)
    
    def get_connections(self, name : str) -> List[RoadgraphConnection]:
        return self.connections_by_name[name]

    def continuations(self, name : str) -> List[str]:
        """Returns the names of all lanes that continue from the given lane."""
        if name not in self.lanes:
            return []
        conns = self.connections_by_name[name]
        return [c.lane2 for c in conns if c.type == RoadgraphConnectionEnum.CONTINUES and c.lane1 == name]

    def extend(self, name : str, maximum = None) -> List[str]:
        """Returns a sequence of lanes that extend from the current lane
        without taking any options, e.g., diverging, turning, etc."""
        if maximum is None:
            maximum = len(self.lanes)
        result = [name]
        for i in range(maximum):
            cont = self.continuations(result[-1])
            if len(cont) != 1:
                break
            result.append(cont[0])
            if cont[0] == name:  #loop
                break
        return result
    
    def left_adjacent(self, name : str) -> Optional[str]:
        """Returns the name of the lane to the left of the given lane, if it exists."""
        if name not in self.lanes:
            return None
        lane = self.lanes[name]
        if lane.left is None:
            # no left curve?
            return None
        conns = self.connections_by_name[name]
        for c in conns:
            if c.type == RoadgraphConnectionEnum.ADJACENT and c.lane1 == name:
                return c.lane2
        return None

    def right_adjacent(self, name : str) -> Optional[str]:
        """Returns the name of the lane to the right of the given lane, if it exists."""
        if name not in self.lanes:
            return None
        lane = self.lanes[name]
        if lane.right is None:
            # no left curve?
            return None
        conns = self.connections_by_name[name]
        for c in conns:
            if c.type == RoadgraphConnectionEnum.ADJACENT and c.lane2 == name:
                return c.lane1
        return None