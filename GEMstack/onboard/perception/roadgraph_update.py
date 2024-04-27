from ...state import Roadgraph, Roadmap, ObjectFrameEnum, RoadgraphCurveEnum, RoadgraphRegionEnum, RoadgraphRegion
from ..component import Component
from ...utils import serialization
from ...mathutils.transforms import *

from copy import deepcopy

class StaticRoadgraphUpdater(Component):
    
    def __init__(self, mapfn : str, execute_logic = False, vehicle_interface = None):
        self.mapfn = mapfn
        self.execute_logic = execute_logic
        self.cache = None
        with open(mapfn,'r') as f:
            self.roadmap = serialization.load(f)
        if not isinstance(self.roadmap,(Roadmap,Roadgraph)):
            raise ValueError("Invalid roadmap file "+mapfn)
    
    def rate(self):
        return 10.0
    
    def state_inputs(self):
        return ['vehicle']
    
    def state_outputs(self):
        return ['roadgraph']
    
    def update(self, vehicle):
        
        if(self.execute_logic):
            return self.process_logic(self.roadmap, vehicle)

        return self.roadmap
    
    def process_logic(self, roadgraph_source, vehicle):
        """
            Responsible for calculating roadgraph elements that aren't 
            directly observed, but can be inferred from other roadgraph 
            elements (e.g., parkable curbsides).
        """

        roadgraph = deepcopy(roadgraph_source)

        if(self.cache is None):
            self.cache = roadgraph

        # Some poses are not in absolute cartesian frame
        if(vehicle.pose.frame != ObjectFrameEnum.ABSOLUTE_CARTESIAN):
            return self.cache

        # Get all curbs
        all_curbs = []
        for curve_name in roadgraph.curves:
            curve = roadgraph.curves[curve_name]
            if curve.type == RoadgraphCurveEnum.CURB:
                all_curbs.append({"name": curve_name, "curve": curve})

        for curb_obj in all_curbs:
            curb = curb_obj["curve"]
            curb_name = curb_obj["name"]

            # Assume curb is composed of a single segment, with 2 points
            curb_begin = curb.segments[0][0]
            curb_end = curb.segments[0][-1]
            curb_midpoint = vector_mul(vector_add(curb_begin, curb_end), 0.5)
            
            # Calculate the normal vector of the curb facing the vehicle
            curb_normal = vector_cross(vector_sub(curb_begin, curb_midpoint), [0, 0, 1])
            if(vector_dot(curb_normal, [vehicle.pose.x, vehicle.pose.y, 0]) < 0):
                curb_normal = vector_mul(curb_normal, -1)
            curb_normal = vector_mul(curb_normal, 1.0 / vector_norm(curb_normal))

            # Create rectangular region for curbside parking
            curb_width = 4.0    # TODO: Use a heuristic to determine width
            roadgraph.regions[curb_name + "_curbside"] = RoadgraphRegion(
                type = RoadgraphRegionEnum.CURB_SIDE,
                outline = [
                    tuple(vector_add(curb_begin, vector_mul(curb_normal, curb_width))[:2]),
                    tuple(vector_add(curb_end, vector_mul(curb_normal, curb_width))[:2]),
                    tuple(curb_end[:2]),
                    tuple(curb_begin[:2])
                ]
            )
        
        
        self.cache = roadgraph
        
        return roadgraph
