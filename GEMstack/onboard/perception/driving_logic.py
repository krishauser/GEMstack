from typing import List, Tuple
from ...mathutils.transforms import *
from ..component import Component
from ...utils import serialization, settings
from ...state import AllState,VehicleState,Route,ObjectFrameEnum,Roadmap,Roadgraph,RoadgraphRegionEnum
from ...mathutils import collisions
from ...mathutils.transforms import normalize_vector
from ...state.intent import VehicleIntent,VehicleIntentEnum
import os
import copy
from time import time
from dataclasses import replace
from queue import PriorityQueue
import numpy as np


class DrivingLogicIntent(Component):
    """
        Component that sets the vehicle intent. 

        TODO: Create intent based on mission. Move hardcoded logic to a config file. 
    """
    def __init__(self):
        self.t_start = None

        self.pullover_point = None
        self.pullover_point_frame = None
        self.has_arrived = False

    def state_inputs(self):
        return ['all']

    def state_outputs(self) -> List[str]:
        return ['intent']

    def rate(self):
        return 0.5

    def update(self, all_state):
        
        current_t = all_state.vehicle.pose.t
        
        if(self.t_start is None):
            self.t_start = current_t
        
        elapsed_t = current_t - self.t_start


        intent = None

        if(elapsed_t < 3.0):
            intent = VehicleIntent(intent=VehicleIntentEnum.DRIVING,entity='')
            return intent
        
        if(not self.has_arrived and not self.have_arrived_at_pullover_spot(all_state)):
            if(self.pullover_point is None):
                self.pullover_point, self.pullover_point_frame = self.find_pullover_spot(all_state)

            intent = VehicleIntent(intent=VehicleIntentEnum.PULL_OVER,entity='',pullover_target=self.pullover_point, frame=self.pullover_point_frame)
            return intent
        
        if(self.have_arrived_at_pullover_spot(all_state)):
            self.has_arrived = True

        intent = VehicleIntent(intent=VehicleIntentEnum.IDLE,entity='')

        return intent
    
    def have_arrived_at_pullover_spot(self, state):
        """
            Check if the vehicle has arrived at the pullover spot. 
        """
        if(not state.intent.intent is VehicleIntentEnum.PULL_OVER):
            return False
        
        vehicle = state.vehicle.to_frame(state.intent.frame, state.vehicle.pose, state.start_vehicle_pose)
        pullover_point = state.intent.pullover_target
        
        dist = vector_dist([vehicle.pose.x, vehicle.pose.y], pullover_point)
        print(dist)
        if(dist < 1.5):
            return True
        
        return False

    def find_pullover_spot(self, state):
        """
            Find a spot to pull over to the right. 
        """
        
        # Get all curbsides
        all_curbsides = []
        for region in state.roadgraph.regions.values():
            if region.type is RoadgraphRegionEnum.CURB_SIDE:
                all_curbsides.append(region)

        # Use the first curb side
        # TODO: Add heuristic to find the closest curbside
        curb_side = all_curbsides[0]
        

        # Get long and short edge. We are parking parallel to the long edge. 
        long_edge = None
        short_edge = None
        if (vector_norm(vector_sub(curb_side.outline[0], curb_side.outline[1])) > vector_norm(vector_sub(curb_side.outline[2], curb_side.outline[1]))):
            long_edge = [curb_side.outline[1], curb_side.outline[0]]
            short_edge = [curb_side.outline[1], curb_side.outline[2]]
        else:
            long_edge = [curb_side.outline[1], curb_side.outline[2]]
            short_edge = [curb_side.outline[1], curb_side.outline[0]]
        
        # Get midline of the curbside
        midline_start = vector_mul(vector_add(short_edge[0], short_edge[1]), 0.5)
        midline_end = vector_add(vector_sub(long_edge[1], long_edge[0]), midline_start)

        # Search across midline for points that don't collide with objects
        SEARCH_STEPS = 100
        empty_points = []
        for i in range(0, SEARCH_STEPS):
            u = i / SEARCH_STEPS
            
            search_point = vector_add(
                vector_mul(midline_start, 1.0-u),
                vector_mul(midline_end, u)
            )

            #TODO: Convert to CollisionDetector2D object. 
            intersects = False
            for agent_s in state.agents.values():
                agent = agent_s.to_frame(state.roadgraph.frame, state.vehicle.pose, state.start_vehicle_pose)
                if(collisions.point_in_polygon_2d(search_point, agent.polygon_parent())):
                    intersects = True
                    break
            
            if(not intersects):
                empty_points.append(search_point)
                
        
        # Find point furthest from all vehicles
        points_with_distances = []
        for point in empty_points:
            min_dist = float('inf')
            for agent_s in state.agents.values():
                agent = agent_s.to_frame(state.roadgraph.frame, state.vehicle.pose, state.start_vehicle_pose)
                for agent_point in agent.polygon_parent():
                    dist = vector_dist(point, agent_point)
                    if(dist < min_dist):
                        min_dist = dist
            
            #for  in curb_side.outline:
            dist = vector_dist(point, midline_start)
            if(dist < min_dist):
                min_dist = dist

            dist = vector_dist(point, midline_end)
            if(dist < min_dist):
                min_dist = dist
            
            
            points_with_distances.append((point, min_dist))
        
        # Get point with max distance
        max_point = None
        max_dist = 0
        for point, dist in points_with_distances:
            if(dist > max_dist):
                max_dist = dist
                max_point = point

        return max_point, state.roadgraph.frame



