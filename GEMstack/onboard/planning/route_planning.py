from typing import List
from ..component import Component
from ...utils import serialization
from ...state import Route,ObjectFrameEnum, AllState, VehicleState, Roadgraph, MissionObjective
import os
import numpy as np
import yaml

class StaticRoutePlanner(Component):
    """Reads a route from disk and returns it as the desired route."""
    def __init__(self, routefn : str, frame : str = 'start'):
        self.routefn = routefn
        base, ext = os.path.splitext(routefn)
        if ext in ['.json','.yml','.yaml']:
            with open(routefn,'r') as f:
                self.route = serialization.load(f)
        elif ext == '.csv':
            waypoints = np.loadtxt(routefn,delimiter=',',dtype=float)
            if waypoints.shape[1] == 3:
                waypoints = waypoints[:,:2]
            if frame == 'start':
                self.route = Route(frame=ObjectFrameEnum.START,points=waypoints.tolist())
            elif frame == 'global':
                self.route = Route(frame=ObjectFrameEnum.GLOBAL,points=waypoints.tolist())
            elif frame == 'cartesian':
                self.route = Route(frame=ObjectFrameEnum.ABSOLUTE_CARTESIAN,points=waypoints.tolist())
            else:
                raise ValueError("Unknown route frame {} must be start, global, or cartesian".format(frame))
        else:
            raise ValueError("Unknown route file extension",ext)

    def state_inputs(self):
        return []

    def state_outputs(self) -> List[str]:
        return ['route']

    def rate(self):
        return 1.0

    def update(self):
        return self.route


def get_all_lane_points(roadgraph: Roadgraph) -> List:
    all_points = []
    for value in roadgraph.lanes.values():
        for pts in value.left.segments:
            for pt in pts:
                all_points.append(pt)
    return all_points


# implement route planning method
def generate_route(roadgraph, destination: List, vehicle: VehicleState, roadgraph_type='roadgraph'):
    if roadgraph_type == 'roadgraph':
        all_road_points = get_all_lane_points(roadgraph)
    elif roadgraph_type == 'point_list':
        all_road_points = roadgraph

    waypoints = []
    waypoints.append(destination)
    return waypoints


class SummoningRoutePlanner(Component):
    """Reads a route from disk and returns it as the desired route."""
    def __init__(self, roadgraphfn : str, frame : str = 'global'):
        self.frame = frame
        base, ext = os.path.splitext(roadgraphfn)
        if ext in ['.json', '.yml', '.yaml']:
            with open(roadgraphfn, 'r') as f:
                self.roadgraph = serialization.load(f)
                self.all_lane_points = get_all_lane_points(self.roadgraph)
        elif ext == '.csv':
            self.roadgraph = np.loadtxt(roadgraphfn,delimiter=',',dtype=float)
            self.roadgraph_type = 'point_list'

    def state_inputs(self):
        return ['vehicle', 'mission']

    def state_outputs(self) -> List[str]:
        return ['route']

    def rate(self):
        return 1.0

    def update(self, vehicle: VehicleState, mission: MissionObjective):
        self.vehicle = vehicle

        # TODO: get from the server
        self.destinations = [(23.0, 3.0, 0.0),(2.0, 6.0, 0.0)]

        waypoints = generate_route(self.roadgraph, self.destinations, self.vehicle,  self.roadgraph_type)

        if self.frame == 'start':
            self.route = Route(frame=ObjectFrameEnum.START, points=waypoints.tolist())
        elif self.frame == 'global':
            self.route = Route(frame=ObjectFrameEnum.GLOBAL, points=waypoints.tolist())
        elif self.frame == 'cartesian':
            self.route = Route(frame=ObjectFrameEnum.ABSOLUTE_CARTESIAN, points=waypoints.tolist())

        return self.route