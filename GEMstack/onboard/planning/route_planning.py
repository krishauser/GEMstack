from typing import List
from ..component import Component
from ...utils import serialization
from ...state import Route,ObjectFrameEnum, AllState, VehicleState, Roadgraph, MissionObjective
import os
import numpy as np
import yaml
from ...onboard.planning import RRT

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


def find_available_pose(position, lane_points):
    target_xy = np.array(position[:2])
    lane_np = np.array([[x, y] for x, y, _ in lane_points])

    dists = np.linalg.norm(lane_np - target_xy, axis=1)
    idx = np.argmin(dists)
    x, y = lane_np[idx]

    if idx < len(lane_points) - 1:
        next_pt = lane_points[idx + 1]
    elif idx > 0:
        next_pt = lane_points[idx - 1]
    else:
        return x, y, 0.0

    dx = next_pt[0] - x
    dy = next_pt[1] - y
    yaw = np.arctan2(dy, dx)

    return [x, y, yaw]


def generate_route_free_run(current_pose, goal_position, roadgraph, roadgraph_type, map_margin=5.0, try_times=5):
    """
    Assume the vehicle can run in both directions in the lanes
    """
    if roadgraph_type == 'roadgraph':
        all_lane_points = get_all_lane_points(roadgraph)
    else:  # roadgraph_type == 'point_list'
        all_lane_points = roadgraph
    points = np.array(all_lane_points)

    # Decide the route searching boundaries
    map_boundaries = [np.min(points[:, 0]) - map_margin, np.max(points[:, 0]) + map_margin,
                      np.min(points[:, 1]) - map_margin, np.max(points[:, 1]) + map_margin]

    start_pose = [current_pose.x, current_pose.y, current_pose.yaw]
    # TODO: find yaw with free lane direction
    goal_pose = find_available_pose(goal_position, points)

    searcher = RRT.BiRRT(start_pose, goal_pose, all_lane_points, map_boundaries)
    waypoints = []
    for _ in range(try_times):
        waypoints = searcher.search()
        if waypoints:
            break
    print("waypoints:", waypoints)
    if not waypoints:
        raise RuntimeError('No waypoints found')
    return waypoints


class SummoningRoutePlanner(Component):
    """Reads a route from disk and returns it as the desired route."""
    def __init__(self, roadgraphfn : str, frame : str = 'start'):
        self.frame = frame
        base, ext = os.path.splitext(roadgraphfn)
        if ext in ['.json', '.yml', '.yaml']:
            with open(roadgraphfn, 'r') as f:
                self.roadgraph = serialization.load(f)
                self.roadgraph_type = 'roadgraph'
        elif ext == '.csv':
            self.roadgraph = np.loadtxt(roadgraphfn,delimiter=',',dtype=float)
            self.roadgraph_type = 'point_list'
        else:
            raise ValueError("Unknown roadgraph file extension",ext)

    def state_inputs(self):
        return ['vehicle']

    def state_outputs(self) -> List[str]:
        return ['route']

    def rate(self):
        return 10.0

    def update(self, vehicle: VehicleState):
        self.current_pose = vehicle.pose

        # TODO: get from the server
        self.goal_positions = [(23.0, 3.0, 0.0),(2.0, 6.0, 0.0)]    # x, y, z

        waypoints = generate_route_free_run(self.current_pose, self.goal_positions[0], self.roadgraph, self.roadgraph_type)
        waypoints = np.array(waypoints)
        if waypoints.shape[1] == 3:
            waypoints = waypoints[:, :2]

        if self.frame == 'start':
            self.route = Route(frame=ObjectFrameEnum.START, points=waypoints.tolist())
        elif self.frame == 'global':
            self.route = Route(frame=ObjectFrameEnum.GLOBAL, points=waypoints.tolist())
        elif self.frame == 'cartesian':
            self.route = Route(frame=ObjectFrameEnum.ABSOLUTE_CARTESIAN, points=waypoints.tolist())

        return self.route