from typing import List, Tuple
from ..component import Component
from ...utils import serialization
from ...state import Route, ObjectFrameEnum, AllState, VehicleState, Roadgraph, MissionObjective, MissionEnum, VehicleIntent, VehicleIntentEnum
import os
import numpy as np
import yaml
from . import RRT
import csv


SAVE_ROUTE = False


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
        for pts in value.right.segments:
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


def generate_route_free_run(current_pose, goal_position, roadgraph, roadgraph_type, map_margin=5.0, try_times=5)-> Tuple[bool, List[Tuple[float, float]]]:
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
    # goal_pose = find_available_pose(goal_position, points)
    goal_pose = [goal_position[0], goal_position[1], np.pi]

    # # Save lane_points
    # np.savetxt("lane_points.txt", points, delimiter=',',fmt='%f')

    searcher = RRT.BiRRT(start_pose, goal_pose, all_lane_points, map_boundaries)
    waypoints = []
    res = True
    for i in range(try_times):
        print(f"Try time: {i+1}")
        waypoints = searcher.search()
        if waypoints:
            break
    # print("waypoints:", waypoints)
    if not waypoints:
        waypoints = [start_pose]
        res = False
    
    print("=====================================================waypoint type:", type(waypoints[0]))
        
    return res, waypoints


class SummoningRoutePlanner(Component):
    """Reads a route from disk and returns it as the desired route."""
    def __init__(self, roadgraphfn : str, frame : str = 'start'):
        self.trytime = 0
        self.frame = frame
        base, ext = os.path.splitext(roadgraphfn)
        if ext in ['.json', '.yml', '.yaml']:
            with open(roadgraphfn, 'r') as f:
                self.roadgraph = serialization.load(f)
                self.roadgraph_type = 'roadgraph'
        elif ext == '.csv':
            self.roadgraph = np.loadtxt(roadgraphfn,delimiter=',',dtype=float)
            self.roadgraph_type = 'pointlist'
        else:
            raise ValueError("Unknown roadgraph file extension",ext)

    def state_inputs(self):
        return ['all']

    def state_outputs(self) -> List[str]:
        return ['route', 'mission']

    def rate(self):
        return 1.0

    def update(self, state: AllState):
        route = state.route
        mission = state.mission
        intent = state.intent
        current_pose = state.vehicle.pose

        self.trytime += 1
        print("Try time:", self.trytime)

        print('Input states:')
        print(mission)

        # TODO: get from the server
        target_location = request.g[] # position in longitude / latitude in GNSS
        target_location = [15, 11, 0.0]  # x, y, z

        # if current_pose.x == 0 and current_pose.y == 0:
        if mission.type == MissionEnum.PLAN and intent.intent == VehicleIntentEnum.IDLE:
            res, waypoints = generate_route_free_run(current_pose, target_location, self.roadgraph, self.roadgraph_type)
            waypoints = np.array(waypoints)

            if res:
                print("Route found.")
                mission.type = MissionEnum.UNPARK
            else:
                print('Can not find a route.')

            # route with heading
            if waypoints.shape[1] == 3:
                waypoints = waypoints[:, :2]

            if SAVE_ROUTE:
                np.savetxt("summoning_route_plan.txt", waypoints, delimiter=",", fmt="%f")
                with open("summoning_route_plan.csv", "w", newline='') as f:
                    writer = csv.writer(f)
                    for row in waypoints:
                        writer.writerow(["%f" % val for val in row])

            waypoints = waypoints.tolist()

            if self.frame == 'start':
                route = Route(frame=ObjectFrameEnum.START, points=waypoints)
            elif self.frame == 'global':
                route = Route(frame=ObjectFrameEnum.GLOBAL, points=waypoints)
            elif self.frame == 'cartesian':
                route = Route(frame=ObjectFrameEnum.ABSOLUTE_CARTESIAN, points=waypoints)
            else:
                raise ValueError("Unknown frame argument")


        print('Output states:')
        print(mission)

        return route, mission


