import os
from typing import Dict, List

import numpy as np
# from GEMstack.onboard.component import Component
# from GEMstack.state.agent import AgentState
# from GEMstack.state.all import AllState,
# from GEMstack.state.physical_object import ObjectFrameEnum
# from GEMstack.state.route import PlannerEnum, Route
from .rrt_star import RRTStar
from ..interface.gem import GEMInterface
from ..component import Component
from ...state import AllState, Roadgraph, Route, PlannerEnum, ObjectFrameEnum, Path, \
    VehicleState, AgentState, AgentEnum, RoadgraphRegion, RoadgraphRegionEnum
from .RRT import BiRRT
from .reeds_shepp_parking import ReedsSheppParking
from ...utils import serialization
import math


def get_lane_points_from_roadgraph(roadgraph: Roadgraph) -> List:
    """
    Get all lane points from a roadgraph.
    Ouput: A list of [x, y, z], z = 0 for general cases.
    """
    lane_points = []
    for lane in roadgraph.lanes.values():
        for pts in lane.left.segments:
            for pt in pts:
                lane_points.append(pt)
        for pts in lane.right.segments:
            for pt in pts:
                lane_points.append(pt)
    return lane_points


def find_available_pose_in_lane(location, roadgraph, pose_yaw=None):
    # TODO: Check, not complete
    x, y = location
    min_dist = np.inf
    for lane in roadgraph.lanes.values():
        for pts in lane.right.segments:
            for idx, pt in enumerate(pts):
                dist = math.dist((x, y), pt[:2])
                if dist < min_dist:
                    right_x, right_y = pt[0], pt[1]
                    min_dist = dist
                    goal_lane = lane
                    if idx > 0:
                        dx = pts[idx][0] - pts[idx - 1][0]
                        dy = pts[idx][1] - pts[idx - 1][1]
                    else:
                        dx = pts[idx + 1][0] - pts[idx][0]
                        dy = pts[idx + 1][1] - pts[idx][1]
                    goal_yaw = math.atan2(dy, dx)

    min_dist = np.inf
    for pts in goal_lane.left.segments:
        for pt in pts:
            dist = math.dist((right_x, right_y), pt[:2])
            if dist < min_dist:
                left_x, left_y = pt[0], pt[1]

    goal_x = (left_x + right_x) / 2
    goal_y = (left_y + right_y) / 2

    if pose_yaw is not None:
        return [goal_x, goal_y, pose_yaw]

    return [goal_x, goal_y, goal_yaw]


class RoutePlanningComponent(Component):
    """Reads a route from disk and returns it as the desired route."""
    def __init__(self, roadgraphfn : str = None, map_frame : str = None):
        self.planner = None
        self.route = None

        """ Read offline map of lanes """
        if map_frame == 'global':
            self.map_frame = ObjectFrameEnum.GLOBAL
        elif map_frame == 'start':
            self.map_frame = ObjectFrameEnum.START
        else:
            raise ValueError("Frame argument not available. Should be 'start' or 'global'.")

        base, ext = os.path.splitext(roadgraphfn)
        if ext in ['.json', '.yml', '.yaml']:
            with open(roadgraphfn, 'r') as f:
                self.roadgraph = serialization.load(f)
            self.map_type = 'roadgraph'
        elif ext in ['.csv', '.txt']:
            roadgraph = np.loadtxt(roadgraphfn, delimiter=',', dtype=float)
            self.map_type = 'pointlist'
            self.roadgraph = Path(frame=self.map_frame, points=roadgraph.tolist())
        else:
            raise ValueError("Unknown roadgraph file extension", ext)

        # Used as route searchers' time limit as well as the update rate of the component
        self.update_rate = 0.25

    def state_inputs(self):
        return ["all"]

    def state_outputs(self) -> List[str]:
        return ['route']

    def rate(self):
        return self.update_rate

    def update(self, state: AllState):
        print("Route Planner's mission:", state.mission_plan.planner_type)
        # print("type of mission plan:", type(PlannerEnum.RRT_STAR))
        # print("Route Planner's mission:", state.mission_plan.planner_type.value == PlannerEnum.RRT_STAR.value)
        # print("Route Planner's mission:", state.mission_plan.planner_type.value == PlannerEnum.PARKING.value)
        # print("Mission plan:", state.mission_plan)
        print("Vehicle x:", state.vehicle.pose.x)
        print("Vehicle y:", state.vehicle.pose.y)
        print("Vehicle yaw:", state.vehicle.pose.yaw)

        """ Transform offline map to start frame """
        if self.roadgraph.frame is not ObjectFrameEnum.START:
            self.roadgraph = self.roadgraph.to_frame(ObjectFrameEnum.START, start_pose_abs=state.start_vehicle_pose)

        if state.mission_plan.planner_type.value == PlannerEnum.PARKING.value:
            print("I am in PARKING mode")
            # Return a route after doing some processing based on mission plan REMOVE ONCE OTHER PLANNERS ARE IMPLEMENTED
            base_path = os.path.dirname(__file__)
            file_path = os.path.join(base_path, "../../knowledge/routes/forward_15m_extra.csv")
        
            waypoints = np.loadtxt(file_path, delimiter=',', dtype=float)
            if waypoints.shape[1] == 3:
                    waypoints = waypoints[:,:2]
            print("waypoints", waypoints)
            self.route = Route(frame=ObjectFrameEnum.START,points=waypoints.tolist())

        elif state.mission_plan.planner_type.value == PlannerEnum.RRT_STAR.value:
            print("I am in RRT mode")
            start = (state.vehicle.pose.x+1, state.vehicle.pose.y+1)
            goal = (state.mission_plan.goal_x, state.mission_plan.goal_y) #When we implement kinodynamic, we need to include target_yaw also
            x_bounds = (0,20)
            y_bounds = (0,20)
            step_size = 1.0
            max_iter = 2000
            occupancy_grid = np.zeros((20, 20), dtype=int) 
            occupancy_grid[5:10, 5:10] = 1
            self.planner = RRTStar(start, goal, x_bounds, y_bounds, max_iter=max_iter, step_size=step_size, vehicle_width=1, occupancy_grid=occupancy_grid)
            rrt_resp = self.planner.plan()
            self.route = Route(frame=ObjectFrameEnum.START, points=rrt_resp)

        # Idle mode. No mission, no driving. (Added by Summoning)
        elif state.mission_plan.planner_type == PlannerEnum.IDLE:
            print("I am in IDLE mode")
            if state.vehicle.v < 0.01:
                self.route = None

        # Summoning driving mode. TODO: to be integrated with planning team's searcher
        elif state.mission_plan.planner_type == PlannerEnum.SUMMON_DRIVING:
            print("I am in SUMMON_DRIVING mode")
            # Get all the points of lanes
            if self.map_type == 'roadgraph':
                self.lane_points = get_lane_points_from_roadgraph(self.roadgraph)
            elif self.map_type == 'pointlist':
                self.lane_points = self.roadgraph.points

            # Find appropriate start and goal points that are on the lanes and fix for searching
            start_pose = find_available_pose_in_lane([state.vehicle.pose.x, state.vehicle.pose.y],
                                                    self.roadgraph, pose_yaw=state.vehicle.pose.yaw)
            goal_pose = find_available_pose_in_lane([state.mission_plan.goal_pose.x, state.mission_plan.goal_pose.y], self.roadgraph)

            print('Start pose:', start_pose)
            print('Goal pose:', goal_pose)

            # Search for waypoints
            searcher = BiRRT(start_pose, goal_pose, self.lane_points, update_rate=self.update_rate)
            waypoints = searcher.search()

            # For now, waypoints of [x, y, heading] is not working in longitudinal_planning. Use [x, y] instead.
            if waypoints:
                waypoints = np.array(waypoints)
                if waypoints.shape[1] == 3:
                    waypoints = waypoints[:, :2]
                waypoints = waypoints.tolist()

            if waypoints:
                self.route = Route(frame=ObjectFrameEnum.START, points=waypoints)
            else:
                self.route = state.route   # Fail to find a path, keep the origin route.

        # Parallel parking mode. TODO: To be integrate with parallel parking
        elif state.mission_plan.planner_type == PlannerEnum.PARALLEL_PARKING:
            print("I am in PARALLEL_PARKING mode")
            vehicle_pose = [state.vehicle.pose.x, state.vehicle.pose.y, state.vehicle.pose.yaw]

            detected_cones = []
            for name, agent in state.agents.items():
                if agent.type == AgentEnum.CONE:
                    detected_cones.append([agent.pose.x, agent.pose.y])

            if self.map_type == 'roadgraph':
                parking_slot = []
                for name, region in self.roadgraph.regions:
                    if region.type == RoadgraphRegionEnum.PARKING_LOT:
                        parking_slot.append(region.outline)
            else:
                parking_lots = None

            searcher = ReedsSheppParking(vehicle_pose=vehicle_pose, parking_lots=parking_lots,
                                         detected_cones=detected_cones, update_rate=self.update_rate)
            waypoints = searcher.find_collision_free_trajectory()

            if waypoints:
                self.route = Route(frame=ObjectFrameEnum.START, points=waypoints)
            else:
                self.route = state.route  # Fail to find a path, keep the origin route.

        else:
            print("Unknown mode")

        if self.route is not None:
            print("Route existed.")
        
        return self.route