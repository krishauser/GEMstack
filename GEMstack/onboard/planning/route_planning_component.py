import os
from typing import Dict, List

import numpy as np
# from GEMstack.onboard.component import Component
# from GEMstack.state.agent import AgentState
# from GEMstack.state.all import AllState,
# from GEMstack.state.physical_object import ObjectFrameEnum
# from GEMstack.state.route import PlannerEnum, Route
from .rrt_star import RRTStar

from ..component import Component
from ...state import AllState, Roadgraph, Route, PlannerEnum, ObjectFrameEnum, Path, VehicleState, AgentState, AgentEnum
from .RRT import BiRRT
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


def plan_available_pose_on_lane(location, roadgraph, current_yaw=None):
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

    if current_yaw is not None:
        return [goal_x, goal_y, current_yaw]

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

        # TODO: Transform global map to start frame
        if self.map_frame == ObjectFrameEnum.GLOBAL:
            self.start_pose_global = None      # Read from GNSS
            self.roadgraph = self.roadgraph.to_frame(ObjectFrameEnum.START, start_pose_abs=self.start_pose_global)

        # If parking route existed, not search anymore. For simulation testing only. TODO: Delete after integration
        self.parking_route_existed = False

    def state_inputs(self):
        return ["all"]

    def state_outputs(self) -> List[str]:
        return ['route']

    def rate(self):
        return 10.0

    def update(self, state: AllState):
        print("Route Planner's mission:", state.mission_plan.planner_type)
        # print("type of mission plan:", type(PlannerEnum.RRT_STAR))
        # print("Route Planner's mission:", state.mission_plan.planner_type.value == PlannerEnum.RRT_STAR.value)
        # print("Route Planner's mission:", state.mission_plan.planner_type.value == PlannerEnum.PARKING.value)
        # print("Mission plan:", state.mission_plan)
        # print("Vehicle x:", state.vehicle.pose.x)
        # print("Vehicle y:", state.vehicle.pose.y)
        # print("Vehicle yaw:", state.vehicle.pose.yaw)

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

        # Summoning driving mode. (Added by Summoning) TODO: to be integrated with planning team's searcher with local obstacle avoidance
        elif state.mission_plan.planner_type == PlannerEnum.SUMMON_DRIVING:
            print("I am in SUMMON_DRIVING mode")
            # Get all the points of lanes and transform to START frame
            if self.map_type == 'roadgraph':
                self.lane_points = get_lane_points_from_roadgraph(self.roadgraph)
            elif self.map_type == 'pointlist':
                self.lane_points = self.roadgraph.points

            # Transform the goal to START frame
            if state.mission_plan.goal_pose.frame == ObjectFrameEnum.START:
                goal_pose_ = state.mission_plan.goal_pose
            elif state.mission_plan.goal_pose.frame == ObjectFrameEnum.GLOBAL:
                goal_pose_ = state.mission_plan.goal_pose.to_frame(ObjectFrameEnum.START,
                                                                   start_pose_abs=self.start_pose_global)
            else:
                raise ValueError("Frame argument not available. Should be 'start' or 'global'.")

            # Check map searching boundaries for the searcher
            points = np.array(self.lane_points)
            map_margin = 5.0
            map_size = [np.min(points[:, 0]) - map_margin, np.max(points[:, 0]) + map_margin,
                        np.min(points[:, 1]) - map_margin, np.max(points[:, 1]) + map_margin]
            # Search for waypoints
            # current_pose = [state.vehicle.pose.x, state.vehicle.pose.y, state.vehicle.pose.yaw]
            current_pose = plan_available_pose_on_lane([state.vehicle.pose.x, state.vehicle.pose.y],
                                                    self.roadgraph, current_yaw=state.vehicle.pose.yaw)
            goal_pose = plan_available_pose_on_lane([goal_pose_.x, goal_pose_.y], self.roadgraph)
            searcher = BiRRT(current_pose, goal_pose, self.lane_points, map_size, OFFSET=0.8)
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

        # Parallel parking mode. (Added by Summoning) TODO: To be integrate with parallel parking
        elif state.mission_plan.planner_type == PlannerEnum.PARALLEL_PARKING:
            print("I am in PARALLEL_PARKING mode")
            # TODO: Get cones information from perception, should have a AgentEnum for cones, for example AgentEnum.CONE
            # cones = []
            # for obj in state.agents.values():
            #     if obj.type == AgentEnum.CONE:
            #         cones.append(obj)

            """ BEGIN: For simulation test only. Delete after integration """""""""""""""
            def generate_turn(pose, turn_angle=45, turn_radius=3.0, turn_direction="right",
                              acceleration=5.0, deceleration=2.0, max_speed=1.0, num_points=10):
                theta = np.deg2rad(turn_angle)
                x, y, yaw = pose
                R = turn_radius
                if turn_direction == "left":
                    cx = x + R * np.cos(yaw - np.pi / 2)
                    cy = y + R * np.sin(yaw - np.pi / 2)
                    start_angle = yaw - np.pi / 2
                    angles = np.linspace(start_angle, start_angle - theta, num_points)
                    end_yaw = start_angle - theta
                elif turn_direction == "right":
                    cx = x + R * np.cos(yaw + np.pi / 2)
                    cy = y + R * np.sin(yaw + np.pi / 2)
                    start_angle = yaw + np.pi / 2
                    angles = np.linspace(start_angle, start_angle + theta, num_points)
                    end_yaw = start_angle + theta
                else:
                    raise ValueError('Invalid turn direction')
                end_yaw = (end_yaw + np.pi) % (2 * np.pi) - np.pi  # normalize angle
                waypoints = []
                for angle in angles:
                    xi = cx + R * np.cos(angle)
                    yi = cy + R * np.sin(angle)
                    waypoints.append([xi, yi])
                end_pose = waypoints[-1][0], waypoints[-1][1], end_yaw
                return waypoints, end_pose

            ### FOR SIMULATION TEST ONLY ###
            def generate_pull_over_route(pose, turn_angle=45, turn_radius=3.0, turn_direction="right"):
                waypoints1, end_pose = generate_turn(pose, turn_angle=turn_angle, turn_radius=turn_radius,
                                                     turn_direction=turn_direction)
                if turn_direction == "right":
                    waypoints2, _ = generate_turn(end_pose, turn_angle=turn_angle, turn_radius=turn_radius,
                                                  turn_direction="left")
                else:
                    waypoints2, _ = generate_turn(end_pose, turn_angle=turn_angle, turn_radius=turn_radius,
                                                  turn_direction="right")
                waypoints = waypoints1 + waypoints2
                return waypoints

            if not self.parking_route_existed:
                current_pose = [state.vehicle.pose.x, state.vehicle.pose.y, state.vehicle.pose.yaw]
                waypoints = generate_pull_over_route(current_pose, turn_angle=45, turn_radius=2.0, turn_direction="right")
                self.route = Route(frame=ObjectFrameEnum.START, points=waypoints)

                self.parking_route_existed = True
            else:
                self.route = state.route
            """ END """""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""

        else:
            print("Unknown mode")

        if self.route is not None:
            print("Route existed.")
        
        return self.route