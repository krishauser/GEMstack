import os
from typing import Dict, List

import numpy as np
from GEMstack.onboard.component import Component
from GEMstack.state.agent import AgentState
from GEMstack.state.mission import MissionEnum
from GEMstack.state.all import AllState
from GEMstack.state.physical_object import ObjectFrameEnum, ObjectPose
from GEMstack.state.route import PlannerEnum, Route
from .rrt_star import RRTStar
from typing import List
from ..component import Component
from ...utils import serialization
from ...state import Route, ObjectFrameEnum
import math
import requests


class RoutePlanningComponent(Component):
    """Reads a route from disk and returns it as the desired route."""

    def __init__(self):
        print("Route Planning Component init")
        self.planner = None
        self.route = None

    def state_inputs(self):
        return ["all"]

    def state_outputs(self) -> List[str]:
        return ["route"]

    def rate(self):
        return 10.0

    def update(self, state: AllState):
        # print("Route Planner's mission:", state.mission_plan.planner_type.value)
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
            file_path = os.path.join(
                base_path, "../../knowledge/routes/forward_15m_extra.csv"
            )

            waypoints = np.loadtxt(file_path, delimiter=",", dtype=float)
            if waypoints.shape[1] == 3:
                waypoints = waypoints[:, :2]
            print("waypoints", waypoints)
            self.route = Route(frame=ObjectFrameEnum.START, points=waypoints.tolist())
        elif state.mission_plan.planner_type.value == PlannerEnum.RRT_STAR.value:
            print("I am in RRT mode")
            start = (state.vehicle.pose.x + 1, state.vehicle.pose.y + 1)
            goal = (
                state.mission_plan.goal_x,
                state.mission_plan.goal_y,
            )  # When we implement kinodynamic, we need to include target_yaw also
            x_bounds = (0, 20)
            y_bounds = (0, 20)
            step_size = 1.0
            max_iter = 2000
            occupancy_grid = np.zeros((20, 20), dtype=int)
            occupancy_grid[5:10, 5:10] = 1
            self.planner = RRTStar(
                start,
                goal,
                x_bounds,
                y_bounds,
                max_iter=max_iter,
                step_size=step_size,
                vehicle_width=1,
                occupancy_grid=occupancy_grid,
            )
            rrt_resp = self.planner.plan()
            self.route = Route(frame=ObjectFrameEnum.START, points=rrt_resp)
        else:
            print("Unknown mode")

        return self.route


def is_inside_geofence(x, y, xmin, xmax, ymin, ymax):
    return xmin < x < xmax and ymin < y < ymax


def max_visible_arc(circle_center, radius, geofence):
    xc, yc = circle_center
    (xmin, ymin), (xmax, ymax) = geofence

    angles = np.linspace(0, 2 * np.pi, 500, endpoint=False)
    arc_segments = []
    curr_segment = []

    first_inside = last_inside = False

    for i, theta in enumerate(angles):
        x = xc + radius * np.cos(theta)
        y = yc + radius * np.sin(theta)

        inside = is_inside_geofence(x, y, xmin, xmax, ymin, ymax)

        if i == 0:
            first_inside = inside
        if i == len(angles) - 1:
            last_inside = inside

        if inside:
            curr_segment.append((x, y))
        else:
            if curr_segment:
                arc_segments.append(curr_segment)
                curr_segment = []

    if curr_segment:
        arc_segments.append(curr_segment)

    # If arc wraps around from 2π back to 0, combine first and last segments
    if first_inside and last_inside and len(arc_segments) > 1:
        arc_segments[0] = arc_segments[-1] + arc_segments[0]
        arc_segments.pop()

    if not arc_segments:
        return []

    max_arc = list(max(arc_segments, key=len))
    for i in range(len(max_arc)):
        # max_arc[i].append(heading_on_circle(xc,yc,max_arc[0], max_arc[1]))
        max_arc[i] = np.array(max_arc[i])
        np.append(max_arc[i], 0)
    return max_arc


def heading_on_circle(cx, cy, px, py):
    dx = px - cx
    dy = py - cy
    tx = -dy
    ty = dx
    return math.atan2(ty, tx)  # Heading in radians


def check_point_exists(vehicle, server_url="http://localhost:8000"):
    try:
        response = requests.get(f"{server_url}/api/inspect")
        response.raise_for_status()
        points = response.json().get("coords", [])

        if points:
            pt1 = ObjectPose(
                frame=ObjectFrameEnum.GLOBAL,
                t=0,
                x=points[0]["lng"],
                y=points[0]["lat"],
            )
            pt2 = ObjectPose(
                frame=ObjectFrameEnum.GLOBAL,
                t=0,
                x=points[1]["lng"],
                y=points[1]["lat"],
            )
            pt1.to_frame(ObjectFrameEnum.START)
            pt2.to_frame(ObjectFrameEnum.START)
            return True, [[pt1.x, pt1.y], [pt2.x, pt2.y]]
        return False, []

    except requests.exceptions.RequestException as e:
        print("Error contacting server:", e)
        return False, []


class InspectRoutePlanner(Component):
    """Reads a route from disk and returns it as the desired route."""

    def __init__(self, state_machine, frame: str = "start"):
        self.geofence_area = [[0, 0], [40, 40]]
        self.state_list = state_machine
        self.index = 1
        self.mission = self.state_list[self.index]
        self.circle_center = [30, 30]
        self.radius = 20
        self.inspection_route = max_visible_arc(
            self.circle_center, self.radius, self.geofence_area
        )
        self.start = [0, 0]

    def state_inputs(self):
        return ["all"]

    def state_outputs(self) -> List[str]:
        return ["route"]

    def rate(self):
        return 1.0

    def update(self, state):
        self.flag = 0
        self.route = Route(frame=ObjectFrameEnum.START, points=((0, 0, 0)))
        if self.mission == "IDLE":
            state.mission.type = MissionEnum.IDLE
            points_found = False
            points_found, pts = check_point_exists(state.vehicle)
            if points_found:
                self.inspection_area = pts
                print(self.state_list[self.index + 1])
                self.mission = self.state_list[self.index + 1]
                self.index += 1
                print("CHANGING STATES", self.mission)
                self.start = [state.vehicle.pose.x, state.vehicle.pose.y]
            self.circle_center = [
                (self.inspection_area[0][0] + self.inspection_area[1][0]) / 2,
                (self.inspection_area[0][1] + self.inspection_area[1][1]) / 2,
            ]
            self.radius = (
                (self.inspection_area[0][0] + self.inspection_area[1][0]) ** 2
                + (self.inspection_area[0][1] + self.inspection_area[1][1]) ** 2
            ) ** 0.5 / 2
            self.inspection_route = max_visible_arc(
                self.circle_center, self.radius, self.geofence_area
            )
        elif self.mission == "NAV":
            state.mission.type = MissionEnum.DRIVE
            start = (state.vehicle.pose.x + 1, state.vehicle.pose.y + 1)
            goal = (self.inspection_route[0][0] - 3, self.inspection_route[0][1] - 3)
            if abs(start[0] - goal[0]) <= 1 and abs(start[1] - goal[1]) <= 1:
                print(self.state_list[self.index + 1])
                self.mission = self.state_list[self.index + 1]
                self.index += 1
                print("CHANGING STATES", self.mission)
            x_bounds = (0, 50)
            y_bounds = (0, 50)
            step_size = 1.0
            max_iter = 2000
            occupancy_grid = np.zeros((50, 50), dtype=int)
            # occupancy_grid[5:10, 5:10] = 1
            self.planner = RRTStar(
                start,
                goal,
                x_bounds,
                y_bounds,
                max_iter=max_iter,
                step_size=step_size,
                vehicle_width=1,
                occupancy_grid=occupancy_grid,
            )
            rrt_resp = self.planner.plan()
            self.route = Route(frame=ObjectFrameEnum.START, points=rrt_resp)
        elif self.mission == "INSPECT":
            state.mission.type = MissionEnum.INSPECT
            start = (state.vehicle.pose.x + 1, state.vehicle.pose.y + 1)
            goal = (self.inspection_route[-1][0], self.inspection_route[-1][1])
            if abs(start[0] - goal[0]) <= 1 and abs(start[1] - goal[1]) <= 1:
                print(self.state_list[self.index + 1])
                self.mission = self.state_list[self.index + 1]
                self.index += 1
                print("CHANGING STATES", self.mission)
            self.flag += 0.1
            self.route = Route(
                frame=ObjectFrameEnum.START, points=self.inspection_route
            )
        elif self.mission == "FINISH":
            state.mission.type = MissionEnum.INSPECT_UPLOAD
            start = (state.vehicle.pose.x + 1, state.vehicle.pose.y + 1)
            goal = (self.start[0], self.start[1])
            if abs(start[0] - goal[0]) <= 1 and abs(start[1] - goal[1]) <= 1:
                print(self.state_list[self.index + 1])
                self.mission = self.state_list[self.index + 1]
                self.index += 1
                print("CHANGING STATES", self.mission)
            x_bounds = (0, 50)
            y_bounds = (0, 50)
            step_size = 1.0
            max_iter = 2000
            occupancy_grid = np.zeros((50, 50), dtype=int)
            self.planner = RRTStar(
                start,
                goal,
                x_bounds,
                y_bounds,
                max_iter=max_iter,
                step_size=step_size,
                vehicle_width=1,
                occupancy_grid=occupancy_grid,
            )
            rrt_resp = self.planner.plan()
            self.route = Route(frame=ObjectFrameEnum.START, points=rrt_resp)

        return self.route
