import os
from typing import Dict, List

import numpy as np
from GEMstack.onboard.component import Component
from GEMstack.state.agent import AgentState
from GEMstack.state.mission import MissionEnum
from GEMstack.state.all import AllState
from GEMstack.state.mission_plan import MissionPlan
from GEMstack.state.physical_object import ObjectFrameEnum, ObjectPose
from GEMstack.state.route import PlannerEnum, Route
from GEMstack.state.vehicle import VehicleState
from .planner import optimized_kinodynamic_rrt_planning
from .map_utils import load_pgm_to_occupancy_grid
from .rrt_star import RRTStar
from typing import List
from ..component import Component
from ...utils import serialization
from ...state import Route, ObjectFrameEnum
import math
import requests

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from .occupancy_grid2 import OccupancyGrid2
import cv2


ORIGIN_PX = (190, 80)
SCALE_PX_PER_M = 6.5  # px per meter


ORIGIN_PX       = (190, 80)
SCALE_PX_PER_M  = 6.5  # px per meter

class RoutePlanningComponent(Component):
    """Reads a route from disk and returns it as the desired route."""
    def __init__(self):
        print("Route Planning Component init")
        self.planner = None
        self.route = None
        self.bridge = CvBridge()
        self.img_pub = rospy.Publisher("/image_with_car_xy", Image, queue_size=1)
        self.occupancy_grid = OccupancyGrid2()

    def state_inputs(self):
        return ["vehicle", "agents", "mission_plan"]

    def state_outputs(self) -> List[str]:
        return ['route']

    def rate(self):
        return 10.0

    def _car_to_pixel(self, x, y, img_w, img_h):
        # (x,y)[m] → (u,v)[px]
        u0, v0 = ORIGIN_PX
        u = int(round(u0 + SCALE_PX_PER_M * x))
        v = int(round(v0 - SCALE_PX_PER_M * y))

        # clamp to image bounds
        u = max(0, min(u, img_w - 1))
        v = max(0, min(v, img_h - 1))
        return u, v

    def _pixel_to_car(self, u, v, img_w, img_h):
        # clamp to image bounds
        u = max(0, min(u, img_w - 1))
        v = max(0, min(v, img_h - 1))

        # origin and scale (same as in _car_to_pixel)
        u0, v0 = ORIGIN_PX
        scale = SCALE_PX_PER_M

        # invert:
        #   u = u0 + scale * x   →   x = (u - u0) / scale
        #   v = v0 - scale * y   →   y = (v0 - v) / scale
        x = (u - u0) / scale
        y = (v0 - v) / scale

        return x, y

    def visualize_route_pixels(self, route_pts, start_pt, goal_pt):
        """
        route_pts: list of (u, v) in pixels
        start_pt:  (u, v) in pixels
        goal_pt:   (u, v) in pixels
        """
        # 1. Copy the base image
        script_dir = os.path.dirname(os.path.abspath(__file__))
        frame_path  = os.path.join(script_dir, "out.pgm")
        frame = cv2.imread(frame_path, cv2.IMREAD_COLOR)
        img_h, img_w = frame.shape[:2]
        print("frame shape", frame.shape)
        print("frame dtype", frame.dtype)

        # 2. Optionally clamp all points to image bounds
        def clamp(pt):
            u, v = pt
            u = max(0, min(int(round(u)), img_w - 1))
            v = max(0, min(int(round(v)), img_h - 1))
            return (u, v)

        pts = np.array([clamp(p) for p in route_pts], dtype=np.int32).reshape(-1, 1, 2)

        # 3. Draw the route polyline in green
        cv2.polylines(
            frame,
            [pts],
            isClosed=False,
            color=(0, 255, 0),
            thickness=2
        )

        # 4. Draw start marker in blue (star)
        u_s, v_s = clamp(start_pt)
        cv2.drawMarker(
            frame,
            (u_s, v_s),
            color=(255, 0, 0),
            markerType=cv2.MARKER_STAR,
            markerSize=20,
            thickness=3
        )

        # 5. Draw goal marker in red (tilted cross)
        u_g, v_g = clamp(goal_pt)
        cv2.drawMarker(
            frame,
            (u_g, v_g),
            color=(0, 0, 255),
            markerType=cv2.MARKER_TILTED_CROSS,
            markerSize=20,
            thickness=3
        )

        # 6. Publish via ROS
        out = self.bridge.cv2_to_imgmsg(frame, "bgr8")
        print("Publishing out: ", out)
        out.header.stamp = rospy.Time.now()
        self.img_pub.publish(out)

    def update(self, vehicle: VehicleState, agents: Dict[str, AgentState], mission_plan: MissionPlan) -> Route:
        print("agent", agents.items())
        print("vehicle", vehicle.pose.frame)
        print("Converting to GNSS frame", vehicle.pose.to_frame(ObjectFrameEnum.GLOBAL, start_pose_abs = mission_plan.start_vehicle_pose))
        vehicle_global_pose = vehicle.pose.to_frame(ObjectFrameEnum.GLOBAL, start_pose_abs = mission_plan.start_vehicle_pose)
        # for n, a in agents.items():
        #     print("==========================\nAgent:", n, a.pose, a.velocity)
        #     print('==============', a.pose.frame==ObjectFrameEnum.START)
        #     print('==============', a.type==AgentEnum.PEDESTRIAN)
        #     if a.type == AgentEnum.PEDESTRIAN:
        #         print("Pedestrian detected")
        #         # Do something with the pedestrian information

        print("Route Planner's mission:", mission_plan.planner_type.value)
        print("type of mission plan:", type(PlannerEnum.RRT_STAR))
        print("Route Planner's mission:", mission_plan.planner_type.value == PlannerEnum.RRT_STAR.value)
        print("Route Planner's mission:", mission_plan.planner_type.value == PlannerEnum.PARKING.value)
        print("Mission plan:", mission_plan)
        print("Vehicle x:", vehicle.pose.x)
        print("Vehicle y:", vehicle.pose.y)
        print("Vehicle yaw:", vehicle.pose.yaw)

        ## Step. 1 Convert vehicle pose to global frame
        vehicle_global_pose = vehicle.pose.to_frame(ObjectFrameEnum.GLOBAL, start_pose_abs = mission_plan.start_vehicle_pose)
        # self.occupancy_grid.gnss_to_image(vehicle_global_pose.x, vehicle_global_pose.y) # Brijesh check if x corresponds to lon and y corresponds to lat. If not SWAP
        self.occupancy_grid.gnss_to_image(vehicle_global_pose.x, vehicle_global_pose.y)

        ## Step 2: Get start image coordinates aka position of vehicle in image
        start_x, start_y = self.occupancy_grid.gnss_to_image_coords(vehicle_global_pose.x, vehicle_global_pose.y) # Brijesh check if x corresponds to lon and y corresponds to lat. If not SWAP
        start_yaw = vehicle_global_pose.yaw
        print("Start image coordinates", start_x, start_y, "yaw", start_yaw)

        ## Step 3. Convert goal to global frame
        goal_global_pose = mission_plan.goal_vehicle_pose.to_frame(ObjectFrameEnum.GLOBAL, start_pose_abs = mission_plan.start_vehicle_pose)
        goal_x, goal_y = self.occupancy_grid.gnss_to_image_coords(goal_global_pose.x, goal_global_pose.y) # Brijesh check if x corresponds to lon and y corresponds to lat. If not SWAP
        goal_yaw = goal_global_pose.yaw
        print("Goal image coordinates", goal_x, goal_y, "yaw", goal_yaw)
        # self.occupancy_grid[start_x-5:start_x +5][start_y-5] = 1
        if mission_plan.planner_type.value == PlannerEnum.PARKING.value:
            print("I am in PARKING mode")
            # Return a route after doing some processing based on mission plan REMOVE ONCE OTHER PLANNERS ARE IMPLEMENTED
            base_path = os.path.dirname(__file__)
            file_path = os.path.join(base_path, "../../knowledge/routes/forward_15m_extra.csv")

            waypoints = np.loadtxt(file_path, delimiter=',', dtype=float)
            if waypoints.shape[1] == 3:
                waypoints = waypoints[:,:2]
            print("waypoints", waypoints)
            self.route = Route(frame=ObjectFrameEnum.START,points=waypoints.tolist())
        elif mission_plan.planner_type.value == PlannerEnum.RRT_STAR.value:
            print("I am in RRT mode")
            # start = (state.vehicle.pose.x, state.vehicle.pose.y)
            script_dir = os.path.dirname(os.path.abspath(__file__))
            map_path  = os.path.join(script_dir, "highbay_image.pgm")

            print("map_path", map_path)

            map_img = cv2.imread(map_path, cv2.IMREAD_UNCHANGED)
            occupancy_grid = (map_img > 0).astype(np.uint8) #Brijesh check what this does, I assume black is free and white is occupied so this would return white for everything that is not 100% black
            x_bounds = (0,occupancy_grid.shape[1])
            y_bounds = (0,occupancy_grid.shape[0])
            start = (start_x, start_y) # add start_yaw. @Sai I have not integrated yaw into kinodynamic yet. but plls add
            goal = (goal_x, goal_y) # add goal_yaw. @Sai I have not integrated yaw into kinodynamic yet. but plls add
            step_size = 1.0
            max_iter = 2000
            self.planner = RRTStar(start, goal, x_bounds, y_bounds, max_iter=max_iter, step_size=step_size, vehicle_width=1, occupancy_grid=occupancy_grid)
            print("RRT mode")
            rrt_resp = self.planner.plan()
            # self.visualize_route_pixels(rrt_resp, start, goal)
            for i in range(len(rrt_resp)):
                x, y = rrt_resp[i]
                # Convert to car coordinates
                waypoint_lat, waypoint_lon = self.occupancy_grid.image_to_gnss(x, y) # Converts pixel to global frame. Brijesh check again what x corresponds to. Is x lat or is x lon? Change accordingly. Same as above comments
                # Convert global to start frame
                waypoint_start_pose = ObjectPose.from_frame(ObjectFrameEnum.GLOBAL, ObjectFrameEnum.START, waypoint_lat, waypoint_lon, 0.0, start_pose_abs=mission_plan.start_vehicle_pose) #not handling yaw cuz we don't know how to
                rrt_resp[i] = (waypoint_start_pose.x, waypoint_start_pose.y)

            print("Route points in start frame: ", rrt_resp) # Comment this out once you are done debugging
            self.route = Route(frame=ObjectFrameEnum.START, points=rrt_resp)
            # print("Route planning complete")

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
    flag_full_circle = True
    tangent_min = 0
    min_index = 0

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
            # Calculate the tangent heading in a clockwise direction
            tangent_heading = -np.arctan2(y - yc, x - xc)  # Clockwise heading
            if abs(1 - tangent_heading) > abs(1 - tangent_min):
                if np.arctan2(yc, xc) < np.arctan2(y, x):
                    tangent_min = tangent_heading
                    min_index = i
        else:
            flag_full_circle = False
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
        max_arc[i] = np.array(max_arc[i])
        np.append(max_arc[i], heading_on_circle(xc, yc, max_arc[0][0], max_arc[1][0]))
        # np.append(max_arc[i], 0)

    if flag_full_circle:
        max_arc = max_arc[min_index:] + max_arc[:min_index]

    return max_arc[::-1]


def heading_on_circle(cx, cy, px, py):
    dx = px - cx
    dy = py - cy
    tx = -dy
    ty = dx
    return math.atan2(ty, tx)  # Heading in radians


def check_point_exists(vehicle, start_pose, server_url="http://localhost:8000"):
    print("Vehicle pose frame", vehicle.pose.frame)
    # vehicle_global_pose = vehicle.pose.to_frame(ObjectFrameEnum.GLOBAL, start_frame=vehicle.pose)
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
            pt1 = pt1.to_frame(ObjectFrameEnum.START, start_pose_abs=start_pose)
            pt2 = pt2.to_frame(ObjectFrameEnum.START, start_pose_abs=start_pose)
            return True, [[pt1.x, pt1.y], [pt2.x, pt2.y]]
        return False, []

    except requests.exceptions.RequestException as e:
        print("Error contacting server:", e)
        return False, []


class InspectRoutePlanner(Component):
    """Reads a route from disk and returns it as the desired route."""

    def __init__(self, state_machine, frame: str = "start"):
        self.geofence_area = [[0, 0], [20, 20]]
        self.state_list = state_machine
        self.index = 1
        self.mission = self.state_list[self.index]
        self.circle_center = [10, 17.5]
        self.radius = 5
        self.inspection_route = max_visible_arc(
            self.circle_center, self.radius, self.geofence_area
        )
        self.start = [0, 0]
        self.bridge = CvBridge()
        self.img_pub = rospy.Publisher("/image_with_car_xy", Image, queue_size=1)
        self.occupancy_grid = OccupancyGrid2()
        self.planned_path_already = False
        self.x = None

    def state_inputs(self):
        return ["all"]

    def state_outputs(self) -> List[str]:
        return ["route", "mission"]

    def rate(self):
        return 2.0

    def _car_to_pixel(self, x, y, img_w, img_h):
        # (x,y)[m] → (u,v)[px]
        u0, v0 = ORIGIN_PX
        u = int(round(u0 + SCALE_PX_PER_M * x))
        v = int(round(v0 - SCALE_PX_PER_M * y))

        # clamp to image bounds
        u = max(0, min(u, img_w - 1))
        v = max(0, min(v, img_h - 1))
        return u, v

    def _pixel_to_car(self, u, v, img_w, img_h):
        # clamp to image bounds
        u = max(0, min(u, img_w - 1))
        v = max(0, min(v, img_h - 1))

        # origin and scale (same as in _car_to_pixel)
        u0, v0 = ORIGIN_PX
        scale = SCALE_PX_PER_M

        # invert:
        #   u = u0 + scale * x   →   x = (u - u0) / scale
        #   v = v0 - scale * y   →   y = (v0 - v) / scale
        x = (u - u0) / scale
        y = (v0 - v) / scale

        return x, y

    def visualize_route_pixels(self, route_pts, start_pt, goal_pt):
        """
        route_pts: list of (u, v) in pixels
        start_pt:  (u, v) in pixels
        goal_pt:   (u, v) in pixels
        """
        # 1. Copy the base image
        script_dir = os.path.dirname(os.path.abspath(__file__))
        frame_path = os.path.join(script_dir, "out.pgm")
        frame = cv2.imread(frame_path, cv2.IMREAD_COLOR)
        img_h, img_w = frame.shape[:2]
        # print("frame shape", frame.shape)
        # print("frame dtype", frame.dtype)

        # 2. Optionally clamp all points to image bounds
        def clamp(pt):
            u, v = pt
            u = max(0, min(int(round(u)), img_w - 1))
            v = max(0, min(int(round(v)), img_h - 1))
            return (u, v)

        pts = np.array([clamp(p) for p in route_pts], dtype=np.int32).reshape(-1, 1, 2)

        # 3. Draw the route polyline in green
        cv2.polylines(frame, [pts], isClosed=False, color=(0, 255, 0), thickness=2)

        # 4. Draw start marker in blue (star)
        u_s, v_s = clamp(start_pt)
        cv2.drawMarker(
            frame,
            (u_s, v_s),
            color=(255, 0, 0),
            markerType=cv2.MARKER_STAR,
            markerSize=20,
            thickness=3,
        )

        # 5. Draw goal marker in red (tilted cross)
        u_g, v_g = clamp(goal_pt)
        cv2.drawMarker(
            frame,
            (u_g, v_g),
            color=(0, 0, 255),
            markerType=cv2.MARKER_TILTED_CROSS,
            markerSize=20,
            thickness=3,
        )

        # 6. Publish via ROS
        out = self.bridge.cv2_to_imgmsg(frame, "bgr8")
        # print("Publishing out: ", out)
        out.header.stamp = rospy.Time.now()
        self.img_pub.publish(out)

    def rrt_route(self, state, goal_start_pose):
        ## Step. 1 Convert vehicle pose to global frame
        vehicle_global_pose = state.vehicle.pose.to_frame(ObjectFrameEnum.GLOBAL, start_pose_abs = state.start_vehicle_pose)
        self.occupancy_grid.gnss_to_image(vehicle_global_pose.x, vehicle_global_pose.y) # Brijesh check if x corresponds to lon and y corresponds to lat. If not SWAP
        # self.occupancy_grid.gnss_to_image(-88.236365, 40.092722)

        ## Step 2: Get start image coordinates aka position of vehicle in image
        start_x, start_y = self.occupancy_grid.gnss_to_image_coords(vehicle_global_pose.x, vehicle_global_pose.y) # Brijesh check if x corresponds to lon and y corresponds to lat. If not SWAP
        # start_x, start_y = self.occupancy_grid.gnss_to_image_coords(40.092722, -88.236365)
        start_yaw = vehicle_global_pose.yaw
        print("Start image coordinates", start_x, start_y, "yaw", start_yaw)

        ## Step 3. Convert goal to global frame
        goal_global_pose = goal_start_pose.to_frame(ObjectFrameEnum.GLOBAL, start_pose_abs = state.start_vehicle_pose)
        goal_x, goal_y = self.occupancy_grid.gnss_to_image_coords(goal_global_pose.x, goal_global_pose.y) # Brijesh check if x corresponds to lon and y corresponds to lat. If not SWAP
        # goal_x, goal_y = self.occupancy_grid.gnss_to_image_coords(40.092889, -88.235686)
        goal_yaw = goal_global_pose.yaw
        print("Goal image coordinates", goal_x, goal_y, "yaw", goal_yaw)

        script_dir = os.path.dirname(os.path.abspath(__file__))
        map_path  = os.path.join(script_dir, "highbay_image.pgm")

        print("map_path", map_path)

        map_img = cv2.imread(map_path, cv2.IMREAD_UNCHANGED)
        occupancy_grid = (map_img > 0).astype(np.uint8) #Brijesh check what this does, I assume black is free and white is occupied so this would return white for everything that is not 100% black
        # x_b
        self.t_last = Noneounds = (0,occupancy_grid.shape[1])
        # y_bounds = (0,occupancy_grid.shape[0])
        # start = (start_x, start_y) # add start_yaw. @Sai I have not integrated yaw into kinodynamic yet. but plls add
        # goal = (goal_x, goal_y) # add goal_yaw. @Sai I have not integrated yaw into kinodynamic yet. but plls add
        # step_size = 1.0
        # max_iter = 2000

        script_dir = os.path.dirname(os.path.abspath(__file__))
        map_path  = os.path.join(script_dir, "highbay_image.pgm")
        
        # occupancy_grid = load_pgm_to_occupancy_grid(map_path)
        start_w = [start_y, start_x, start_yaw]
        goal_w = [goal_y, goal_x, goal_yaw]
        # occupancy_grid[start_x-5:start_x +5][start_y-5] = 1

        path = optimized_kinodynamic_rrt_planning(
            start_w, goal_w, occupancy_grid
        )
        
        
        
        print("RRT mode", path)
        # rrt_resp = self.planner.plan()
        # self.visualize_route_pixels(rrt_resp, start, goal)
        waypoints = []
        for i in range(len(path)):
            x, y, theta = path[i]
            # Convert to car coordinates
            waypoint_lat, waypoint_lon = self.occupancy_grid.image_to_gnss(y, x) # Converts pixel to global frame. Brijesh check again what x corresponds to. Is x lat or is x lon? Change accordingly. Same as above comments
            # Convert global to start frame
            waypoint_global_pose = ObjectPose(
                frame=ObjectFrameEnum.GLOBAL,
                t=state.start_vehicle_pose.t,
                x=waypoint_lon,
                y=waypoint_lat,
                yaw=theta,
            )
            waypoint_start_pose = waypoint_global_pose.to_frame(ObjectFrameEnum.START, start_pose_abs=state.start_vehicle_pose) #not handling yaw cuz we don't know how to
            waypoints.append((waypoint_start_pose.x, waypoint_start_pose.y))

        print("Route points in start frame: ", waypoints) # Comment this out once you are done debugging
        return Route(frame=ObjectFrameEnum.START, points=waypoints)

    def update(self, state):
        self.flag = 0
        self.route = Route(frame=ObjectFrameEnum.START, points=((0, 0, 0)))
        print("Mode ", state.mission.type)
        print("Mission state:", self.mission)
        if self.mission == "IDLE":
            state.mission.type = MissionEnum.IDLE
            points_found, pts = check_point_exists(
                state.vehicle, state.start_vehicle_pose
            )
            if points_found:
                self.inspection_area = pts
                print("Inspection coordinates:", self.inspection_area)
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
            
            print("I MMMMMMMMMMMMM HEEEEERRRRRRRRRRRRRRReeeeeeeeeeeee")
            
            start = (state.vehicle.pose.x, state.vehicle.pose.y)
            goal = ObjectPose(
                frame=ObjectFrameEnum.START,
                t=state.start_vehicle_pose.t,
                x=state.vehicle.pose.x + 15,
                y=state.vehicle.pose.y,
                yaw=0.0,
            )
            print("Current Position: ", start)
            print("Goal Position: ", goal)
            if abs(state.vehicle.pose.x - goal.x) <= 1 and abs(state.vehicle.pose.y - goal.y) <= 1:
                print(self.state_list[self.index + 1])
                self.mission = self.state_list[self.index + 1]
                self.index += 1
                print("CHANGING STATES", self.mission)
            
            if self.planned_path_already == False:
                self.x = self.rrt_route(state, goal)
                    # self.route = self.rrt_route(state, goal)
                self.planned_path_already = True
            # self.route = Route(
            #     frame=ObjectFrameEnum.START, points=self.route
            # )
            self.route = self.x
            
                

        elif self.mission == "INSPECT":
            state.mission.type = MissionEnum.INSPECT
            start = (state.vehicle.pose.x + 1, state.vehicle.pose.y + 1)
            goal = (self.inspection_route[-1][0], self.inspection_route[-1][1])
            # if abs(start[0] - goal[0]) <= 1 and abs(start[1] - goal[1]) <= 1: # and self.flag>1:
            #     print(self.state_list[self.index + 1])
            #     self.mission = self.state_list[self.index + 1]
            #     self.index += 1
            #     print("CHANGING STATES", self.mission)
            self.flag += 0.1
            self.route = Route(
                frame=ObjectFrameEnum.START, points=self.inspection_route
            )
        elif self.mission == "FINISH":
            state.mission.type = MissionEnum.INSPECT_UPLOAD
            goal = state.start_vehicle_pose

            if abs(state.vehicle.pose.x - goal.x) <= 1 and abs(state.vehicle.pose.y - goal.y) <= 1:
                print(self.state_list[self.index + 1])
                self.mission = self.state_list[self.index + 1]
                self.index += 1
                print("CHANGING STATES", self.mission)

            self.route = self.rrt_route(state, goal)

        print('-------------------------------------------------')
        print(self.route)
        return [self.route, state.mission]
