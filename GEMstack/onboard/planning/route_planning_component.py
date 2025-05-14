import os
from typing import List

import numpy as np

from GEMstack.onboard.component import Component
from GEMstack.state.mission import MissionEnum
from GEMstack.state.all import AllState
from GEMstack.state.physical_object import ObjectFrameEnum, ObjectPose
from GEMstack.state.route import PlannerEnum, Route
from GEMstack.state.vehicle import VehicleState
from GEMstack.state.agent import AgentState
from GEMstack.state.intent import VehicleIntentEnum
from GEMstack.state.mission_plan import MissionPlan, ModeEnum
from GEMstack.state.obstacle import Obstacle, ObstacleMaterialEnum
from .planner import optimized_kinodynamic_rrt_planning
from typing import List
from ..component import Component
from ...state import Route, ObjectFrameEnum
import math
import requests

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from .occupancy_grid import OccupancyGrid
import cv2


######################################################################
#################### PLANNING HORIZONTAL #############################
######################################################################
class RoutePlanningComponentExample(Component):
    """Reads a route from disk and returns it as the desired route."""
    def __init__(self):
        print("Route Planning Component init")
        self.planner = None
        self.route = None
        self.bridge = CvBridge()
        self.occupancy_grid = OccupancyGrid()
        self.img_pub = rospy.Publisher("/occupancy_grid", Image, queue_size=1)
        self.previous_obstacles = None
        self.frame = None

    def state_inputs(self):
        return ["vehicle", "mission_plan", "obstacles"]

    def state_outputs(self) -> List[str]:
        return ['route']

    def rate(self):
        return 10.0
    

    def update(self, vehicle: VehicleState, mission_plan: MissionPlan, obstacles: Obstacle) -> Route:

        if self.frame is None:
            if mission_plan.mode == ModeEnum.HARDWARE:
                self.frame = ObjectFrameEnum.GLOBAL
            else: #simulation
                self.frame = ObjectFrameEnum.ABSOLUTE_CARTESIAN

        if self.previous_obstacles is None:
            self.previous_obstacles = len(obstacles)

        # Convert vehicle pose to global frame
        vehicle_global_pos = vehicle.pose.to_frame(self.frame, start_pose_abs=mission_plan.start_vehicle_pose)

        # Convert vehicle pose to image coordinates
        self.occupancy_grid.gnss_to_image(vehicle_global_pos.x, vehicle_global_pos.y)

        obstacles_global_poses = []
        if DEBUG:
            print("Number of Detected Obstacles: ", len(obstacles))
        for n, o in obstacles.items():
            print("Obstacle: ", o)
            if o.material == ObstacleMaterialEnum.TRAFFIC_CONE:
                obstacle_global_pose = o.pose.to_frame(self.frame, start_pose_abs=mission_plan.start_vehicle_pose)
                obstacles_global_poses.append((obstacle_global_pose.x, obstacle_global_pose.y))
            

        rects = self.occupancy_grid.draw_obstacle_cones(obstacles_global_poses)

        if DEBUG:
            print("Route Planner's mission:", mission_plan.planner_type.value)
            print("type of mission plan:", type(PlannerEnum.RRT_STAR))
            print("Vehicle x:", vehicle.pose.x)
            print("Vehicle y:", vehicle.pose.y)
            print("Vehicle yaw:", vehicle.pose.yaw)

        if mission_plan.planner_type.value == PlannerEnum.PARKING.value:
            if DEBUG:
                print("Route Planning in PARKING mode")
            base_path = os.path.dirname(__file__)
            file_path = os.path.join(base_path, "../../knowledge/routes/forward_15m_extra.csv")
            waypoints = np.loadtxt(file_path, delimiter=',', dtype=float)
            if waypoints.shape[1] == 3:
                    waypoints = waypoints[:,:2]
            self.route = Route(frame=ObjectFrameEnum.START,points=waypoints.tolist())
        elif mission_plan.planner_type.value == PlannerEnum.RRT_STAR.value:
            if DEBUG:
                print("Route Planning in RRT mode")

            ## Step 1: Convert vehicle pose to global frame
            vehicle_global_pose = vehicle.pose.to_frame(
                self.frame, start_pose_abs=mission_plan.start_vehicle_pose
            )

            # Plot motion image
            self.occupancy_grid.gnss_to_image_with_heading(
                vehicle_global_pose.x, vehicle_global_pose.y, vehicle_global_pose.yaw
            )  
            ## Step 2: Get start image coordinates aka position of vehicle in image
            start_x, start_y = self.occupancy_grid.gnss_to_image_coords(
                vehicle_global_pose.x, vehicle_global_pose.y
            ) 

            start_yaw = vehicle.pose.yaw + math.pi
            if DEBUG:
                print("Start image coordinates", start_x, start_y, "yaw", start_yaw)

            ## Step 3. Convert goal to global frame
            goal_global_pose = mission_plan.goal_vehicle_pose.to_frame(
                self.frame, start_pose_abs=mission_plan.start_vehicle_pose
            )
            
            goal_x, goal_y = self.occupancy_grid.gnss_to_image_coords(
                goal_global_pose.x, goal_global_pose.y
            )  

            goal_yaw = start_yaw
            if DEBUG:
                print("Goal image coordinates", goal_x, goal_y, "yaw", goal_yaw)

            script_dir = os.path.dirname(os.path.abspath(__file__))
            map_path = os.path.join(script_dir, "highbay_image.pgm")
            map_img = cv2.imread(map_path, cv2.IMREAD_UNCHANGED)

            for i, (rect_pt1_x, rect_pt1_y, rect_pt2_x, rect_pt2_y) in enumerate(rects):
                if rect_pt1_x < rect_pt2_x and rect_pt1_y < rect_pt2_y:
                    # It's a rectangle with areaf
                    cv2.rectangle(map_img, (rect_pt1_x, rect_pt1_y), (rect_pt2_x, rect_pt2_y), (255, 255, 255), 6)  # White
                    if DEBUG:
                        print(f"[DEBUG-ME] Drew WHITE rectangle for cluster {i}")
                elif rect_pt1_x == rect_pt2_x and rect_pt1_y < rect_pt2_y:
                    # It's a vertical line
                    cv2.line(map_img, (rect_pt1_x, rect_pt1_y), (rect_pt2_x, rect_pt2_y), (255, 255, 255), 6) # White line
                    if DEBUG:
                        print(f"[DEBUG-ME] Drew WHITE vertical line for cluster {i}")
                elif rect_pt1_y == rect_pt2_y and rect_pt1_x < rect_pt2_x:
                    # It's a horizontal line
                    cv2.line(map_img, (rect_pt1_x, rect_pt1_y), (rect_pt2_x, rect_pt2_y), (255, 255, 255), 6) # White line
                    if DEBUG:
                        print(f"[DEBUG-ME] Drew WHITE horizontal line for cluster {i}")
                else:
                    if DEBUG:
                        print(f"[DEBUG-ME] Cluster {i} BBox has zero/negative area and is not a simple line. rect_pt1_x={rect_pt1_x}, rect_pt2_x={rect_pt2_x}, rect_pt1_y={rect_pt1_y}, rect_pt2_y={rect_pt2_y}")

            cv2.imwrite("highbay_image_with_cones.pgm", map_img)
            occupancy_grid = (map_img > 0).astype(
                np.uint8
            ) 
            cv2.imwrite("occupancy_grid_after_>0.pgm", occupancy_grid * 255)
            self.t_last = None
            self.bounds = (0, occupancy_grid.shape[1])
            start_w = [start_x, start_y, start_yaw]
            goal_w = [goal_x, goal_y, goal_yaw]

            if self.route == None or len(obstacles) != self.previous_obstacles:
                    
                self.previous_obstacles = len(obstacles)
                path = optimized_kinodynamic_rrt_planning(start_w, goal_w, occupancy_grid)
                waypoints = []
                for i in range(len(path)):
                    x, y, theta = path[i]
                waypoints = []
                for i in range(0, len(path), 10):
                    x, y, theta = path[i]
                    # Converts pixel to global frame.
                    waypoint_lat, waypoint_lon = self.occupancy_grid.image_to_gnss(x, y)  

                    # Convert global to start frame
                    waypoint_global_pose = ObjectPose(
                        frame=self.frame,
                        t=mission_plan.start_vehicle_pose.t,
                        x=waypoint_lon,
                        y=waypoint_lat,
                        yaw=theta,
                    )
                    waypoint_start_pose = waypoint_global_pose.to_frame(
                        ObjectFrameEnum.START, start_pose_abs=mission_plan.start_vehicle_pose
                    )
                    waypoints.append((waypoint_start_pose.x, waypoint_start_pose.y))
                
                self.route = Route(
                    frame=ObjectFrameEnum.START, points=waypoints)
                if DEBUG:
                    print("Route points in start frame: ", waypoints)
            return self.route

        else:
            print("Unknown mode")
        
        return self.route



######################################################################
#################### INSPECTION VERTICAL #############################
######################################################################

# Constants for planning
ORIGIN_PX = (190, 80)
SCALE_PX_PER_M = 6.5
DEBUG = True


# Functions to dynamically calculate a circular or linear path around the inspection area
def is_inside_geofence(x, y, xmin, xmax, ymin, ymax):
    return xmin < x < xmax and ymin < y < ymax


def max_visible_arc(circle_center, radius, geofence):
    """Circular arc calculation around the inspection area."""
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

    if flag_full_circle:
        max_arc = max_arc[min_index:] + max_arc[:min_index]

    return max_arc[::-1]


def heading(cx, cy, px, py):
    """Calculate the heading of the vehicle wrt to a fixed point"""
    dx = px - cx
    dy = py - cy
    tx = -dy
    ty = dx
    return math.degrees(math.atan2(ty, tx))  # Heading in radians


def create_path_around_inspection(inspection_area, geofence, margin=1.0):
    """Linear path around the inspection area"""
    (ixmin, iymin), (ixmax, iymax) = inspection_area
    (gxmin, gymin), (gxmax, gymax) = geofence

    # Define top path
    top_y = iymax + margin
    bottom_y = iymin - margin

    top_possible = gymin < top_y < gymax
    bottom_possible = gymin < bottom_y < gymax

    top_path = (
        [(ixmin - 2, top_y), (ixmax + 2, top_y)] if top_possible else None
    )
    right_path = (
        [(ixmax + margin, top_y), (ixmax + margin, bottom_y)] if top_possible else None
    )
    bottom_path = (
        [(ixmax + margin, bottom_y), (ixmin - margin, bottom_y)]
        if bottom_possible
        else None
    )
    left_path = (
        [(ixmin - margin, bottom_y), (ixmin - margin, top_y)]
        if bottom_possible
        else None
    )

    if top_possible:
        full_path = top_path
    elif bottom_possible:
        full_path = bottom_path
    else:
        full_path = top_path[:len(top_path)/2] # only half part of the top

    return full_path


def check_point_exists(vehicle, start_pose, server_url="https://cs588-prod.up.railway.app"):
    """Querying the frontend to get the inspection area."""
    print("Vehicle pose frame", vehicle.pose.frame)
    try:
        response = requests.get(f"{server_url}/api/inspect")
        response.raise_for_status()
        points = response.json().get("coords", [])

        if points:
            pt1 = ObjectPose(
                frame=ObjectFrameEnum.GLOBAL,
                t=0,
                x=points[0]["lon"],
                y=points[0]["lat"],
            )
            pt2 = ObjectPose(
                frame=ObjectFrameEnum.GLOBAL,
                t=0,
                x=points[1]["lon"],
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
    """Inspection route planner that controls the state transition logic for the vertical behavior of the vehicle
    while inspection."""

    def __init__(self, state_machine, frame: str = "start"):
        self.geofence_area = [[-40, -40], [40, 40]]
        self.state_list = state_machine
        self.index = 0
        self.mission = self.state_list[self.index]
        self.circle_center = [10,5]
        self.radius = 2
        self.start = [0, 0]
        self.bridge = CvBridge()
        self.img_pub = rospy.Publisher("/image_with_car_xy", Image, queue_size=1)
        self.occupancy_grid = OccupancyGrid()
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
        vehicle_global_pose = state.vehicle.pose.to_frame(
            ObjectFrameEnum.GLOBAL, start_pose_abs=state.start_vehicle_pose
        )
        self.occupancy_grid.gnss_to_image(
            vehicle_global_pose.x, vehicle_global_pose.y
        )

        ## Step 2: Get start image coordinates aka position of vehicle in image
        start_x, start_y = self.occupancy_grid.gnss_to_image_coords(
            vehicle_global_pose.x, vehicle_global_pose.y
        )
        start_yaw = vehicle_global_pose.yaw
        print("Start image coordinates", start_x, start_y, "yaw", start_yaw)

        ## Step 3. Convert goal to global frame
        goal_global_pose = goal_start_pose.to_frame(
            ObjectFrameEnum.GLOBAL, start_pose_abs=state.start_vehicle_pose
        )
        goal_x, goal_y = self.occupancy_grid.gnss_to_image_coords(
            goal_global_pose.x, goal_global_pose.y
        )
        goal_yaw = goal_global_pose.yaw
        print("Goal image coordinates", goal_x, goal_y, "yaw", goal_yaw)

        script_dir = os.path.dirname(os.path.abspath(__file__))
        map_path = os.path.join(script_dir, "highbay_image.pgm")
        print("map_path", map_path)

        map_img = cv2.imread(map_path, cv2.IMREAD_UNCHANGED)
        occupancy_grid = (map_img > 0).astype(
            np.uint8
        )
        self.t_last = None
        self.bounds = (0, occupancy_grid.shape[1])

        script_dir = os.path.dirname(os.path.abspath(__file__))
        map_path = os.path.join(script_dir, "highbay_image.pgm")

        start_w = [start_y, start_x, start_yaw]
        goal_w = [goal_y, goal_x, goal_yaw]

        path = optimized_kinodynamic_rrt_planning(start_w, goal_w, occupancy_grid)

        waypoints = []
        for i in range(len(path)):
            x, y, theta = path[i]
            # Convert to car coordinates
            waypoint_lat, waypoint_lon = self.occupancy_grid.image_to_gnss(
                y, x
            )
            # Convert global to start frame
            waypoint_global_pose = ObjectPose(
                frame=ObjectFrameEnum.GLOBAL,
                t=state.start_vehicle_pose.t,
                x=waypoint_lon,
                y=waypoint_lat,
                yaw=theta,
            )
            waypoint_start_pose = waypoint_global_pose.to_frame(
                ObjectFrameEnum.START, start_pose_abs=state.start_vehicle_pose
            )
            waypoints.append((waypoint_start_pose.x, waypoint_start_pose.y))
            waypoint_global_pose = ObjectPose(
                frame=ObjectFrameEnum.GLOBAL,
                t=state.start_vehicle_pose.t,
                x=waypoint_lon,
                y=waypoint_lat,
                yaw=theta,
            )
            waypoint_start_pose = waypoint_global_pose.to_frame(
                ObjectFrameEnum.START, start_pose_abs=state.start_vehicle_pose
            )
            waypoints.append((waypoint_start_pose.x, waypoint_start_pose.y))

        return Route(frame=ObjectFrameEnum.START, points=waypoints)

    def update(self, state):
        # Default route to ensure that the IDLE state does not run into error
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

                ## Inspection through a circular arc
                # self.circle_center = [
                #     (self.inspection_area[0][0] + self.inspection_area[1][0]) / 2,
                #     (self.inspection_area[0][1] + self.inspection_area[1][1]) / 2,
                # ]
                # self.radius = (
                #     (self.inspection_area[0][0] + self.inspection_area[1][0]) ** 2
                #     + (self.inspection_area[0][1] + self.inspection_area[1][1]) ** 2
                # ) ** 0.5 / 2
                # self.inspection_route = max_visible_arc(
                #     self.circle_center, self.radius, self.geofence_area
                # )

                ## Inspection through a linear path
                self.inspection_route = create_path_around_inspection(
                    self.inspection_area, self.geofence_area
                )

        elif self.mission == "NAV":
            state.mission.type = MissionEnum.DRIVE

            start = (state.vehicle.pose.x, state.vehicle.pose.y)
            goal = ObjectPose(
                frame=ObjectFrameEnum.START,
                t=state.start_vehicle_pose.t,
                x=self.inspection_route[0][0],
                y=self.inspection_route[0][1],
                yaw=0,
            )
            print("Current Position: ", start)
            print("Goal Position: ", goal)

            if self.planned_path_already == False:
                ## Ensure that RRT star does not run for every iteration of the component
                self.x = self.rrt_route(state, goal)
                self.planned_path_already = True
            self.route = self.x

            ## GOAL condition
            if (abs(state.vehicle.pose.x - goal.x) <= 3 and abs(state.vehicle.pose.y - goal.y) <= 3):
                print(self.state_list[self.index + 1])
                self.mission = self.state_list[self.index + 1]
                self.index += 1
                print("CHANGING STATES", self.mission)
                self.planned_path_already = False

        elif self.mission == "INSPECT":
            state.mission.type = MissionEnum.INSPECT
            start = (state.vehicle.pose.x, state.vehicle.pose.y)
            goal = (self.inspection_route[-1][0], self.inspection_route[-1][1])

            self.route = Route(
                frame=ObjectFrameEnum.START, points=self.inspection_route
            )

            ## GOAL condition
            if (abs(state.vehicle.pose.x - goal[0]) <= 3 and abs(state.vehicle.pose.y - goal[1]) <= 3):
                print(self.state_list[self.index + 1])
                self.mission = self.state_list[self.index + 1]
                self.index += 1
                print("CHANGING STATES", self.mission)

            
            ## Camera trigger logic
            if (
                heading(
                    self.circle_center[0],
                    self.circle_center[1],
                    state.vehicle.pose.x,
                    state.vehicle.pose.y,
                )
                * state.vehicle.pose.yaw
                > 0
            ):
                state.intent = VehicleIntentEnum.CAMERA_FR
            else:
                state.intent = VehicleIntentEnum.CAMERA_RR

        elif self.mission == "FINISH":
            state.mission.type = MissionEnum.INSPECT_UPLOAD
            goal = ObjectPose(
                frame=ObjectFrameEnum.START,
                t=state.start_vehicle_pose.t,
                x=0,
                y=0,
                yaw=0,
            )
            print("Goal Position: ", goal)

            if self.planned_path_already == False:
                ## Ensure that RRT star does not run for every iteration of the component
                self.x = self.rrt_route(state, goal)
                self.planned_path_already = True
            self.route = self.x

        print("-------------------------------------------------")
        return [self.route, state.mission]
    