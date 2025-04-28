import os
from typing import Dict, List

import numpy as np
from GEMstack.onboard.component import Component
from GEMstack.state.agent import AgentState
from GEMstack.state.all import AllState
from GEMstack.state.physical_object import ObjectFrameEnum
from GEMstack.state.route import PlannerEnum, Route
from .rrt_star import RRTStar
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from .occupancy_grid2 import OccupancyGrid2

import cv2

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
        for n, a in agents.items():
            print("==========================\nAgent:", n, a.pose, a.velocity)
            print('==============', a.pose.frame==ObjectFrameEnum.START)
            print('==============', a.type==AgentEnum.PEDESTRIAN)
            if a.type == AgentEnum.PEDESTRIAN:
                print("Pedestrian detected")
                # Do something with the pedestrian information
            
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
        self.occupancy_grid.gnss_to_image(vehicle_global_pose.x, vehicle_global_pose.y) # Brijesh check if x corresponds to lon and y corresponds to lat. If not SWAP
        
        ## Step 2: Get start image coordinates aka position of vehicle in image
        start_x, start_y = self.occupancy_grid.gnss_to_image_coords(vehicle_global_pose.x, vehicle_global_pose.y) # Brijesh check if x corresponds to lon and y corresponds to lat. If not SWAP
        start_yaw = vehicle_global_pose.yaw
        print("Start image coordinates", start_x, start_y, "yaw", start_yaw)
        
        ## Step 3. Convert goal to global frame
        goal_global_pose = mission_plan.goal_vehicle_pose.to_frame(ObjectFrameEnum.GLOBAL, start_pose_abs = mission_plan.start_vehicle_pose)
        goal_x, goal_y = self.occupancy_grid.gnss_to_image_coords(goal_global_pose.x, goal_global_pose.y) # Brijesh check if x corresponds to lon and y corresponds to lat. If not SWAP
        goal_yaw = goal_global_pose.yaw
        print("Goal image coordinates", goal_x, goal_y, "yaw", goal_yaw)

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