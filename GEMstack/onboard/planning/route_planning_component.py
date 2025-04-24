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
        
    def state_inputs(self):
        return ["all"]

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




    def update(self, state: AllState):
        print("Route Planner's mission:", state.mission_plan.planner_type.value)
        print("type of mission plan:", type(PlannerEnum.RRT_STAR))
        print("Route Planner's mission:", state.mission_plan.planner_type.value == PlannerEnum.RRT_STAR.value)
        print("Route Planner's mission:", state.mission_plan.planner_type.value == PlannerEnum.PARKING.value)
        print("Mission plan:", state.mission_plan)
        print("Vehicle x:", state.vehicle.pose.x)
        print("Vehicle y:", state.vehicle.pose.y)
        print("Vehicle yaw:", state.vehicle.pose.yaw)
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
            map_path  = os.path.join(script_dir, "out.pgm") 

            print("map_path", map_path)

            map_img = cv2.imread(map_path, cv2.IMREAD_UNCHANGED)
            occupancy_grid = (map_img > 0).astype(np.uint8) # (590, 1656)
            x_bounds = (0,occupancy_grid.shape[1])
            y_bounds = (0,occupancy_grid.shape[0])
            start_u_x, start_u_y = self._car_to_pixel(state.vehicle.pose.x, state.vehicle.pose.y, occupancy_grid.shape[1], occupancy_grid.shape[0])
            start = (start_u_x, start_u_y)
            goal_u_x, goal_u_y = self._car_to_pixel(state.mission_plan.goal_x, state.mission_plan.goal_y, occupancy_grid.shape[1], occupancy_grid.shape[0])
            goal = (goal_u_x, goal_u_y) # When we implement kinodynamic, we need to include target_yaw also
            step_size = 1.0
            max_iter = 2000

            self.planner = RRTStar(start, goal, x_bounds, y_bounds, max_iter=max_iter, step_size=step_size, vehicle_width=1, occupancy_grid=occupancy_grid)
            print("RRT mode")
            rrt_resp = self.planner.plan()
            self.visualize_route_pixels(rrt_resp, start, goal)
            for i in range(len(rrt_resp)):
                x, y = rrt_resp[i]
                # Convert to car coordinates
                car_x, car_y = self._pixel_to_car(x, y, occupancy_grid.shape[1], occupancy_grid.shape[0])
                rrt_resp[i] = (car_x, car_y)
            
            self.route = Route(frame=ObjectFrameEnum.START, points=rrt_resp)
            # print("Route planning complete")

        else:
            print("Unknown mode")
        
        return self.route