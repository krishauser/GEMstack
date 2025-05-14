import os
from typing import Dict, List

import numpy as np
from GEMstack.onboard.component import Component
from GEMstack.state.agent import AgentState
from GEMstack.state.all import AllState
from GEMstack.state.physical_object import ObjectFrameEnum, ObjectPose
from ...state import AgentState, AgentEnum, EntityRelation, EntityRelationEnum, ObjectFrameEnum, VehicleState, MissionPlan, Obstacle, ObstacleMaterialEnum
from GEMstack.state.route import PlannerEnum, Route
from .occupancy_grid2 import OccupancyGrid2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from .planner import optimized_kinodynamic_rrt_planning
from .collision import build_collision_lookup
import math

import cv2

class RoutePlanningComponentReal(Component):
    """Reads a route from disk and returns it as the desired route."""
    def __init__(self):
        print("Route Planning Component init")
        self.planner = None
        self.route = None
        self.bridge = CvBridge()
        self.occupancy_grid = OccupancyGrid2()
        self.img_pub = rospy.Publisher("/occupancy_grid", Image, queue_size=1)
        self.previous_obstacles = None

    def state_inputs(self):
        return ["vehicle", "agents", "mission_plan", "obstacles"]

    def state_outputs(self) -> List[str]:
        return ['route']

    def rate(self):
        return 10.0
    

    def update(self, vehicle: VehicleState, agents: Dict[str, AgentState], mission_plan: MissionPlan, obstacles: Obstacle) -> Route:
        print("&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&")
        print("agent", agents.items())
        print("vehicle", vehicle.pose.x, vehicle.pose.y, vehicle.pose.yaw)
        print("start vehicle", mission_plan.start_vehicle_pose)
        print("MOJI MOJ", vehicle.pose.to_frame(ObjectFrameEnum.GLOBAL, start_pose_abs=mission_plan.start_vehicle_pose))
        print("&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&")
        if self.previous_obstacles is None:
            self.previous_obstacles = len(obstacles)

        # Convert vehicle pose to global frame
        vehicle_global_pos = vehicle.pose.to_frame(ObjectFrameEnum.GLOBAL, start_pose_abs=mission_plan.start_vehicle_pose)

        # Convert vehicle pose to image coordinates
        self.occupancy_grid.gnss_to_image(vehicle_global_pos.x, vehicle_global_pos.y)

        obstacles_global_poses = []
        print("obstacles - length", len(obstacles))
        for n, o in obstacles.items():
            print("==========================\nAgent:", n, o.pose, o.velocity)
            print('==============', o.pose.frame==ObjectFrameEnum.START)
            print('==============', o.type)
            if o.type == ObstacleMaterialEnum.TRAFFIC_CONE:
                print("CONE detected")
                obstacle_global_pose = o.pose.to_frame(ObjectFrameEnum.GLOBAL, start_pose_abs=mission_plan.start_vehicle_pose)
                print("obstacle_global_pose", obstacle_global_pose)
                obstacles_global_poses.append((obstacle_global_pose.y, obstacle_global_pose.x))
            

        rects = self.occupancy_grid.draw_obstacle_cones(obstacles_global_poses)

        # rects = self.occupancy_grid.compute_cone_rectangles(
        #     obstacles_global_poses,
        # )

        print("RECTANGLES", rects)


        # for n, a in agents.items():
        #     print("==========================\nAgent:", n, a.pose, a.velocity)
        #     print('==============', a.pose.frame==ObjectFrameEnum.START)
        #     print('==============', a.type==AgentEnum.PEDESTRIAN)
        #     if a.type == AgentEnum.PEDESTRIAN:
        #         print("Pedestrian detected")
        
        # for n, o in obstacles.items():
        #     print("==========================\nAgent:", n, o.pose, o.velocity)
        #     print('==============', o.pose.frame==ObjectFrameEnum.START)
        #     print('==============', o.type)
        #     if o.type == ObstacleMaterialEnum.TRAFFIC_CONE:
        #         print("CONE detected")
            
        print("Route Planner's mission:", mission_plan.planner_type.value)
        print("type of mission plan:", type(PlannerEnum.RRT_STAR))
        print("Route Planner's mission:", mission_plan.planner_type.value == PlannerEnum.RRT_STAR.value)
        print("Route Planner's mission:", mission_plan.planner_type.value == PlannerEnum.PARKING.value)
        print("Mission plan:", mission_plan)
        print("Vehicle x:", vehicle.pose.x)
        print("Vehicle y:", vehicle.pose.y)
        print("Vehicle yaw:", vehicle.pose.yaw)

        if mission_plan.planner_type.value == PlannerEnum.PARKING.value:
            print("I am in PARKING mode")
            base_path = os.path.dirname(__file__)
            file_path = os.path.join(base_path, "../../knowledge/routes/forward_15m_extra.csv")
            waypoints = np.loadtxt(file_path, delimiter=',', dtype=float)
            if waypoints.shape[1] == 3:
                    waypoints = waypoints[:,:2]
            self.route = Route(frame=ObjectFrameEnum.START,points=waypoints.tolist())
        elif mission_plan.planner_type.value == PlannerEnum.RRT_STAR.value:
            print("I am in RRT mode")

            ## Step 1: Convert vehicle pose to global frame
            vehicle_global_pose = vehicle.pose.to_frame(
                ObjectFrameEnum.GLOBAL, start_pose_abs=mission_plan.start_vehicle_pose
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
            print("Start image coordinates", start_x, start_y, "yaw", start_yaw)

            # ## Step 3. Convert goal to global frame
            # goal_start_pose = ObjectPose(
            #     frame=ObjectFrameEnum.START,
            #     t=mission_plan.start_vehicle_pose.t,
            #     x=vehicle.pose.x + 15,
            #     y=vehicle.pose.y,
            #     yaw=0,
            # )

            goal_global_pose = mission_plan.goal_vehicle_pose.to_frame(
                ObjectFrameEnum.GLOBAL, start_pose_abs=mission_plan.start_vehicle_pose
            )
            
            goal_x, goal_y = self.occupancy_grid.gnss_to_image_coords(
                goal_global_pose.x, goal_global_pose.y
            )  

            goal_yaw = start_yaw #goal_global_pose.yaw
            print("Goal image coordinates", goal_x, goal_y, "yaw", goal_yaw)

            script_dir = os.path.dirname(os.path.abspath(__file__))
            map_path = os.path.join(script_dir, "highbay_image.pgm")

            print("map_path", map_path)

            map_img = cv2.imread(map_path, cv2.IMREAD_UNCHANGED)

            for i, (rect_pt1_x, rect_pt1_y, rect_pt2_x, rect_pt2_y) in enumerate(rects):
                if rect_pt1_x < rect_pt2_x and rect_pt1_y < rect_pt2_y:
                    # It's a rectangle with areaf
                    cv2.rectangle(map_img, (rect_pt1_x, rect_pt1_y), (rect_pt2_x, rect_pt2_y), (255, 255, 255), 6)  # White
                    print(f"[DEBUG-ME] Drew WHITE rectangle for cluster {i}")
                elif rect_pt1_x == rect_pt2_x and rect_pt1_y < rect_pt2_y:
                    # It's a vertical line
                    cv2.line(map_img, (rect_pt1_x, rect_pt1_y), (rect_pt2_x, rect_pt2_y), (255, 255, 255), 6) # White line
                    print(f"[DEBUG-ME] Drew WHITE vertical line for cluster {i}")
                elif rect_pt1_y == rect_pt2_y and rect_pt1_x < rect_pt2_x:
                    # It's a horizontal line
                    cv2.line(map_img, (rect_pt1_x, rect_pt1_y), (rect_pt2_x, rect_pt2_y), (255, 255, 255), 6) # White line
                    print(f"[DEBUG-ME] Drew WHITE horizontal line for cluster {i}")
                else:
                    print(f"[DEBUG-ME] Cluster {i} BBox has zero/negative area and is not a simple line. rect_pt1_x={rect_pt1_x}, rect_pt2_x={rect_pt2_x}, rect_pt1_y={rect_pt1_y}, rect_pt2_y={rect_pt2_y}")

            cv2.imwrite("highbay_image_with_cones.pgm", map_img)
            occupancy_grid = (map_img > 0).astype(
                np.uint8
            ) 
            cv2.imwrite("occupancy_grid_after_>0.pgm", occupancy_grid * 255)
            self.t_last = None
            self.bounds = (0, occupancy_grid.shape[1])
            # y_bounds = (0,occupancy_grid.shape[0])
            # start = (start_x, start_y) # add start_yaw. @Sai I have not integrated yaw into kinodynamic yet. but plls add
            # goal = (goal_x, goal_y) # add goal_yaw. @Sai I have not integrated yaw into kinodynamic yet. but plls add
            # step_size = 1.0
            # max_iter = 2000

            # script_dir = os.path.dirname(os.path.abspath(__file__))
            # map_path = os.path.join(script_dir, "highbay_image.pgm")
            
            

            # occupancy_grid = load_pgm_to_occupancy_grid(map_path)
            start_w = [start_x, start_y, start_yaw]
            goal_w = [goal_x, goal_y, goal_yaw]
            # occupancy_grid[start_x-5:start_x +5][start_y-5] = 1

            if self.route == None or len(obstacles) != self.previous_obstacles:
                    
                self.previous_obstacles = len(obstacles)
                path = optimized_kinodynamic_rrt_planning(start_w, goal_w, occupancy_grid)

                # print("RRT mode", path)
                # rrt_resp = self.planner.plan()
                # self.visualize_route_pixels(rrt_resp, start, goal)
                waypoints = []
                for i in range(len(path)):
                    x, y, theta = path[i]
                waypoints = []
                for i in range(0, len(path), 10):
                    x, y, theta = path[i]
                    # Convert to car coordinates
                    waypoint_lat, waypoint_lon = self.occupancy_grid.image_to_gnss(
                        x, y
                    )  # Converts pixel to global frame. Brijesh check again what x corresponds to. Is x lat or is x lon? Change accordingly. Same as above comments
                    # Convert global to start frame
                    waypoint_global_pose = ObjectPose(
                        frame=ObjectFrameEnum.GLOBAL,
                        t=mission_plan.start_vehicle_pose.t,
                        x=waypoint_lon,
                        y=waypoint_lat,
                        yaw=theta,
                    )
                    waypoint_start_pose = waypoint_global_pose.to_frame(
                        ObjectFrameEnum.START, start_pose_abs=mission_plan.start_vehicle_pose
                    )  # not handling yaw cuz we don't know how to
                    waypoints.append((waypoint_start_pose.x, waypoint_start_pose.y))
                    # waypoint_global_pose = ObjectPose(
                    #     frame=ObjectFrameEnum.GLOBAL,
                    #     t=mission_plan.start_vehicle_pose.t,
                    #     x=waypoint_lon,
                    #     y=waypoint_lat,
                    #     yaw=theta,
                    # )
                    # waypoint_start_pose = waypoint_global_pose.to_frame(
                    #     ObjectFrameEnum.START, start_pose_abs=mission_plan.start_vehicle_pose
                    # )  # not handling yaw cuz we don't know how to
                    # waypoints.append((waypoint_start_pose.x, waypoint_start_pose.y))
                
                self.route = Route(
                    frame=ObjectFrameEnum.START, points=waypoints)

                print("Route points in start frame: ", waypoints) # Comment this out once you are done debugging
            return self.route


        else:
            print("Unknown mode")
        
        return self.route
















