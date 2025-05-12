import os
from typing import Dict, List

import numpy as np
from GEMstack.onboard.component import Component
from GEMstack.state.agent import AgentState
from GEMstack.state.all import AllState
from GEMstack.state.physical_object import ObjectFrameEnum
from GEMstack.state.route import PlannerEnum, Route, Path
from GEMstack.state.trajectory import Trajectory
# from .rrt_star import RRTStar
from .parking_route_planner import ParkingPlanner
from .parking_scanning import StraightLineMotion
from .longitudinal_planning import longitudinal_plan
import time


class RoutePlanningComponent(Component):
    """Reads a route from disk and returns it as the desired route."""
    def __init__(self):
        print("Route Planning Component init")
        self.route = None
        self.planner = None
        self.compute_parking_route = False
        self.done_computing = False
        self.already_computed = False
        
    def state_inputs(self):
        return ["all"]

    def state_outputs(self) -> List[str]:
        return ['route']

    def rate(self):
        return 10.0 # very high for our computation ability

    def update(self, state: AllState):

        if state.mission_plan.planner_type.name == "PARKING" and not self.compute_parking_route:
            print("I am in PARKING mode")

            desired_points = [(state.vehicle.pose.x, state.vehicle.pose.y),
                              (state.vehicle.pose.x, state.vehicle.pose.y)]
            desired_path = Path(state.vehicle.pose.frame, desired_points)
            
            desired_path = desired_path.to_frame(state.vehicle.pose.frame, current_pose=state.vehicle.pose, start_pose_abs=state.start_vehicle_pose)

            self.compute_parking_route = True

            return desired_path
        elif state.mission_plan.planner_type.name == "PARKING" and self.compute_parking_route and not self.done_computing:
            state.vehicle.pose.yaw = 0 # needed this to avoid a weird error in the parking planner
            
            if not self.already_computed:
                self.planner = ParkingPlanner()
                self.done_computing = True
                time_before = time.time()
                self.route = self.planner.update(state)
                time_after = time.time()
                print("COMPUTE TIME:", time_after - time_before)
                self.route = self.route.to_frame(ObjectFrameEnum.START, current_pose=state.vehicle.pose, start_pose_abs=state.start_vehicle_pose)
                self.planner.visualize_trajectory(self.route)
                self.already_computed = True

            return self.route
        elif state.mission_plan.planner_type.name == "RRT_STAR":
            print("I am in RRT mode")
        elif state.mission_plan.planner_type.name == "SCANNING":

            print("I am in SCANNING mode")
            desired_points = [(state.vehicle.pose.x, state.vehicle.pose.y),
                              (state.vehicle.pose.x + 1, state.vehicle.pose.y)]
            desired_path = Path(ObjectFrameEnum.START, desired_points)
            
            desired_path = desired_path.to_frame(state.vehicle.pose.frame, current_pose=state.vehicle.pose, start_pose_abs=state.start_vehicle_pose)
            return desired_path
            

        
        return None