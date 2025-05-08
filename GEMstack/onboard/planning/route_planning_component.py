import os
from typing import Dict, List
import numpy as np
from GEMstack.onboard.component import Component
from GEMstack.state.agent import AgentState
from GEMstack.state.all import AllState
from GEMstack.state.physical_object import ObjectFrameEnum
from GEMstack.state.route import PlannerEnum, Route, Path
from GEMstack.state.trajectory import Trajectory
from .rrt_star import RRTStar
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
        self.planning_time = 0.0
        self.last_planning_time = 0.0
    def state_inputs(self):
        return ["all"]
    def state_outputs(self) -> List[str]:
        return ['route']
    def rate(self):
        return 10.0 # very high for our computation ability
    def update(self, state: AllState):
        # print("Route Planner's mission:", state.mission_plan.planner_type.value)
        # print("type of mission plan:", type(PlannerEnum.RRT_STAR))
        # print("Route Planner's mission:", state.mission_plan.planner_type.value == PlannerEnum.RRT_STAR.value)
        # print("Route Planner's mission:", state.mission_plan.planner_type.value == PlannerEnum.PARKING.value)
        # print("Mission plan:", state.mission_plan)
        # print("Vehicle x:", state.vehicle.pose.x)
        # print("Vehicle y:", state.vehicle.pose.y)
        # print("Vehicle yaw:", state.vehicle.pose.yaw)
        if state.mission_plan.planner_type.name == "PARKING" and not self.compute_parking_route:
            print("I am in PARKING mode")
            # Not sure where I should construct this object
            # if isinstance(self.planner, StraightLineMotion): # we are transitioning from scanning to parking
                # Check if the car is still in motion from scanning behavior
                # if it is, we need to stop it before parking
                # self.planner.brake() # get car totally stopped before parking
            # state.vehicle.pose.yaw = 0 # needed this to avoid a weird error in the parking planner
            # self.planner = ParkingPlanner()
            # self.computed_parking_route = True
            # # Return a route after doing some processing based on mission plan REMOVE ONCE OTHER PLANNERS ARE IMPLEMENTED
            # traj = self.planner.update(state)
            # self.planner.visualize_trajectory(traj)
            # self.planner.visualize_trajectory_animated(traj)
            desired_points = [(state.vehicle.pose.x, state.vehicle.pose.y),
                              (state.vehicle.pose.x, state.vehicle.pose.y)]
            desired_path = Path(state.vehicle.pose.frame, desired_points)
            self.compute_parking_route = True
            return desired_path
        elif state.mission_plan.planner_type.name == "PARKING" and self.compute_parking_route and not self.done_computing:
            state.vehicle.pose.yaw = 0 # needed this to avoid a weird error in the parking planner
            if not self.already_computed:
                print("\n=== Starting Parking Route Planning ===")
                start_time = time.time()
                self.planner = ParkingPlanner()
                self.done_computing = True
                self.route = self.planner.update(state)
                # Calculate planning time
                self.planning_time = time.time() - start_time
                print(f"\n=== Planning Metrics ===")
                print(f"Total Planning Time: {self.planning_time:.3f} seconds")
                if self.last_planning_time > 0:
                    print(f"Previous Planning Time: {self.last_planning_time:.3f} seconds")
                    print(f"Time Difference: {self.planning_time - self.last_planning_time:.3f} seconds")
                self.last_planning_time = self.planning_time
                print("=== Planning Complete ===\n")
                self.planner.visualize_trajectory(self.route)
                self.already_computed = True
            return self.route
        elif state.mission_plan.planner_type.name == "RRT_STAR":
            print("I am in RRT mode")
        elif state.mission_plan.planner_type.name == "SCANNING":
            # Run brute force straight line motion
            # if (self.planner is None) or (not isinstance(self.planner, StraightLineMotion)):
                # self.planner = StraightLineMotion()
            # print("Mission plan:", state.mission_plan)
            # print("Vehicle x:", state.vehicle.pose.x)
            # print("Vehicle y:", state.vehicle.pose.y)
            # print("Vehicle yaw:", state.vehicle.pose.yaw)
            print("I am in SCANNING mode")
            desired_points = [(state.vehicle.pose.x, state.vehicle.pose.y),
                              (state.vehicle.pose.x + 1, state.vehicle.pose.y)]
            desired_path = Path(state.vehicle.pose.frame, desired_points)
            # @TODO these are constants we need to get from settings
            return desired_path
            # self.planner.update_speed()
            # We want to just go straight
            # Use longitudinal planner to go in a straight line
            # Get current
        return None