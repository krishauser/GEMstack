import os
from typing import Dict, List

import numpy as np
from GEMstack.onboard.component import Component
from GEMstack.state.agent import AgentState
from GEMstack.state.all import AllState
from GEMstack.state.physical_object import ObjectFrameEnum
from GEMstack.state.route import PlannerEnum, Route



class RoutePlanningComponent(Component):
    """Reads a route from disk and returns it as the desired route."""
    def __init__(self):
        print("Route Planning Component init")
        self.planner = None
        self.route = None
        
    def state_inputs(self):
        return ["all"]

    def state_outputs(self) -> List[str]:
        return ['route']

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
        if state.mission_plan.planner_type.name == "PARKING":
            print("I am in PARKING mode")
            # Return a route after doing some processing based on mission plan REMOVE ONCE OTHER PLANNERS ARE IMPLEMENTED
           
        elif state.mission_plan.planner_type.name == "SCANNING":
            print("I am in SCANNING mode")
        else:
            print("Unknown mode")
        
        return self.route