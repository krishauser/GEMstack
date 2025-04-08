import os
from typing import List

import numpy as np
from GEMstack.onboard.component import Component
from GEMstack.state.all import AllState
from GEMstack.state.physical_object import ObjectFrameEnum
from GEMstack.state.route import PlannerEnum, Route



class RoutePlanningComponent(Component):
    """Reads a route from disk and returns it as the desired route."""
    def __init__(self):
        print("Route Planning Component init")
        
    def state_inputs(self):
        return ["all"]

    def state_outputs(self) -> List[str]:
        return ['route']

    def rate(self):
        return 10.0

    def update(self, state: AllState):
        print("Route Planner's mission:", state.mission_plan)
        print("Vehicle x:", state.vehicle.pose.x)
        print("Vehicle y:", state.vehicle.pose.y)
        print("Vehicle yaw:", state.vehicle.pose.yaw)
        if state.mission_plan.planner_type == PlannerEnum.PARKING:
            print("Parking mode")
        elif state.mission_plan.planner_type == PlannerEnum.RRT_STAR:
            print("RRT mode")
        else:
            print("Unknown mode")
        
        ## Return a route after doing some processing based on mission plan REMOVE ONCE OTHER PLANNERS ARE IMPLEMENTED
        base_path = os.path.dirname(__file__)
        file_path = os.path.join(base_path, "../../knowledge/routes/forward_15m.csv")
    
        waypoints = np.loadtxt(file_path, delimiter=',', dtype=float)
        if waypoints.shape[1] == 3:
                waypoints = waypoints[:,:2]
        print("waypoints", waypoints)
        route = Route(frame=ObjectFrameEnum.START,points=waypoints.tolist())
        print("route", route)
        return route