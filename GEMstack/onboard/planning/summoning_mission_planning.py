from typing import List
from ..component import Component
from ...utils import serialization
from ...state import Route, ObjectFrameEnum, AllState, VehicleState, Roadgraph, MissionObjective, MissionEnum, \
    VehicleIntent, VehicleIntentEnum, MissionPlan, PlannerEnum, ObjectPose
import os
import numpy as np
import requests
import json
import time
import math


def check_distance(goal, current_pose):
    goal = np.array(goal)[:2]
    current_pose = np.array([current_pose.x, current_pose.y])
    return np.linalg.norm(goal - current_pose)


class SummoningMissionPlanner(Component):
    def __init__(self, distance_to_goal_to_start_parking, distance_error_of_idle_from_parking, state_machine):
        print("Initializing SummoningMissionPlanner:", distance_to_goal_to_start_parking, distance_error_of_idle_from_parking, state_machine)
        self.distance_to_goal_to_start_parking = distance_to_goal_to_start_parking
        self.distance_error_of_idle_from_parking = distance_error_of_idle_from_parking
        self.state_list = [eval(s) for s in state_machine]

        self.count = 0      # for simulate a delay to get the gaol location

    def state_inputs(self):
        return ['all']

    def state_outputs(self) -> List[str]:
        return ['mission_plan']

    def rate(self):
        return 1.0

    def update(self, state: AllState):
        vehicle = state.vehicle
        mission_plan = state.mission_plan
        if mission_plan is None:
            mission_plan = MissionPlan()
            mission_plan.planner_type = PlannerEnum.IDLE

        # for simulate a delay to get the gaol location
        self.count += 1
        if self.count <3:
            return mission_plan

        # TODO: Modify to a GET request to get goal location from the server
        # url = ""
        # response = requests.get(url)
        # if response.status_code == 200:
        #     data = response.json()
        #     goal_location = data['goal_location']
        #     goal_location = ObjectPose(t=time.time(), x=goal_location[0], y=goal_location[1], frame=ObjectFrameEnum.GLOBAL)

        goal_location = [10, 11]  # for test only
        self.goal_pose = ObjectPose(t=time.time(), x=goal_location[0], y=goal_location[1],
                                    frame=ObjectFrameEnum.START)

        if mission_plan.planner_type == PlannerEnum.IDLE and self.goal_pose is not None:
            mission_plan.goal_pose = self.goal_pose
            mission_plan.planner_type = PlannerEnum.SUMMON_DRIVING

        elif mission_plan.planner_type == PlannerEnum.SUMMON_DRIVING:
            dist = check_distance([self.goal_pose.x, self.goal_pose.y], vehicle.pose)
            if dist < self.distance_error_of_idle_from_parking:
                mission_plan.planner_type = PlannerEnum.PARALLEL_PARKING

        elif mission_plan.planner_type == PlannerEnum.PARALLEL_PARKING:
            dist = check_distance(state.route.points[-1], vehicle.pose)
            if dist < self.distance_error_of_idle_from_parking:
                mission_plan.planner_type = PlannerEnum.IDLE

        else:
            mission_plan = state.mission_plan

        # TODO: POST request to update status to the server
        # data =
        # response = requests.post(url=, data=)

        return mission_plan