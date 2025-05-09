from typing import List, Union

from klampt.vis import scene

from ..component import Component
from ...utils import serialization
from ...state import Route, ObjectFrameEnum, AllState, VehicleState, Roadgraph, MissionObjective, MissionEnum, \
    VehicleIntent, VehicleIntentEnum, MissionPlan, PlannerEnum, ObjectPose
from ..interface.gem import GEMInterface, GNSSReading
import os
import numpy as np
import requests
import json
import time
import math
from ...utils import settings


def check_distance(goal_pose : ObjectPose, current_pose : ObjectPose):
    goal = np.array([goal_pose.x, goal_pose.y])
    current = np.array([current_pose.x, current_pose.y])
    return np.linalg.norm(goal - current)


class StateMachine:
    def __init__(self, state_list : List = None):
        self.state_list = state_list
        self.state_index = 0
        self.initial_state = self.state_list[0]
        self.current_state = self.state_list[self.state_index]

    def next_state(self):
        if self.state_index < len(self.state_list) - 1:
            self.state_index += 1
            return self.state_list[self.state_index]
        else:
            self.state_index = 0
            return self.state_list[0]


class SummoningMissionPlanner(Component):
    def __init__(self, state_machine):
        self.state_machine = StateMachine([eval(s) for s in state_machine])
        self.goal_location = None
        self.new_goal = False
        self.goal_pose = None
        self.start_pose = None
        self.start_time = time.time()

        self.count = 0      # for test only, simulate a delay to get the gaol location

        # Set False when omitting the webapp. TODO: add a flag to the config file
        self.flag_use_webapp = False

        if self.flag_use_webapp:
            # Initialize the state in the server
            url = "http://localhost:8000/api/status"
            data = {
                "status": "IDLE"
            }
            response = requests.post(url=url, json=data)
            if response.status_code == 200:
                print("Status updated successfully")
            else:
                print("Failed to update status:", response.status_code)
            url = "http://localhost:8000/api/summon"
            data = {
                "lat": 0,
                "lon": 0
            }
            response = requests.post(url=url, json=data)
            if response.status_code == 200:
                print("Initialize goal location successfully")
            else:
                print("Failed to initialize goal location:", response.status_code)

    def state_inputs(self):
        return ['all']

    def state_outputs(self) -> List[str]:
        return ['mission_plan']

    def rate(self):
        return 1.0

    def update(self, state: AllState):
        start_vehicle_pose = state.start_vehicle_pose
        vehicle = state.vehicle
        mission_plan = state.mission_plan

        # To simulate a delay to get the goal location
        self.count += 1
        if self.count <2:
            return mission_plan


        if self.flag_use_webapp:
            goal_location = None
            url = "http://localhost:8000/api/summon"
            response = requests.get(url)
            print("GET:", response)
            if response.status_code == 200:
                data = response.json()
                if data['lat'] == 0 and data['lon'] == 0:
                    print("No goal location received")
                    goal_location = None
                    goal_frame = None
                else:
                    # TODO: ADD CONVERSION FROM LAT/LON TO X/Y GLOBAL COORDINATES
                    goal_location = [data['lat'] , data['lon']]
                    goal_frame = 'global'
                    print("Goal location:", goal_location)
                    print("Goal frame:", goal_frame)

                    goal_location = [10, 0]  # for simlulation test only
                    goal_frame = 'start'
        else:
            # Test points:
            # Key points: [0, 0], [0, 30], [38.5, 8.5], [33, 14]，[27.5, 8.5]，[15, 3], [2.5, 8.5], [15, 14], [-3, 14], [-8.5, 8.5]
            # Points not in the lane:[15, -3], [15, 6], [15, 11], [15, 17]
            goal_location = [15, 6]
            goal_frame = 'start'
            # goal_location = [-88.235828, 40.092741]  # for highbay test only [-88.2358085, 40.092819]
            # goal_frame = 'global'

        if self.goal_location == goal_location:
            self.new_goal = False
        else:
            self.new_goal = True
            self.goal_location = goal_location

        if self.new_goal:
            if goal_frame == 'global':
                self.goal_pose = ObjectPose(t=time.time(), x=goal_location[0], y=goal_location[1],
                                            frame=ObjectFrameEnum.GLOBAL)
            elif goal_frame == 'cartesian':
                self.goal_pose = ObjectPose(t=time.time(), x=goal_location[0], y=goal_location[1],
                                        frame=ObjectFrameEnum.ABSOLUTE_CARTESIAN)
            elif goal_frame == 'start':
                self.goal_pose = ObjectPose(t=time.time() - self.start_time, x=goal_location[0], y=goal_location[1],
                                            frame=ObjectFrameEnum.START)
            else:
                raise ValueError("Invalid frame argument")

        if self.goal_pose:
            self.goal_pose = self.goal_pose.to_frame(ObjectFrameEnum.START, start_pose_abs=start_vehicle_pose)

        # Initiate state
        if mission_plan is None:
            mission_plan = MissionPlan()
            mission_plan.planner_type = self.state_machine.initial_state

        # Receive goal location from the server and start driving
        elif mission_plan.planner_type == PlannerEnum.IDLE:
            if self.new_goal:
                mission_plan.goal_pose = self.goal_pose
                mission_plan.planner_type = self.state_machine.next_state()
                print("============== Next state:", mission_plan.planner_type)

        # Reach the end of the route, begin to search for parking
        elif mission_plan.planner_type == PlannerEnum.SUMMON_DRIVING:
            mission_plan.goal_pose = self.goal_pose
            if state.route:
                _, closest_index = state.route.closest_point([vehicle.pose.x, vehicle.pose.y], edges=False)
                if closest_index == len(state.route.points) - 1:
                    mission_plan.planner_type = self.state_machine.next_state()
                    print("============== Next state:", mission_plan.planner_type)

        # Finish parking, back to idle and wait for the next goal location
        elif mission_plan.planner_type == PlannerEnum.PARALLEL_PARKING:
            if state.route:
                _, closest_index = state.route.closest_point([vehicle.pose.x, vehicle.pose.y], edges=False)
                if closest_index == len(state.route.points) - 1:
                    mission_plan.planner_type = self.state_machine.next_state()
                    self.goal_pose = None
                    mission_plan.goal_pose = self.goal_pose
                    print("============== Next state:", mission_plan.planner_type)

        # No state change
        else:
            mission_plan = state.mission_plan


        if self.flag_use_webapp:
            url = "http://localhost:8000/api/status"
            data = {
                "status": mission_plan.planner_type.name
            }
            print("POST:", data)
            response = requests.post(url=url, json=data)
            if response.status_code == 200:
                print("Status updated successfully")
            else:
                print("Failed to update status:", response.status_code)


        return mission_plan