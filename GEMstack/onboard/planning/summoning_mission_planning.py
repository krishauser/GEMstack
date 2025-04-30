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


def check_pose_distance(goal_pose : Union[List, ObjectPose], current_pose : ObjectPose):
    if type(goal_pose) is ObjectPose:
        goal = np.array([goal_pose.x, goal_pose.y])
    elif type(goal_pose) is List:
        goal = np.array([goal_pose[0], goal_pose[1]])
    current = np.array([current_pose.x, current_pose.y])
    return np.linalg.norm(goal - current)


class SummoningMissionPlanner(Component):
    def __init__(self, mode, state_machine):
        self.mode = mode
        self.state_machine = StateMachine([eval(s) for s in state_machine])
        self.goal_location = None
        self.new_goal = False
        self.goal_pose = None
        self.start_pose = None
        self.start_time = time.time()

        self.count = 0      # for test only, simulate a delay to get the gaol location

    def state_inputs(self):
        return ['all']

    def state_outputs(self) -> List[str]:
        return ['mission_plan']

    def rate(self):
        return 1.0

    def update(self, state: AllState):
        vehicle = state.vehicle
        mission_plan = state.mission_plan
        start_vehicle_pose = state.start_vehicle_pose

        # To simulate a delay to get the goal location
        self.count += 1
        if self.count <3:
            return mission_plan

        if self.mode == 'sim':
            settings.get('simulator.scene', None)
            goal_location = scene('goal_location', [0.0, 0.0])  # for simulation test only
            goal_frame = 'start'
        elif self.mode == 'real':
            # TODO: Modify to a GET request to get goal location from the server
            # current_goal_location = self.goal_location
            # url = ""
            # response = requests.get(url)
            # if response.status_code == 200:
            #     data = response.json()
            #     goal_location = data['goal_location']
            #     goal_frame = data['goal_frame']
            goal_location = [-88.235828, 40.092741]  # for highbay test only [-88.2358085, 40.092819]
            goal_frame = 'global'
        else:
            raise ValueError("Invalid mode argument")

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

        # Close to the goal location, begin to search for parking
        elif mission_plan.planner_type == PlannerEnum.SUMMON_DRIVING:
            mission_plan.goal_pose = self.goal_pose
            dist = check_pose_distance(mission_plan.goal_pose, vehicle.pose)
            print("Distance to the goal:", dist)
            if dist < 5:
                mission_plan.planner_type = self.state_machine.next_state()

        # Finish parking, back to idle and wait for the next goal location
        elif mission_plan.planner_type == PlannerEnum.PARALLEL_PARKING:
            if state.route:
                parking_spot = state.route.points[-1]
                dist = check_pose_distance(parking_spot, vehicle.pose)
                print("Distance to the end point of the route:", dist)
                if dist < 1 and vehicle.v < 0.01:
                    mission_plan.planner_type = self.state_machine.next_state()
                    self.goal_pose = None
                    mission_plan.goal_pose = self.goal_pose

        # No state change
        else:
            mission_plan = state.mission_plan

        # TODO: POST request to update status to the server
        # data =
        # response = requests.post(url=url, data=data)

        return mission_plan