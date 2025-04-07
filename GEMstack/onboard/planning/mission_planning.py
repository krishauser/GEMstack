from typing import List
from ..component import Component
from ...utils import serialization
from ...state import Route, ObjectFrameEnum, AllState, VehicleState, Roadgraph, MissionObjective, MissionEnum, VehicleIntent, VehicleIntentEnum, TaskPhase, TaskEnum
import os
import numpy as np
import requests


class SummoningMissionPlanner(Component):
    def state_inputs(self):
        return ['all']

    def state_outputs(self) -> List[str]:
        return ['mission', 'task']

    def rate(self):
        return 0.5

    def update(self, state: AllState):
        task = state.task
        mission = state.mission

        print('Input:')
        print(task)
        print(mission)

        # TODO: Modify to a GET request to get task phase from the server
        if task.phase == TaskEnum.IDLE:
            task.phase = TaskEnum.START

        elif mission.type == MissionEnum.IDLE and task.phase == TaskEnum.START:
            mission.type = MissionEnum.PLAN

        elif mission.type == MissionEnum.DRIVE:
            task.phase = TaskEnum.NAVIGATING

        elif mission.type == MissionEnum.IDLE and task.phase == TaskEnum.NAVIGATING:
            task.phase = TaskEnum.ARRIVED

        # TODO: POST update status to the server

        print('Output:')
        print(task)
        print(mission)

        return mission, task