from typing import List
from ..component import Component
from ...utils import serialization
from ...state import AllState, VehicleIntent, VehicleIntentEnum, MissionObjective, MissionEnum, Trajectory, Path
import numpy as np


def parking_plan(roadgraph):
    intent = VehicleIntentEnum.PARKING
    # TODO: logic of determine which type of parking to be executed

    return intent


def leave_parking_plan(roadgraph):
    intent = VehicleIntentEnum.LEAVING_PARKING
    # TODO: logic of determine which type of parking to be executed

    return intent


class ParkingTypePlanner(Component):
    def state_inputs(self):
        return ['all']

    def state_outputs(self) -> List[str]:
        return ['intent']

    def rate(self):
        return 1.0

    def update(self, state: AllState):
        mission = state.mission
        intent = state.intent
        roadgraph = state.roadgraph

        print('Input:')
        print(intent)
        print(mission)

        if mission.type == MissionEnum.PARK:
            intent = parking_plan(roadgraph)

        elif mission.type == MissionEnum.UNPARK:
            intent = leave_parking_plan(roadgraph)

        print('Output:')
        print(intent)

        return intent