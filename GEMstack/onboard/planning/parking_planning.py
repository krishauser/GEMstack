from typing import List
from ..component import Component
from ...utils import serialization
from ...state import AllState, VehicleIntent, VehicleIntentEnum, MissionObjective, MissionEnum, Trajectory, Path
import numpy as np

from .longitudinal_planning import longitudinal_plan


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

        print('Input states:')
        print(intent)
        print(mission)

        if mission.type == MissionEnum.PARK:
            intent.intent = parking_plan(roadgraph)

        elif mission.type == MissionEnum.UNPARK:
            intent.intent = leave_parking_plan(roadgraph)

        print('Output states:')
        print(intent)

        return intent


### FOR TEST ###
def generate_turn(vehicle, turn_angle=45, turn_radius=3.0, turn_direction="right",
                             acceleration=5.0, deceleration=2.0, max_speed=2.0, num_points=10):
    theta = np.deg2rad(turn_angle)
    x, y, yaw = vehicle.pose.x, vehicle.pose.y, vehicle.pose.yaw
    current_speed = vehicle.v
    R = turn_radius

    if turn_direction == "right":
        cx = x + R * np.cos(yaw - np.pi / 2)
        cy = y + R * np.sin(yaw - np.pi / 2)
        start_angle = yaw - np.pi / 2
        angles = np.linspace(start_angle, start_angle - theta, num_points)
    else:  # left
        cx = x + R * np.cos(yaw + np.pi / 2)
        cy = y + R * np.sin(yaw + np.pi / 2)
        start_angle = yaw + np.pi / 2
        angles = np.linspace(start_angle, start_angle + theta, num_points)

    waypoints = []
    for angle in angles:
        xi = cx + R * np.cos(angle)
        yi = cy + R * np.sin(angle)
        waypoints.append([xi, yi])

    path = Path(frame=vehicle.pose.frame, points=waypoints)
    # traj = longitudinal_plan(path, acceleration, deceleration, max_speed, current_speed)

    return path


class ParkingRoutePlanner(Component):
    def state_inputs(self):
        return ['all']

    def state_outputs(self) -> List[str]:
        return ['route', 'intent']

    def rate(self):
        return 10.0

    def update(self, state: AllState):
        intent = state.intent
        vehicle = state.vehicle
        route = state.route
        roadgraph = state.roadgraph

        print('Input states:')
        print(intent)

        # TODO: implement parking planning 
        if intent.intent == VehicleIntentEnum.HEAD_IN_PARKING:
            pass
        elif intent.intent == VehicleIntentEnum.BACK_IN_PARKING:
            pass
        elif intent.intent == VehicleIntentEnum.PARALLEL_PARKING:
            pass
        ### FOR TEST ###
        # TODO: Modify to stop aside the target location
        elif intent.intent == VehicleIntentEnum.PARKING:    # for test
            route = generate_turn(vehicle, turn_angle=45, turn_radius=3.0, turn_direction="right", num_points=10)

        if intent.intent in [VehicleIntentEnum.PARKING,
                             VehicleIntentEnum.HEAD_IN_PARKING, 
                             VehicleIntentEnum.BACK_IN_PARKING,
                             VehicleIntentEnum.PARALLEL_PARKING
                             ]:
            intent.intent = VehicleIntentEnum.IDLE

        print('Output states:')
        print(intent)

        return route, intent


class LeaveParkingRoutePlanner(Component):
    def state_inputs(self):
        return ['all']

    def state_outputs(self) -> List[str]:
        return ['route', 'intent']

    def rate(self):
        return 10.0

    def update(self, state: AllState):
        intent = state.intent
        vehicle = state.vehicle
        route = state.route
        roadgraph = state.roadgraph

        print('Input states:')
        print(intent)

        # TODO: implement leave parking planning
        if intent.intent == VehicleIntentEnum.LEAVING_HEAD_IN_PARKING:
            pass
        elif intent.intent == VehicleIntentEnum.LEAVING_BACK_IN_PARKING:
            pass
        elif intent.intent == VehicleIntentEnum.LEAVING_PARALLEL_PARKING:
            pass
        ### FOR TEST ###
        # TODO: Modify to leave from stop aside the target location
        elif intent.intent == VehicleIntentEnum.LEAVING_PARKING:    # for test
            pass
            # route = generate_turn(vehicle, turn_angle=15, turn_radius=3.0, turn_direction="left", num_points=10)

        if intent.intent in [VehicleIntentEnum.LEAVING_PARKING,
                             VehicleIntentEnum.LEAVING_HEAD_IN_PARKING, 
                             VehicleIntentEnum.LEAVING_BACK_IN_PARKING,
                             VehicleIntentEnum.LEAVING_PARALLEL_PARKING
                             ]:
            intent.intent = VehicleIntentEnum.DRIVING

        print('Output states:')
        print(intent)

        return route, intent