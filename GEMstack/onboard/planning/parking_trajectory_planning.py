from typing import List
from ..component import Component
from ...utils import serialization
from ...state import AllState, VehicleIntent, VehicleIntentEnum, MissionObjective, MissionEnum, Trajectory, Path
import numpy as np

from .longitudinal_planning import longitudinal_plan


### FOR TEST ###
def generate_turn_trajectory(vehicle, turn_angle=45, turn_radius=3.0, turn_direction="right",
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

    traj = longitudinal_plan(path, acceleration, deceleration, max_speed, current_speed)

    return traj


class ParkingTrajectoryPlanner(Component):
    def state_inputs(self):
        return ['all']

    def state_outputs(self) -> List[str]:
        return ['trajectory', 'intent', 'mission']

    def rate(self):
        return 10.0

    def update(self, state: AllState):
        intent = state.intent
        mission = state.mission
        vehicle = state.vehicle
        roadgraph = state.roadgraph

        print('Input:')
        print(intent)
        print(mission)

        traj = None
        # TODO: implement parking trajectory planning
        if intent.intent == VehicleIntentEnum.HEAD_IN_PARKING:
            pass
        elif intent.intent == VehicleIntentEnum.BACK_IN_PARKING:
            pass
        elif intent.intent == VehicleIntentEnum.PARALLEL_PARKING:
            pass
        ### FOR TEST ###
        # TODO: Modify to stop aside the target location
        elif intent.intent == VehicleIntentEnum.PARKING:    # for test
            traj = generate_turn_trajectory(vehicle, turn_angle=45, turn_radius=3.0, turn_direction="right", num_points=10)

        intent.intent = VehicleIntentEnum.IDLE
        mission.type = MissionEnum.IDLE

        print('Output:')
        print(intent)
        print(mission)

        return traj, intent, mission
