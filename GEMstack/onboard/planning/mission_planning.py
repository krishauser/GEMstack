from typing import List, Union
from klampt.vis import scene
from ..component import Component
from ...state import Route, ObjectFrameEnum, AllState, VehicleState, Roadgraph, MissionObjective, MissionEnum, ObjectPose
import numpy as np
import requests
import time
import yaml


def check_distance(goal_pose: Union[ObjectPose, list], current_pose : ObjectPose):
    if isinstance(goal_pose, ObjectPose):
        goal = np.array([goal_pose.x, goal_pose.y])
    else:
        goal = np.array([goal_pose[0], goal_pose[1]])
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
    def __init__(self, use_webapp, webapp_url, goal=None, state_machine=None):
        self.state_machine = StateMachine([eval(s) for s in state_machine])

        # if use_webapp is True, goal should be None
        self.goal_location = goal['location'] if not use_webapp else None
        self.goal_frame = goal['frame'] if not use_webapp else None
        self.new_goal = False
        self.goal_pose = None
        self.start_pose = None
        self.start_time = time.time()
        self.end_of_driving_route =  None
        self.search_count = 0

        self.flag_use_webapp = use_webapp
        self.url_status = f"{webapp_url}/api/status"
        self.url_summon = f"{webapp_url}/api/summon"

        if self.flag_use_webapp:
            # Initialize the state in the server
            data = {
                "status": "IDLE"
            }
            response = requests.post(url=self.url_status, json=data)
            if response.status_code == 200:
                print("Status updated successfully")
            else:
                print("Failed to update status:", response.status_code)
            data = {
                "lat": 0,
                "lon": 0
            }
            response = requests.post(url=self.url_summon, json=data)
            if response.status_code == 200:
                print("Initialize goal location successfully")
            else:
                print("Failed to initialize goal location:", response.status_code)

    def state_inputs(self):
        return ['all']

    def state_outputs(self) -> List[str]:
        return ['mission']

    def rate(self):
        return 1.0

    def update(self, state: AllState):
        vehicle = state.vehicle
        mission = state.mission
        route = state.route

        if self.flag_use_webapp:
            goal_location = None
            goal_frame = None
            response = requests.get(self.url_summon)
            print("GET:", response)
            if response.status_code == 200:
                data = response.json()
                if data['lat'] == 0 and data['lon'] == 0:   # TODO: lat and lon equal to 0 is meaningful, should change
                    print("No goal location received")
                    goal_location = None
                    goal_frame = None
                else:
                    goal_location = [data['lon'] , data['lat']]
                    goal_frame = 'global'
                    print("Goal location:", goal_location)
                    print("Goal frame:", goal_frame)

            if self.goal_location == goal_location:
                self.new_goal = False
            else:
                self.new_goal = True
                self.goal_location = goal_location
                self.goal_frame = goal_frame
        else:
            self.new_goal = True
            goal_location = self.goal_location
            goal_frame = self.goal_frame

        start_vehicle_pose = state.start_vehicle_pose

        if goal_frame == 'global':
            self.goal_pose = ObjectPose(t=time.time(), x=goal_location[0], y=goal_location[1],
                                        frame=ObjectFrameEnum.GLOBAL)
        elif goal_frame == 'cartesian':
            self.goal_pose = ObjectPose(t=time.time(), x=goal_location[0], y=goal_location[1],
                                    frame=ObjectFrameEnum.ABSOLUTE_CARTESIAN)
        elif goal_frame == 'start':
            self.goal_pose = ObjectPose(t=time.time() - self.start_time, x=goal_location[0], y=goal_location[1],
                                        frame=ObjectFrameEnum.START)
        elif goal_frame is None:
            pass
        else:
            raise ValueError("Invalid frame argument")

        if self.goal_pose:
            if start_vehicle_pose.frame == ObjectFrameEnum.ABSOLUTE_CARTESIAN and goal_frame == 'global':
                # For global map simulation test
                with open("scenes/summoning_map_sim_setting.yaml", "r") as f:
                    data = yaml.safe_load(f)
                    start= data['start_pose_global']
                start_pose_global = ObjectPose(frame=ObjectFrameEnum.GLOBAL, t=start_vehicle_pose.t, x=start[0], y=start[1], yaw=start[2])
                self.goal_pose = self.goal_pose.to_frame(ObjectFrameEnum.START, start_pose_abs=start_pose_global)
            else:
                self.goal_pose = self.goal_pose.to_frame(ObjectFrameEnum.START, start_pose_abs=start_vehicle_pose)
            print("Goal pose:", self.goal_pose)

        # Initiate state
        if mission is None:
            mission = MissionObjective()
            mission.type = self.state_machine.initial_state

        # Receive goal location from the server and start driving
        elif mission.type == MissionEnum.IDLE:
            if self.new_goal:
                mission.goal_pose = self.goal_pose
                mission.type = self.state_machine.next_state()
                self.new_goal = False
                print("============== Next state:", mission.type)

        # Reach the end of the route, begin to search for parking
        # elif mission.type == MissionEnum.SUMMONING_DRIVE:
        elif mission.type == MissionEnum.SUMMON_DRIVING:
            mission.goal_pose = self.goal_pose
            if route:
                _, closest_index = route.closest_point([vehicle.pose.x, vehicle.pose.y], edges=False)
                if closest_index == len(route.points) - 1 or check_distance(mission.goal_pose, vehicle.pose) < 5:
                    if vehicle.v < 0.1:
                        mission.type = self.state_machine.next_state()
                        print("============== Next state:", mission.type)
                self.end_of_driving_route = route.points[-1]
            else:
                self.search_count += 1

        # Finish parking, back to idle and wait for the next goal location
        elif mission.type == MissionEnum.PARALLEL_PARKING:
            if route:
                if route.points[-1] != self.end_of_driving_route:
                    _, closest_index = route.closest_point([vehicle.pose.x, vehicle.pose.y], edges=False)
                    if vehicle.v < 0.01 and (closest_index >= len(route.points) - 1 or check_distance(route.points[-1], vehicle.pose) < 0.5):
                        # Set everything to idle
                        mission.type = self.state_machine.next_state()
                        print("============== Next state:", mission.type)
            else:
                self.search_count += 1

        else:
            raise ValueError("Invalid mission type")

        # Can not find a path, stop mission.
        if mission.type != MissionEnum.IDLE:
            print("Route searching times:", self.search_count)
            if self.search_count > 10:
                mission.type = MissionEnum.IDLE
                self.goal_pose = None
                self.goal_location = None
                self.goal_frame = None
                self.search_count = 0


        if self.flag_use_webapp:
            data = {
                "status": mission.type.name
            }
            print("POST:", data)
            response = requests.post(url=self.url_status, json=data)
            if response.status_code == 200:
                print("Status updated successfully")
            else:
                print("Failed to update status:", response.status_code)

        return mission