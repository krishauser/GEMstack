from pydantic import BaseModel, Field
from typing import List, Tuple, Union
from ..component import Component
from ...state import AllState, VehicleState, EntityRelation, EntityRelationEnum, Path, Trajectory, Route, ObjectFrameEnum, AgentState, MissionObjective
import requests
import copy


### MODELS ###
def check_point_exists(server_url="http://localhost:8000"):
    try:
        response = requests.get(f"{server_url}/point")
        response.raise_for_status()
        points = response.json().get("points", [])
    
        for point in points:
            return True, [point["x"], point["y"]]
        return False, []

    except requests.exceptions.RequestException as e:
        print("Error contacting server:", e)
        return False, []

class GetPoint(Component):
    """Follows the given route.  Brakes if you have to yield or
    you are at the end of the route, otherwise accelerates to
    the desired speed.
    """

    def __init__(self, endpoint, type):
        self.api_endpoint = endpoint
        self.bounding_box = None

    def state_inputs(self):
        return ['all']

    def state_outputs(self) -> List[str]:
        return ['all']

    def rate(self):
        return 0.2

    def update(self, state: AllState):
        # request the bounding box, if found then update and then switch the state
        points_found = False
        points_found, pts = check_point_exists()
        if points_found:
            state.goal = pts
            print(state.mission.state_list[state.mission.index+1])
            state.mission.type = state.mission.state_list[state.mission.index+1]
            state.mission.index += 1
            print("CHANGING STATES", state.mission.type)
        return copy.deepcopy(state)
