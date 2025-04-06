from pydantic import BaseModel, Field
from typing import List, Tuple, Union
from ..component import Component
from ...state import AllState, VehicleState, EntityRelation, EntityRelationEnum, Path, Trajectory, Route, ObjectFrameEnum, AgentState
import requests


### MODELS ###

class BoundingBox(BaseModel):
    x1: float = Field(..., ge=-90, le=90)
    x2: float = Field(..., ge=-90, le=90)
    y1: float = Field(..., ge=-90, le=90)
    y2: float = Field(..., ge=-90, le=90)


class BoundingBoxRequester(Component):
    """Follows the given route.  Brakes if you have to yield or
    you are at the end of the route, otherwise accelerates to
    the desired speed.
    """

    def __init__(self, mode : str = 'real', params : dict = {"api_endpoint": "xxxx"}):
        self.api_endpoint = params['api_endpoint']
        self.bounding_box = None

    def state_inputs(self):
        return ['all']

    def state_outputs(self) -> List[str]:
        return self.bounding_box

    def rate(self):
        return 10.0

    def update(self, state: AllState):
        # request the bounding box, if found then update and then switch the state
        if self.bounding_box:
            return
