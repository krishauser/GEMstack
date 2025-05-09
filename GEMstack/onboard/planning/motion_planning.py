from typing import List
from ..component import Component
from ...state import AllState, Trajectory, Route
from ...utils import serialization

class RouteToTrajectoryPlanner(Component):
    """Copies the route directly into the trajectory."""
    def __init__(self, reference_speed = 1.0):
        self.reference_speed = reference_speed

    def state_inputs(self):
        return ['all']

    def state_outputs(self) -> List[str]:
        return ['trajectory']

    def rate(self):
        return 10.0

    def update(self, state : AllState):
        if self.reference_speed is None:
            return state.route
        return state.route.arc_length_parameterize(self.reference_speed)
    