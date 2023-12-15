from typing import List
from ..component import Component
from ...state import AllState, Trajectory, Route
from ...utils import serialization

class RouteToTrajectoryPlanner(Component):
    """Copies the route directly into the trajectory."""
    def __init__(self):
        pass

    def state_inputs(self):
        return ['all']

    def state_outputs(self) -> List[str]:
        return ['trajectory']

    def rate(self):
        return 10.0

    def update(self, state : AllState):
        return Trajectory(frame=state.route.frame,
                   points=state.route.points,
                   times=list(range(len(state.route.points))))
    