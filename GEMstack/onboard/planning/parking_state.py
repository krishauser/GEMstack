from typing import List
from ..component import Component
from ...state import AllState
import time
import rospy

class FakeParkingSim(Component):
    def __init__(self):
        self.start_time = time.time()
        pass

    def state_inputs(self):
        return ["all"]

    def rate(self):
        return 10.0

    def state_outputs(self):
        return ["goal"]
    
    def update(self, state: AllState):
        print(f"---parking state update called--------")
        print(f"state parking goal: {state.goal}")

        mission_plan = []
        return mission_plan