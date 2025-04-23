from ..component import Component
from ...state import AllState
import time

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
        print(f"AllState (parking goal): {state.goal}")
        return