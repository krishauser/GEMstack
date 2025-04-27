from ..component import Component
from typing import List

class Driving_Logic(Component):
    def __init__(self):
        self.intent = 2 # halting

    def state_outputs(self) -> List[str]:
        """Returns the list of AllState outputs this component generates."""
        return ['intent']
    def update(self, *args, **kwargs):
        """Update the component."""
        return self.intent