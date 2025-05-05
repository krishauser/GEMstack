from typing import List
from ..component import Component
from ...state import AllState,VehicleIntent

class DrivingLogic(Component):
     # count of times blinked 
    """Base class for top-level components in the execution stack."""
    def rate(self) -> float:
        """Returns the rate in Hz at which this component should be updated."""
        return .5
    def state_inputs(self) -> List[str]:
        """Returns the list of AllState inputs this component requires."""
        return ['all']
    def state_outputs(self) -> List[str]:
        """Returns the list of AllState outputs this component generates."""
        return ['intent']
    def healthy(self):
        """Returns True if the element is in a stable state."""
        return True
    def initialize(self):
        """Initialize the component. This is called once before the first
        update."""
        self.count = 0 
        return
    def cleanup(self):
        """Cleans up resources used by the component. This is called once after
        the last update."""
        pass
    def update(self, state: AllState):
        """Update the component."""

        state.intent.intent = 2

        return
    def debug(self, item, value):
        """Debugs a streaming value within this component"""
        if hasattr(self, 'debugger'):
            self.debugger.debug(item, value)
    def debug_event(self, label):
        """Debugs an event within this component"""
        if hasattr(self, 'debugger'):
            self.debugger.debug_event(label)