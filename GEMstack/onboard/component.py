from typing import List

class Component:
    """Base class for top-level components in the execution stack."""
    def rate(self) -> float:
        """Returns the rate in Hz at which this component should be updated."""
        return 10.0
    def state_inputs(self) -> List[str]:
        """Returns the list of AllState inputs this component requires."""
        return []
    def state_outputs(self) -> List[str]:
        """Returns the list of AllState outputs this component generates."""
        return []
    def healthy(self):
        """Returns True if the element is in a stable state."""
        return True
    def initialize(self):
        """Initialize the component. This is called once before the first
        update."""
        pass
    def cleanup(self):
        """Cleans up resources used by the component. This is called once after
        the last update."""
        pass
    def update(self, *args, **kwargs):
        """Update the component."""
        raise NotImplementedError()
    def debug(self, item, value):
        """Debugs a streaming value within this component"""
        if hasattr(self, 'debugger'):
            self.debugger.debug(item, value)
    def debug_event(self, label):
        """Debugs an event within this component"""
        if hasattr(self, 'debugger'):
            self.debugger.debug_event(label)