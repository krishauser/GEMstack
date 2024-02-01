from ..component import Component
from ..interface.gem import GEMInterface,GEMVehicleCommand,GEMVehicleReading


class BlinkDistress(Component):
    """Your control loop code should go here.  You will use GEMVehicleCommand
    to communicate with drive-by-wire system to control the vehicle's turn signals.
    """
    def __init__(self, vehicle_interface : GEMInterface):
        self.vehicle_interface = vehicle_interface
        self.command = None
        self.state = None

    def rate(self):
        """Requested update frequency, in Hz"""
        return 0.5

    def initialize(self):
        """Run first"""
        pass

    def cleanup(self):
        """Run last"""
        self.state = None
        self.command.left_turn_signal = False
        self.command.right_turn_signal = False
    
    def update(self):
        """Run in a loop"""
        self.command = self.vehicle_interface.command_from_reading()
        # If the left signal is already on, flash the right one
        if self.state is None or self.state == 1:
            self.state = 2
            self.command.left_turn_signal = False
            self.command.right_turn_signal = True
        # If the right signal is already on, turn them off
        elif self.state == 0:
            self.state = 1
            self.command.left_turn_signal = False
            self.command.right_turn_signal = False
        # If they are off, start with the left one
        else:
            self.state = 0
            self.command.left_turn_signal = True
            self.command.right_turn_signal = False
        self.vehicle_interface.send_command(self.command)
       
    def healthy(self):
        """Returns True if the element is in a stable state."""
        return True
       
