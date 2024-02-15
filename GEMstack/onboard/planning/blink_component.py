from ..component import Component
from ..interface.gem import GEMInterface,GEMVehicleCommand,GEMVehicleReading


class BlinkDistress(Component):
    """Your control loop code should go here.  You will use GEMVehicleCommand
    to communicate with drive-by-wire system to control the vehicle's turn signals.
    """
    def __init__(self, vehicle_interface : GEMInterface):
        self.vehicle_interface = vehicle_interface
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
        command = self.vehicle_interface.command_from_reading()
        command.left_turn_signal = False
        command.right_turn_signal = False
        self.vehicle_interface.send_command(command)

    
    def update(self):
        """Run in a loop"""
        command = self.vehicle_interface.command_from_reading()
        
        # If they are off, start with the left one
        if self.state == 1:
            self.state = 2
            command.left_turn_signal = True
            command.right_turn_signal = False
        # If the left signal is already on, flash the right one
        elif self.state == 2:
            self.state = 0
            command.left_turn_signal = False
            command.right_turn_signal = True
        # If the right signal is already on, turn them off
        else:
            self.state = 1
            command.left_turn_signal = False
            command.right_turn_signal = False
        self.vehicle_interface.send_command(command)

    def healthy(self):
        """Returns True if the element is in a stable state."""
        return True
       
