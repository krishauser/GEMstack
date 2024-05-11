from ..component import Component
from ..interface.gem import GEMInterface,GEMVehicleCommand,GEMVehicleReading
from time import sleep

class BlinkDistress(Component):
    """Your control loop code should go here.  You will use GEMVehicleCommand
    to communicate with drive-by-wire system to control the vehicle's turn signals.
    """
    time = 0
    def __init__(self, vehicle_interface : GEMInterface):
        self.vehicle_interface = vehicle_interface
        self.command = None
        self.timer = 0
        self.rate = 0.5

    def rate(self):
        """Requested update frequency, in Hz"""
        return self.rate

    def initialize(self):
        """Run first"""
        pass

    def cleanup(self):
        """Run last"""
        pass
    
    def update(self):
        """Run in a loop"""
        # we need to set up a GEMVehicleCommand which encapsulates all commands that will be
        # sent to the drive-by-wire system, simultaneously.  To avoid doing arbitrary things
        # to the vehicle, let's maintain the current values (e.g., accelerator, brake pedal,
        # steering angle) from its current readings.
 
        command = self.vehicle_interface.command_from_reading()

        self.timer += 1/self.rate
        self.timer %= 6

        if (self.timer == 2):
            command.left_turn_signal = True
            command.right_turn_signal = False
        elif (self.timer == 4):
            command.left_turn_signal = False
            command.right_turn_signal = True
        elif (self.timer == 0):
            command.left_turn_signal = False
            command.right_turn_signal = False

        # TODO: alter command to execute turn signals, then uncomment line below to send
        # the command to vehicle
        self.vehicle_interface.send_command(command)
       
    def healthy(self):
        """Returns True if the element is in a stable state."""
        return True