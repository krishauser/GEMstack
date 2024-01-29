from ..component import Component
from ..interface.gem import GEMInterface,GEMVehicleCommand,GEMVehicleReading


class BlinkDistress(Component):
    """Your control loop code should go here.  You will use GEMVehicleCommand
    to communicate with drive-by-wire system to control the vehicle's turn signals.
    """
    def __init__(self, vehicle_interface : GEMInterface):
        self.vehicle_interface = vehicle_interface
        self.command = None

        self.sos = [2, 2, 0, 0, 1, 1]
        self.index = 0

    def rate(self):
        """Requested update frequency, in Hz"""
        return 1.0

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
        # TODO: alter command to execute turn signals, then uncomment line below to send
        # the command to vehicle
        # self.vehicle_interface.send_command(command)
        signal = self.sos[self.index]
        if signal == 2:
            # Left
            command.left_turn_signal = True
            command.right_turn_signal = False
        elif signal == 0:
            # Right
            command.left_turn_signal = False
            command.right_turn_signal = True
        else:
            command.left_turn_signal = False
            command.right_turn_signal = False
        print(f"Current signal {signal}")
        self.vehicle_interface.send_command(command)
       
    def healthy(self):
        """Returns True if the element is in a stable state."""
        return True
       
