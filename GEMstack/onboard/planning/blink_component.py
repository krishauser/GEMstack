from ..component import Component
from ..interface.gem import GEMInterface, GEMVehicleCommand, GEMVehicleReading
from pacmod_msgs.msg import PositionWithSpeed, PacmodCmd, SystemRptInt, SystemRptFloat, VehicleSpeedRpt


class BlinkDistress(Component):
    """Your control loop code should go here.  You will use GEMVehicleCommand
    to communicate with drive-by-wire system to control the vehicle's turn signals.
    """

    def __init__(self, vehicle_interface: GEMInterface):
        self.vehicle_interface = vehicle_interface
        self.command = None
        self.turn_status_list = [  # left_turn_signal, right_turn_signal
            [True, False],
            [False, True],
            [False, False],
        ]
        self.turn_idx = 0

    def rate(self):
        """Requested update frequency, in Hz"""
        return 0.5

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
        # the command to vehicle

        # TODO: alter command to execute turn signals, then uncomment line below to send
        command = self.vehicle_interface.command_from_reading()
        self.turn_idx = (self.turn_idx + 1) % len(self.turn_status_list)

        # update command
        command.left_turn_signal = self.turn_status_list[self.turn_idx][0]
        command.right_turn_signal = self.turn_status_list[self.turn_idx][1]

        self.vehicle_interface.send_command(command)

    def healthy(self):
        """Returns True if the element is in a stable state."""
        return True
