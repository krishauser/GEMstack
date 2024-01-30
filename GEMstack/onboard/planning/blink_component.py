from ..component import Component
from ..interface.gem import GEMInterface,GEMVehicleCommand,GEMVehicleReading
import time

class BlinkDistress(Component):
    """Your control loop code should go here.  You will use GEMVehicleCommand
    to communicate with drive-by-wire system to control the vehicle's turn signals.
    """
    def __init__(self, vehicle_interface : GEMInterface):
        self.vehicle_interface = vehicle_interface
        self.command = None

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
        command = self.vehicle_interface.command_from_reading()
        # TODO: alter command to execute turn signals, then uncomment line below to send
        # the command to vehicle
        dt = 0.1
        time_in_state = 0.0
        self.turn_signal_state = 'left'
        command.left_turn_signal = True
        command.right_turn_signal = False
        self.vehicle_interface.send_command(command)
        while True:
            if self.turn_signal_state == 'left':
                if time_in_state >= 2.0:
                    time_in_state = 0.0
                    self.turn_signal_state == 'right'
                    command.left_turn_signal = False
                    command.right_turn_signal = True
                    self.vehicle_interface.send_command(command)
            elif self.turn_signal_state == 'right':
                if time_in_state >= 2.0:
                    time_in_state = 0.0
                    self.turn_signal_state == 'off'
                    command.left_turn_signal = False
                    command.right_turn_signal = False
                    self.vehicle_interface.send_command(command)
            elif self.turn_signal_state == 'off':
                if time_in_state >= 2.0:
                    time_in_state = 0.0
                    self.turn_signal_state == 'left'
                    command.left_turn_signal = True
                    command.right_turn_signal = False
                    self.vehicle_interface.send_command(command)
            time.sleep(dt)
            time_in_state = time_in_state + dt
       
    def healthy(self):
        """Returns True if the element is in a stable state."""
        return True
       
