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

        self.time_last_command = 0
        self.state = 0

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
        if time.time() - self.time_last_command > 2.0:
            self.time_last_command = time.time()
            if self.state == 0:
                self.state = 1
                
                # turn left
                command.left_turn_signal = True
                command.right_turn_signal = False
            
            elif self.state == 1:
                self.state = 2

                # turn right
                command.left_turn_signal = False
                command.right_turn_signal = True

            elif self.state == 2:
                self.state = 0

                command.left_turn_signal = False
                command.right_turn_signal = False
        
        self.vehicle_interface.send_command(command)

        # Print sensor readings:
        print("Speed: {} m/s".format(self.vehicle_interface.last_reading.speed))
        print("Steering Wheel Angle: {} rad".format(self.vehicle_interface.last_reading.steering_wheel_angle))
       
    def healthy(self):
        """Returns True if the element is in a stable state."""
        return True
       
