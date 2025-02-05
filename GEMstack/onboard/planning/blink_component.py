from ..component import Component
from ..interface.gem import GEMInterface,GEMVehicleCommand,GEMVehicleReading
import time

class BlinkDistress(Component):
    """Your control loop code should go here.  You will use GEMVehicleCommand
    to communicate with drive-by-wire system to control the vehicle's turn signals.
    """
    def __init__(self, vehicle_interface : GEMInterface):
        self.vehicle_interface = vehicle_interface
        # command = GEMVehicleCommand()

        # command.left_turn_signal = True
        self.prev_time = time.time()

        while True :
            self.update()

        
        

    def rate(self):
        """Requested update frequency, in Hz"""
        return 0.5

    def initialize(self):
        """Run first"""
        passcommand

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

        # print("curr_accel is {}".format(command.accelerator_pedal_position))
        # print("curr_brake is {}".format(command.brake_pedal_position))
        print("right turn {}".format(command.right_turn_signal))
        print("left turn {}".format(command.left_turn_signal))
        self.curr_time = time.time()

        if (self.curr_time - self.prev_time > 2):
            if(command.left_turn_signal is True):
                command.left_turn_signal = False
                command.right_turn_signal = True
                print("Condition 1")
            elif (command.right_turn_signal is True):
                command.left_turn_signal = False
                command.right_turn_signal = False
                print("Conditon 2")
            else:
                command.left_turn_signal = True
                print("Condition 3")
            self.prev_time = self.curr_time
            # print("theoretical right turn {}".format(command.right_turn_signal))
            # print("theoretical left turn {}".format(command.left_turn_signal))

        self.vehicle_interface.send_command(command)

        
        # command = self.vehicle_interface.command_from_reading()
        # print("real right turn {}".format(command.right_turn_signal))
        # print("real left turn {}".format(command.left_turn_signal))
       
    def healthy(self):
        """Returns True if the element is in a stable state."""
        return True
       
