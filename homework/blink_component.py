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
        
        self.prev_time = time.time()
        self.curr_time = time.time()
               
        while True :
            self.curr_time = time.time()
            if (self.prev_time - self.curr_time > 2):
                self.update()
                self.prev_time = self.curr_time

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
        # self.vehicle_interface.send_command(command)
        
        if command.left_turn_signal is True:
            command.left_turn_signal = False
            command.right_turn_signal = True
            
        elif command.right_turn_signal is True:
            command.left_turn_signal = False
            command.right_turn_signal = False
            
        else:
            command.left_turn_signal = True
            command.right_turn_signal = False
        
        self.vehicle_interface.send_command(command)
       
    def healthy(self):
        """Returns True if the element is in a stable state."""
        return True
       
