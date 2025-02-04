from ..component import Component
from ..interface.gem import GEMInterface,GEMVehicleCommand,GEMVehicleReading
import time

TURN_OFF = 0
TURN_LEFT = 1
TURN_RIGHT = 2

class BlinkDistress(Component):
    """Your control loop code should go here.  You will use GEMVehicleCommand
    to communicate with drive-by-wire system to control the vehicle's turn signals.
    """
    def __init__(self, vehicle_interface : GEMInterface):
        self.vehicle_interface = vehicle_interface
        self.command = None
        self.turn_state = TURN_OFF  # Start with turn signals off
        self.last_update_time = time.time()

    def rate(self):
        """Requested update frequency, in Hz"""
        return 0.5

    def initialize(self):
        """Run first"""
        print("BlinkDistress Component Initialized.")
        self.send_turn_command(TURN_OFF)
        # pass

    def cleanup(self):
        """Run last"""
        print("Cleaning up... Turning off blinkers.")
        self.send_turn_command(TURN_OFF)
        # pass
    
    def update(self):
        """Run in a loop"""
        # we need to set up a GEMVehicleCommand which encapsulates all commands that will be
        # sent to the drive-by-wire system, simultaneously.  To avoid doing arbitrary things
        # to the vehicle, let's maintain the current values (e.g., accelerator, brake pedal,
        # steering angle) from its current readings.
        current_time = time.time()
        if current_time - self.last_update_time >= 2:  # Change signal every 2 seconds
            if self.turn_state == TURN_OFF:
                self.turn_state = TURN_LEFT
            elif self.turn_state == TURN_LEFT:
                self.turn_state = TURN_RIGHT
            else:
                self.turn_state = TURN_OFF

            self.send_turn_command(self.turn_state)
            self.last_update_time = current_time
        
         # Read vehicle sensor data
        vehicle_reading = self.vehicle_interface.get_reading()
        print(f"Vehicle Speed: {vehicle_reading.speed:.2f} m/s")
        print(f"Acceleration: {vehicle_reading.accelerator_pedal_position - vehicle_reading.brake_pedal_position:.2f} m/s²")
        # command = self.vehicle_interface.command_from_reading()
        # TODO: alter command to execute turn signals, then uncomment line below to send
        # the command to vehicle
        # self.vehicle_interface.send_command(command)
    
    def send_turn_command(self, turn_signal):
        """Send the updated turn signal command to the vehicle"""
        command = self.vehicle_interface.command_from_reading()

        # Update only turn signals, keeping other controls unchanged
        if turn_signal == TURN_LEFT:
            command.left_turn_signal = True
            command.right_turn_signal = False
        elif turn_signal == TURN_RIGHT:
            command.left_turn_signal = False
            command.right_turn_signal = True
        else:
            command.left_turn_signal = False
            command.right_turn_signal = False

        self.vehicle_interface.send_command(command)
        print(f"Turn Signal Set: {'LEFT' if turn_signal == TURN_LEFT else 'RIGHT' if turn_signal == TURN_RIGHT else 'OFF'}")


    def healthy(self):
        """Returns True if the element is in a stable state."""
        return True
       
