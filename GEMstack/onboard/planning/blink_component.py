from ..component import Component
from ..interface.gem import GEMInterface,GEMVehicleCommand,GEMVehicleReading


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
        self.turn_state = "left"
        pass

    def cleanup(self):
        """Run last"""
        pass
    def print_sensor_readings(self):
        """Test to print readings from GEMVehicleReading"""
        reading = self.vehicle_interface.get_reading()
        print("Vehicle speed = ", reading.speed)
        print("Steering whee angle = ", reading.steering_wheel_angle)
    
    def update(self):
        """Run in a loop"""
        # we need to set up a GEMVehicleCommand which encapsulates all commands that will be
        # sent to the drive-by-wire system, simultaneously.  To avoid doing arbitrary things
        # to the vehicle, let's maintain the current values (e.g., accelerator, brake pedal,
        # steering angle) from its current readings.
        command = self.vehicle_interface.command_from_reading()
        # TODO: alter command to execute turn signals, then uncomment line below to send
        # the command to vehicle

        # self.print_sensor_readings()

        if self.turn_state == 'left':
            print('left')
            command.left_turn_signal = True # turn left
            self.vehicle_interface.send_command(command)
            self.turn_state = "right"
        elif self.turn_state == "right":
            print('right')
            command.left_turn_signal = False
            self.vehicle_interface.send_command(command)
            command.right_turn_signal = True
            self.vehicle_interface.send_command(command)
            self.turn_state = "none"
        else: 
            print('none')
            command.right_turn_signal = False
            self.vehicle_interface.send_command(command)
            self.turn_state = "left"
       
    def healthy(self):
        """Returns True if the element is in a stable state."""
        return True
       
