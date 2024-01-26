from ..component import Component
from ..interface.gem import GEMInterface,GEMVehicleCommand,GEMVehicleReading


class BlinkDistress(Component):
    """Your control loop code should go here.  You will use GEMVehicleCommand
    to communicate with drive-by-wire system to control the vehicle's turn signals.
    """
    def __init__(self, vehicle_interface : GEMInterface):
        self.vehicle_interface = vehicle_interface
        self.command = None
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
        print("speed: ", self.vehicle_interface.last_reading.speed)
        print("gear: ", self.vehicle_interface.last_reading.gear)
        print("accelerator_pedal_position: ", self.vehicle_interface.last_reading.accelerator_pedal_position)
        print("brake_pedal_position: ", self.vehicle_interface.last_reading.brake_pedal_position)
        print("steering_wheel_angle: ", self.vehicle_interface.last_reading.steering_wheel_angle)
        print("left_turn_signal: ", self.vehicle_interface.last_reading.left_turn_signal)
        print("right_turn_signal: ", self.vehicle_interface.last_reading.right_turn_signal)
        print("headlights_on: ", self.vehicle_interface.last_reading.headlights_on)
        print("horn_on: ", self.vehicle_interface.last_reading.horn_on)
        print("wiper_level: ", self.vehicle_interface.last_reading.wiper_level)
        print("battery_level: ", self.vehicle_interface.last_reading.battery_level)
        print("fuel_level: ", self.vehicle_interface.last_reading.fuel_level)
        print("driving_range: ", self.vehicle_interface.last_reading.driving_range)

        # TODO: alter command to execute turn signals, then uncomment line below to send
        # the command to vehicle
        if self.state == 0:
            command.left_turn_signal = True
            command.right_turn_signal = False
            self.state = 1
        elif self.state == 1:
            command.left_turn_signal = False
            command.right_turn_signal = True
            self.state = 2
        else:
            command.left_turn_signal = False
            command.right_turn_signal = False
            self.state = 0

        self.vehicle_interface.send_command(command)
       
    def healthy(self):
        """Returns True if the element is in a stable state."""
        return True
       
