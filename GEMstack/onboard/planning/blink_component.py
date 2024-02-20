from ..component import Component
from ..interface.gem import GEMInterface,GEMVehicleCommand,GEMVehicleReading
       
class BlinkDistress(Component):
    """Your control loop code should go here.  You will use GEMVehicleCommand
    to communicate with drive-by-wire system to control the vehicle's turn signals.
    """
    def __init__(self, vehicle_interface : GEMInterface):
        self.vehicle_interface = vehicle_interface
        self.command = None
        self.num_blinks = -1
        self.signal_type = [0,2,1]

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
        #TODO: alter command to execute turn signals, then uncomment line below to send

        #maintain current values from readings
        command = self.vehicle_interface.command_from_reading()
        
        # print("Accelerator_pedal_position: ", self.vehicle_interface.last_reading.accelerator_pedal_position)
        # print("Brake_pedal_position: ", self.vehicle_interface.last_reading.brake_pedal_position)
        # print("Headlights_on: ", self.vehicle_interface.last_reading.headlights_on)
        # print("Horn_on: ", self.vehicle_interface.last_reading.horn_on)
        # print("Wiper_level: ", self.vehicle_interface.last_reading.wiper_level)
        # print("Steering_wheel_angle: ", self.vehicle_interface.last_reading.steering_wheel_angle)
        # print("Left_turn_signal: ", self.vehicle_interface.last_reading.left_turn_signal)
        # print("Right_turn_signal: ", self.vehicle_interface.last_reading.right_turn_signal)
        # print("Battery_level: ", self.vehicle_interface.last_reading.battery_level)
        # print("Fuel_level: ", self.vehicle_interface.last_reading.fuel_level)
        # print("Speed: ", self.vehicle_interface.last_reading.speed)
        # print("Gear: ", self.vehicle_interface.last_reading.gear)
        # print("Driving_range: ", self.vehicle_interface.last_reading.driving_range)
        self.num_blinks += 1
        value = self.signal_type[self.num_blinks%len(self.signal_type)]
        if value == 0:
            # right
            command.right_turn_signal = True
            command.left_turn_signal = False
        elif value == 1:
            #left
            command.right_turn_signal = False
            command.left_turn_signal = False
        else:
            command.right_turn_signal = False
            command.left_turn_signal = False
        self.vehicle_interface.send_command(command)
       
    def healthy(self):
        """Returns True if the element is in a stable state."""
        return True
       
