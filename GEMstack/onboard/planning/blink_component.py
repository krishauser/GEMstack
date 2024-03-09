from ..component import Component
from ..interface.gem import GEMInterface,GEMVehicleCommand,GEMVehicleReading


class BlinkDistress(Component):
    """Your control loop code should go here.  You will use GEMVehicleCommand
    to communicate with drive-by-wire system to control the vehicle's turn signals.
    """
    def __init__(self, vehicle_interface : GEMInterface):
        self.vehicle_interface = vehicle_interface
        self.command = None

    def callback(self, data):
        rospy.loginfo(data.data)

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
        if command.left_turn_signal == False and command.right_turn_signal == False:
            command.left_turn_signal = True

        elif command.left_turn_signal == True and command.right_turn_signal == False:
            command.left_turn_signal = False
            command.right_turn_signal = True

        elif command.right_turn_signal == True and command.left_turn_signal == False:
            command.right_turn_signal = False
            command.left_turn_signal = False

        # alter command to execute turn signals
        self.vehicle_interface.send_command(command)
       
    def healthy(self):
        """Returns True if the element is in a stable state."""
        return True
       
