import rospy
from std_msgs.msg import String, Bool, Float32, Float64
from pacmod_msgs.msg import PositionWithSpeed, PacmodCmd, SystemRptInt, SystemRptFloat, VehicleSpeedRpt
import time

# For message format, see
# https://github.com/astuff/astuff_sensor_msgs/blob/3.3.0/pacmod_msgs/msg/PacmodCmd.msg

class BlinkDistress:
    """Your control loop code should go here.  You will use ROS and Pacmod messages
    to communicate with drive-by-wire system to control the vehicle's turn signals.
    """
    def __init__(self):
        # TODO: Initialize your publishers and subscribers here
        # When you create a subscriber to a /pacmod/parsed_tx/X topic, you will need to provide a callback function.
        # You will want this callback to be a BlinkDistress method, such as print_X(self, msg).  msg will have a
        # ROS message type, and you can find out what this is by either reading the documentation or running
        # "rostopic info /pacmod/parsed_tx/X" on the command line.
        self.sub_accel_rpt = rospy.Subscriber('/pacmod/parsed_tx/accel_rpt', SystemRptFloat, self.callback_accel_rpt)
        self.sub_vehicle_speed_rpt = rospy.Subscriber('/pacmod/parsed_tx/vehicle_speed_rpt', VehicleSpeedRpt, self.callback_vehicle_speed_rpt)
        
        self.pub_turn_cmd = rospy.Publisher('/pacmod/as_rx/turn_cmd',PacmodCmd, queue_size=10)
        # self.sub_brake_rpt = self.create_subscription(SystemRptFloat, '/pacmod/parsed_tx/brake_rpt', self.callback_brake_rpt, 10)
        # self.sub_steer_rpt = self.create_subscription(SystemRptFloat, '/pacmod/parsed_tx/steer_rpt', self.callback_steer_rpt, 10)
        # self.sub_shift_rpt = self.create_subscription(SystemRptFloat, '/pacmod/parsed_tx/shift_rpt', self.callback_shift_rpt, 10)
        # self.sub_turn_rpt = self.create_subscription(SystemRptFloat, '/pacmod/parsed_tx/turn_rpt', self.callback_turn_rpt, 10)
        self.time_in_state = 0
        pass

    def callback_accel_rpt(self, msg):
        self.manual_input = msg.manual_input
        self.command = msg.command
        self.output = msg.output
        print('accel_rpt_manual_input = {}'.format(self.manual_input))
        print('accel_rpt_command = {}'.format(self.command))
        print('accel_rpt_output = {}'.format(self.output))

    def callback_vehicle_speed_rpt(self, msg):
        self.vehicle_speed = msg.vehicle_speed
        print('vehicle_speed = {}'.format(self.vehicle_speed))

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
        # TODO: Implement your control loop here
        # You will need to publish a PacmodCmd() to /pacmod/as_rx/turn_cmd.  Read the documentation to see
        # what the data in the message indicates.
        turn_cmd = PacmodCmd()
        self.time_in_state +=  1
        self.pub_turn_cmd.publish(turn_cmd)

        if self.time_in_state % 3 == 1:
            # turn_cmd.clear = True
            turn_cmd.ui16_cmd = 2
            self.pub_turn_cmd.publish(turn_cmd)

        if self.time_in_state % 3 == 2:
            # turn_cmd.clear = True
            turn_cmd.ui16_cmd = 0
            self.pub_turn_cmd.publish(turn_cmd)

        if self.time_in_state % 3 == 0:
            # turn_cmd.clear = True
            turn_cmd.ui16_cmd = 1
            self.pub_turn_cmd.publish(turn_cmd)
        pass
       
    def healthy(self):
        """Returns True if the element is in a stable state."""
        return True
       
    def done(self):
        """Return True if you want to exit."""
        return False



def run_ros_loop(node):
    """Executes the event loop of a node using ROS.  `node` should support
    rate(), initialize(), cleanup(), update(), and done().  You should not modify
    this code.
    """
    #intializes the node. We use disable_signals so that the end() function can be called when Ctrl+C is issued.
    #See http://wiki.ros.org/rospy/Overview/Initialization%20and%20Shutdown
    rospy.init_node(node.__class__.__name__, disable_signals=True)

    node.initialize()
    rate = rospy.Rate(node.rate())
    termination_reason = "undetermined"
    try:
        while not rospy.is_shutdown() and not node.done() and node.healthy():
            node.update()
            rate.sleep()
        if node.done():
            termination_reason = "Node done"
        if not node.healthy():
            termination_reason = "Node in unhealthy state"
    except KeyboardInterrupt:
        termination_reason = "Killed by user"
    except Exception as e:
        termination_reason = "Exception "+str(e)
        raise
    finally:
        node.cleanup()
        rospy.signal_shutdown(termination_reason)

if __name__ == '__main__':
    run_ros_loop(BlinkDistress())