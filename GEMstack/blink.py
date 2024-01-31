import rospy
from std_msgs.msg import String, Bool, Float32, Float64
from pacmod_msgs.msg import (
    PositionWithSpeed,
    PacmodCmd,
    SystemRptInt,
    SystemRptFloat,
    VehicleSpeedRpt,
)

# For message format, see
# https://github.com/astuff/astuff_sensor_msgs/blob/3.3.0/pacmod_msgs/msg/PacmodCmd.msg


def brake_callback(data):
    rospy.loginfo(rospy.get_caller_id() + ": Brake Value - %f", data.manual_input)


def accel_callback(data):
    rospy.loginfo(rospy.get_caller_id() + ": Accel Value - %f", data.manual_input)

class BlinkDistress:
    """Your control loop code should go here.  You will use ROS and Pacmod messages
    to communicate with drive-by-wire system to control the vehicle's turn signals.
    """

    def __init__(self):
        # TODO: Initialize your publishers and subscribers here
        self.accel_sub = rospy.Subscriber("accel_sub", SystemRptFloat, accel_callback)
        self.brake_sub = rospy.Subscriber("brake_sub", SystemRptFloat, brake_callback)
        self.turn_pub = rospy.Publisher("turn_pub", PacmodCmd, queue_size=12)
        # When you create a subscriber to a /pacmod/parsed_tx/X topic, you will need to provide a callback function.
        # You will want this callback to be a BlinkDistress method, such as print_X(self, msg).  msg will have a
        # ROS message type, and you can find out what this is by either reading the documentation or running
        # "rostopic info /pacmod/parsed_tx/X" on the command line.

        pass

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
        rate = rospy.Rate(1/2)

        while not rospy.is_shutdown():
            turn_cmd = PacmodCmd()
            turn_cmd.enable=True
            turn_cmd.ui16_cmd = PacmodCmd.TURN_LEFT
            self.turn_pub.publish(turn_cmd)
            rate.sleep()
            turn_cmd.ui16_cmd = PacmodCmd.TURN_RIGHT
            self.turn_pub.publish(turn_cmd)
            rate.sleep()
            turn_cmd.ui16_cmd = PacmodCmd.TURN_NONE
            self.turn_pub.publish(turn_cmd)
            rate.sleep()
        # You will need to publish a PacmodCmd() to /pacmod/as_rx/turn_cmd.  Read the documentation to see
        # what the data in the message indicates.
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
    # intializes the node. We use disable_signals so that the end() function can be called when Ctrl+C is issued.
    # See http://wiki.ros.org/rospy/Overview/Initialization%20and%20Shutdown
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
        termination_reason = "Exception " + str(e)
        raise
    finally:
        node.cleanup()
        rospy.signal_shutdown(termination_reason)


if __name__ == "__main__":
    run_ros_loop(BlinkDistress())
