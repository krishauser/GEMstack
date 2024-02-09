import rospy
from std_msgs.msg import String, Bool, Float32, Float64
from pacmod_msgs.msg import PositionWithSpeed, PacmodCmd, SystemRptInt, SystemRptFloat, VehicleSpeedRpt

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
        
        # Step 2
        accelSub = rospy.Subscriber("parsed_tx/accel_rpt", SystemRptFloat, self.accel_callback)
        speedSub = rospy.Subscriber("parsed_tx/vehicle_speed_rpt", VehicleSpeedRpt, self.speed_callback)
        
        # Step 3
        self.turn_cmd_pub = rospy.Publisher('/pacmod/as_rx/turn_cmd', PacmodCmd, queue_size=10)
        self.turn_cmd = PacmodCmd()
        self.turn_cmd.ui16_cmd = 1 # default: off
        self.count = 0

    def accel_callback(self, data):
        rospy.loginfo("Accel info from " + rospy.get_caller_id())
        rospy.loginfo("Enabled %d ", data.enabled)
        rospy.loginfo("Override active: %d ", data.override_active)
        rospy.loginfo("Command output fault: %d ", data.command_output_fault)
        rospy.loginfo("Command input fault: %d ", data.command_input_fault)
        rospy.loginfo("Output reported fault: %d ", data.output_reported_fault)
        rospy.loginfo("Pacmod fault: %d ", data.pacmod_fault)
        rospy.loginfo("Vehicle fault: %d ", data.vehicle_fault)

        rospy.loginfo("Manual input: %f ", data.manual_input)
        rospy.loginfo("Command: %f ", data.command)
        rospy.loginfo("Output: %f ", data.output)

    def speed_callback(self, data):
        rospy.loginfo("Speed info from " + rospy.get_caller_id())
        rospy.loginfo("Vehicle speed: %f", data.vehicle_speed)
        rospy.loginfo("Vehicle speed valid: %d", data.vehicle_speed_valid)
        rospy.loginfo("Raw bytes: %u %u", data.vehicle_speed_raw[0], data.vehicle_speed_raw[1])

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

        # turn_cmd.util6: 0: right; 1: turn off; 2: left
        # [left (2), right (0), off (1)]

        if self.count == 0:
            self.turn_cmd.ui16_cmd = 2 # left
        elif self.count == 1:
            self.turn_cmd.ui16_cmd = 0 # right
        else:
            self.turn_cmd.ui16_cmd = 1 # off
        self.count += 1
        self.count = self.count % 3
        self.turn_cmd_pub.publish(self.turn_cmd)
        
       
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
