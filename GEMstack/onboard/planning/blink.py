import rospy
import time
from std_msgs.msg import String, Bool, Float32, Float64
from pacmod_msgs.msg import PositionWithSpeed, PacmodCmd, SystemRptInt, SystemRptFloat, VehicleSpeedRpt

# For message format, see
# https://github.com/astuff/astuff_sensor_msgs/blob/3.3.0/pacmod_msgs/msg/PacmodCmd.msg

class BlinkDistress:
    """Your control loop code should go here.  You will use ROS and Pacmod messages
    to communicate with drive-by-wire system to control the vehicle's turn signals.
    """

    def callback(self,data):
        rospy.loginfo("RECEIVED DATA: %s", data)

    def __init__(self):
        self.turn_cmd = None
        self.turn_publisher = None
        # TODO: Initialize your publishers and subscribers here
        # When you create a subscriber to a /pacmod/parsed_tx/X topic, you will need to provide a callback function.
        # You will want this callback to be a BlinkDistress method, such as print_X(self, msg).  msg will have a
        # ROS message type, and you can find out what this is by either reading the documentation or running
        # "rostopic info /pacmod/parsed_tx/X" on the command line.
        self.turn_subscriber = rospy.Subscriber('/pacmod/parsed_tx/turn_rpt',SystemRptInt,self.callback)
        self.brake_subscriber = rospy.Subscriber('/pacmod/parsed_tx/brake_rpt',SystemRptFloat,self.callback)

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
    
    def update(self,signal):
        """Run in a loop"""
        # TODO: Implement your control loop here
        # You will need to publish a PacmodCmd() to /pacmod/as_rx/turn_cmd.  Read the documentation to see
        # what the data in the message indicates.
        if self.turn_cmd is None:
            self.turn_cmd = PacmodCmd()
        if self.turn_publisher is None:
            self.turn_publisher = rospy.Publisher('/pacmod/as_rx/turn_cmd',PacmodCmd,queue_size = 1)
        self.turn_cmd.ui16_cmd = signal
        self.turn_publisher.publish(self.turn_cmd)
        # self.turn_cmd.ui16_cmd = 2
        # self.turn_publisher.publish(self.turn_cmd)
        # time.sleep(2)
        # self.turn_cmd.ui16_cmd = 0
        # self.turn_publisher.publish(self.turn_cmd)
        # time.sleep(2)
        # self.turn_cmd.ui16_cmd = 1
        # self.turn_publisher.publish(self.turn_cmd)
        # time.sleep(2)


        
       
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
    signal_type = [2,0,1]
    cnt = 0
    node.initialize()
    rate = rospy.Rate(node.rate())
    termination_reason = "undetermined"
    try:
        while not rospy.is_shutdown() and not node.done() and node.healthy():
            node.update(signal_type[cnt%len(signal_type)])
            cnt+=1
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


