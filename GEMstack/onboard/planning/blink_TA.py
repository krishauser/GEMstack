import rospy
import time
from std_msgs.msg import String, Bool, Float32, Float64
from pacmod_msgs.msg import PositionWithSpeed, PacmodCmd, SystemRptInt, SystemRptFloat, VehicleSpeedRpt

class BlinkDistress:
    def __init__(self):
        self.pacmod_enable = False
        self.enable_pub = rospy.Publisher('/pacmod/as_rx/enable', Bool, queue_size=1)
        self.enable_sub = rospy.Subscriber('/pacmod/as_tx/enable', Bool, self.pacmod_enable_callback)
        self.turn_pub = rospy.Publisher('/pacmod/as_rx/turn_cmd', PacmodCmd, queue_size=1)
        self.turn_cmd = PacmodCmd()
        self.turn_cmd.ui16_cmd = 2 # None
        self.num_blinks = -1
        self.pattern = [0,2,1]

    def rate(self):
        """Requested update frequency, in Hz"""
        return 0.5

    def initialize(self):
        """Run first"""
        pass

    def cleanup(self):
        """Run last"""
        print("DISABLING PACMOD")
        enable_cmd = Bool()
        enable_cmd.data = False
        self.enable_pub.publish(enable_cmd)
        rospy.sleep(0.5)
    
    # PACMod enable callback function
    def pacmod_enable_callback(self, msg):
        if self.pacmod_enable == False and msg.data == True:
            print("PACMod enabled")
        elif self.pacmod_enable == True and msg.data == False:
            print("PACMod disabled")
        self.pacmod_enable = msg.data

    def update(self):
        """Run in a loop"""
        
        if not self.pacmod_enable:
            if self.num_blinks < 0:
                print("ENABLING PACMOD")
                enable_cmd = Bool()
                enable_cmd.data = True
                self.enable_pub.publish(enable_cmd)
            return

        self.num_blinks += 1
        self.turn_cmd.ui16_cmd = self.pattern[self.num_blinks%len(self.pattern)]

        print("Blink",self.num_blinks,"command",self.turn_cmd.ui16_cmd)
        self.turn_pub.publish(self.turn_cmd)
       
    def healthy(self):
        """Returns True if the element is in a stable state."""
        return True
        
    def done(self):
        """Return True if you want to exit."""
        return False


def run_ros_loop(node):
    """Executes the event loop of a node using ROS.  `node` should support
    rate(), initialize(), cleanup(), update(), healthy(), and done().
    
    You should not modify this code.
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