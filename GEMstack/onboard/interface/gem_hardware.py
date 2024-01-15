from .gem import *
import math

# ROS Headers
import rospy
from std_msgs.msg import String, Bool, Float32, Float64
from sensor_msgs.msg import PointCloud2
from novatel_gps_msgs.msg import NovatelPosition, NovatelXYZ, Inspva
from radar_msgs.msg import RadarTracks
from tf.transformations import euler_from_quaternion, quaternion_from_euler

# GEM PACMod Headers
from pacmod_msgs.msg import PositionWithSpeed, PacmodCmd, SystemRptFloat, VehicleSpeedRpt


class GEMHardwareInterface(GEMInterface):
    """Interface for interfacing with the physical GEM vehicle."""
    def __init__(self):
        GEMInterface.__init__(self)
        self.last_reading = GEMVehicleReading()
        self.last_reading.speed = 0.0
        self.last_reading.steering_wheel_angle = 0.0
        self.last_reading.accelerator_pedal_position = 0.0
        self.last_reading.brake_pedal_position = 0.0
        self.last_reading.gear = 0
        self.last_reading.left_turn_signal = False
        self.last_reading.right_turn_signal = False
        self.last_reading.horn_on = False
        self.last_reading.wiper_level = 0
        self.last_reading.headlights_on = False
        
        self.speed_sub  = rospy.Subscriber("/pacmod/parsed_tx/vehicle_speed_rpt", VehicleSpeedRpt, self.speed_callback)
        self.steer_sub = rospy.Subscriber("/pacmod/parsed_tx/steer_rpt", SystemRptFloat, self.steer_callback)
        self.gnss_sub = None
        self.imu_sub = None
        self.front_radar_sub = None
        self.lidar_sub = None
        self.stereo_sub = None

        # -------------------- PACMod setup --------------------
        # GEM vehicle enable
        self.enable_sub = rospy.Subscriber('/pacmod/as_tx/enable', Bool, self.pacmod_enable_callback)
        self.enable_pub = rospy.Publisher('/pacmod/as_rx/enable', Bool, queue_size=1)
        self.pacmod_enable = False

        # GEM vehicle gear control, neutral, forward and reverse, publish once
        self.gear_pub = rospy.Publisher('/pacmod/as_rx/shift_cmd', PacmodCmd, queue_size=1)
        self.gear_cmd = PacmodCmd()
        self.gear_cmd.ui16_cmd = 2 # SHIFT_NEUTRAL

        # GEM vehicle brake control
        self.brake_pub = rospy.Publisher('/pacmod/as_rx/brake_cmd', PacmodCmd, queue_size=1)
        self.brake_cmd = PacmodCmd()
        self.brake_cmd.enable = False
        self.brake_cmd.clear  = True
        self.brake_cmd.ignore = True

        # GEM vehicle forward motion control
        self.accel_pub = rospy.Publisher('/pacmod/as_rx/accel_cmd', PacmodCmd, queue_size=1)
        self.accel_cmd = PacmodCmd()
        self.accel_cmd.enable = False
        self.accel_cmd.clear  = True
        self.accel_cmd.ignore = True

        # GEM vehicle turn signal control
        self.turn_pub = rospy.Publisher('/pacmod/as_rx/turn_cmd', PacmodCmd, queue_size=1)
        self.turn_cmd = PacmodCmd()
        self.turn_cmd.ui16_cmd = 1 # None

        # GEM vechile steering wheel control
        self.steer_pub = rospy.Publisher('/pacmod/as_rx/steer_cmd', PositionWithSpeed, queue_size=1)
        self.steer_cmd = PositionWithSpeed()
        self.steer_cmd.angular_position = 0.0 # radians, -: clockwise, +: counter-clockwise
        self.steer_cmd.angular_velocity_limit = 2.0 # radians/second

        """TODO: other commands
        /pacmod/as_rx/headlight_cmd
        /pacmod/as_rx/horn_cmd
        /pacmod/as_rx/turn_cmd
        /pacmod/as_rx/wiper_cmd
        """

        #TODO: publish TwistStamped to /front_radar/front_radar/vehicle_motion to get better radar tracks

    def start(self):
        print("ENABLING PACMOD")
        enable_cmd = Bool()
        enable_cmd.data = True
        self.enable_pub.publish(enable_cmd)

    def speed_callback(self,msg : VehicleSpeedRpt):
        self.last_reading.speed = msg.vehicle_speed   # forward velocity in m/s

    def steer_callback(self, msg):
        self.last_reading.steering_wheel_angle = msg.output

    def get_reading(self) -> GEMVehicleReading:
        return self.last_reading

    def subscribe_sensor(self, name, callback):
        if name == 'gnss':
            self.gnss_sub = rospy.Subscriber("/novatel/inspva", Inspva, callback)
        elif name == 'lidar':
            self.lidar_sub = rospy.Subscriber("/lidar1/velodyne_points", PointCloud2, callback)
        elif name == 'front_radar':
            self.front_radar_sub = rospy.Subscriber("/front_radar/front_radar/radar_tracks", RadarTracks, callback)
    

    # PACMod enable callback function
    def pacmod_enable_callback(self, msg):
        if self.pacmod_enable == False and msg.data == True:
            print("PACMod enabled")
        elif self.pacmod_enable == True and msg.data == False:
            print("PACMod disabled")
        self.pacmod_enable = msg.data

    def send_first_command(self):
        # ---------- Enable PACMod ----------

        # enable forward gear
        self.gear_cmd.ui16_cmd = 3

        # enable brake
        self.brake_cmd.enable  = True
        self.brake_cmd.clear   = False
        self.brake_cmd.ignore  = False
        self.brake_cmd.f64_cmd = 1.0

        # enable gas 
        self.accel_cmd.enable  = True
        self.accel_cmd.clear   = False
        self.accel_cmd.ignore  = False
        self.accel_cmd.f64_cmd = 0.0

        self.gear_pub.publish(self.gear_cmd)
        self.turn_pub.publish(self.turn_cmd)
        self.brake_pub.publish(self.brake_cmd)
        self.accel_pub.publish(self.accel_cmd)


    # Start PACMod interface
    def send_command(self, command : GEMVehicleCommand):
        if command.left_turn_signal:
            self.turn_cmd.ui16_cmd = 2 # left turn signal
        elif command.right_turn_signal:
            self.turn_cmd.ui16_cmd = 0 # right turn signal
        else:
            self.turn_cmd.ui16_cmd = 1

        self.accel_cmd.f64_cmd = command.accelerator_pedal_position
        if command.brake_pedal_position > 0.0:
            self.accel_cmd.f64_cmd = 0.0
        self.brake_cmd.f64_cmd = command.brake_pedal_position
        self.steer_cmd.angular_position = command.steering_wheel_angle
        self.steer_cmd.angular_velocity_limit = command.steering_wheel_speed
        self.accel_pub.publish(self.accel_cmd)
        self.brake_pub.publish(self.brake_cmd)
        self.steer_pub.publish(self.steer_cmd)
        self.turn_pub.publish(self.turn_cmd)

        self.last_command = command
