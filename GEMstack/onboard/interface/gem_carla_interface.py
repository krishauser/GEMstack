from .gem import GEMInterface
from .gem import GEMVehicleCommand
from .gem import GEMVehicleReading
from .gem import ObjectFrameEnum
from .gem import ObjectPose
from .gem import List
from ...utils import settings
import math
import time
# ROS Headers
import rospy
from std_msgs.msg import Bool
from sensor_msgs.msg import Image,PointCloud2
try:
    from novatel_gps_msgs.msg import Inspva
except ImportError:
    pass
try:
    from septentrio_gnss_driver.msg import INSNavGeod
except ImportError:
    pass

from radar_msgs.msg import RadarTracks
from tf.transformations import euler_from_quaternion, quaternion_from_euler

# GEM PACMod Headers
from pacmod_msgs.msg import PositionWithSpeed, PacmodCmd, SystemRptFloat, VehicleSpeedRpt, GlobalRpt

# OpenCV and cv2 bridge
import cv2
import numpy as np
from ...utils import conversions

from .gnss_reading import GNSSReading

class GEMCarlaHardwareInterface(GEMInterface):
    """Interface for connnecting to the carla vehicle."""
    def __init__(self):
        GEMInterface.__init__(self)
        self.max_send_rate = settings.get('carla_vehicle.max_command_rate',10.0)
        self.ros_sensor_topics = settings.get('carla_vehicle.sensors.ros_topics')
        self.last_command_time = 0.0
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
        
        self.speed_sub  = rospy.Subscriber("/carla/parsed_tx/vehicle_speed_rpt", VehicleSpeedRpt, self.speed_callback)
        self.steer_sub = rospy.Subscriber("/carla/parsed_tx/steer_rpt", SystemRptFloat, self.steer_callback)
        self.gnss_sub = None
        self.imu_sub = None
        self.front_radar_sub = None
        self.front_camera_sub = None
        self.front_depth_sub = None
        self.top_lidar_sub = None
        self.stereo_sub = None
        self.faults = []

        # -------------------- PACMod setup --------------------
        # GEM vehicle enable
        self.pacmod_enable = False

        # GEM vehicle gear control, neutral, forward and reverse, publish once
        self.gear_pub = rospy.Publisher('/carla/as_rx/shift_cmd', PacmodCmd, queue_size=1)
        self.gear_cmd = PacmodCmd()
        self.gear_cmd.enable = True
        self.gear_cmd.ui16_cmd = PacmodCmd.SHIFT_NEUTRAL 

        # GEM vehicle brake control
        self.brake_pub = rospy.Publisher('/carla/as_rx/brake_cmd', PacmodCmd, queue_size=1)
        self.brake_cmd = PacmodCmd()
        self.brake_cmd.enable = False
        self.brake_cmd.clear  = True
        self.brake_cmd.ignore = True

        # GEM vehicle forward motion control
        self.accel_pub = rospy.Publisher('/carla/as_rx/accel_cmd', PacmodCmd, queue_size=1)
        self.accel_cmd = PacmodCmd()
        self.accel_cmd.enable = False
        self.accel_cmd.clear  = True
        self.accel_cmd.ignore = True

        # GEM vehicle turn signal control
        self.turn_pub = rospy.Publisher('/carla/as_rx/turn_cmd', PacmodCmd, queue_size=1)
        self.turn_cmd = PacmodCmd()
        self.turn_cmd.ui16_cmd = 1 # None

        # GEM vechile steering wheel control
        self.steer_pub = rospy.Publisher('/carla/as_rx/steer_cmd', PositionWithSpeed, queue_size=1)
        self.steer_cmd = PositionWithSpeed()
        self.steer_cmd.angular_position = 0.0 # radians, -: clockwise, +: counter-clockwise
        self.steer_cmd.angular_velocity_limit = 2.0 # radians/second

        """TODO: other commands
        /pacmod/as_rx/headlight_cmd
        /pacmod/as_rx/horn_cmd
        /pacmod/as_rx/wiper_cmd
        """
        
        #subscribers should go last because the callback might be called before the object is initialized


    def start(self):
        if not settings.get('vehicle.enable_through_joystick',True):
            print("ENABLING PACMOD")
            enable_cmd = Bool()
            enable_cmd.data = True
            self.enable_pub.publish(enable_cmd)
            #this doesn't seem to work super well, need to send enable command multiple times
    
    def time(self):
        seconds = rospy.get_time()
        return seconds

    def speed_callback(self,msg : VehicleSpeedRpt):
        self.last_reading.speed = msg.vehicle_speed   # forward velocity in m/s

    def steer_callback(self, msg):
        self.last_reading.steering_wheel_angle = msg.output

    def get_reading(self) -> GEMVehicleReading:
        return self.last_reading

    def subscribe_sensor(self, name, callback, type = None):
        if name == 'gnss':
            topic = self.ros_sensor_topics[name]
            if topic.endswith('gnss'):
                if type is not None and (type is not Inspva and type is not GNSSReading):
                    raise ValueError("GEMHardwareInterface GEM e2 only supports Inspva/GNSSReading for GNSS")
                if type is Inspva:
                    self.gnss_sub = rospy.Subscriber(topic, Inspva, callback)
                else:
                    def callback_with_gnss_reading(inspva_msg: Inspva):
                        pose = ObjectPose(ObjectFrameEnum.GLOBAL,
                                    x=inspva_msg.longitude,
                                    y=inspva_msg.latitude,
                                    z=inspva_msg.height,
                                    yaw=math.radians(inspva_msg.azimuth),  #heading from north in degrees
                                    roll=math.radians(inspva_msg.roll),
                                    pitch=math.radians(inspva_msg.pitch),
                                    t = time.time())
                        speed = np.sqrt(inspva_msg.east_velocity**2 + inspva_msg.north_velocity**2)
                        callback(GNSSReading(pose,speed,inspva_msg.status))
                    self.gnss_sub = rospy.Subscriber(topic, Inspva, callback_with_gnss_reading)
            else:
                #assume it's septentrio
                if type is not None and (type is not INSNavGeod and type is not GNSSReading):
                    raise ValueError("GEMHardwareInterface GEM e4 only supports INSNavGeod/GNSSReading for GNSS")
                if type is INSNavGeod:
                    self.gnss_sub = rospy.Subscriber(topic, INSNavGeod, callback)
                else:
                    def callback_with_gnss_reading(msg: INSNavGeod):
                        pose = ObjectPose(ObjectFrameEnum.GLOBAL,
                                    x=msg.longitude,
                                    y=msg.latitude,
                                    z=msg.height,
                                    yaw=math.radians(msg.heading),  #heading from north in degrees (TODO: maybe?? check this)
                                    roll=math.radians(msg.roll),
                                    pitch=math.radians(msg.pitch),
                                    )
                        speed = np.sqrt(msg.ve**2 + msg.vn**2)
                        callback(GNSSReading(pose,speed,('error' if msg.error else 'ok')))
                    self.gnss_sub = rospy.Subscriber(topic, Inspva, callback_with_gnss_reading)
        elif name == 'top_lidar':
            topic = self.ros_sensor_topics[name]
            if type is not None and (type is not PointCloud2 and type is not np.ndarray):
                raise ValueError("GEMHardwareInterface only supports PointCloud2 or numpy array for top lidar")
            if type is None or type is PointCloud2:
                self.top_lidar_sub = rospy.Subscriber(topic, PointCloud2, callback)
            else:
                def callback_with_numpy(msg : Image):
                    #print("received image with size",msg.width,msg.height,"encoding",msg.encoding)                    
                    points = conversions.ros_PointCloud2_to_numpy(msg, want_rgb=False)
                    callback(points)
                self.top_lidar_sub = rospy.Subscriber(topic, PointCloud2, callback_with_numpy)
        # elif name == 'front_radar':
        #     if type is not None and type is not RadarTracks:
        #         raise ValueError("GEMHardwareInterface only supports RadarTracks for front radar")
        #     self.front_radar_sub = rospy.Subscriber("/front_radar/front_radar/radar_tracks", RadarTracks, callback)
        elif name == 'front_camera':
            topic = self.ros_sensor_topics[name]
            if type is not None and (type is not Image and type is not cv2.Mat):
                raise ValueError("GEMHardwareInterface only supports Image or OpenCV for front camera")
            if type is None or type is Image:
                self.front_camera_sub = rospy.Subscriber(topic, Image, callback)
            else:
                def callback_with_cv2(msg : Image):
                    #print("received image with size",msg.width,msg.height,"encoding",msg.encoding)                    
                    cv_image = conversions.ros_Image_to_cv2(msg, desired_encoding="8UC4")
                    callback(cv_image[:,:,:-1])
                self.front_camera_sub = rospy.Subscriber(topic, Image, callback_with_cv2)
        elif name == 'front_depth':
            topic = self.ros_sensor_topics[name]
            if type is not None and (type is not Image and type is not cv2.Mat):
                raise ValueError("GEMHardwareInterface only supports Image or OpenCV for front depth")
            if type is None or type is Image:
                self.front_depth_sub = rospy.Subscriber(topic, Image, callback)
            else:
                def callback_with_cv2(msg : Image):
                    #print("received image with size",msg.width,msg.height,"encoding",msg.encoding)                    
                    cv_image = conversions.ros_Image_to_cv2(msg, desired_encoding="passthrough")
                    callback(cv_image)
                self.front_depth_sub = rospy.Subscriber(topic, Image, callback_with_cv2)


    # PACMod enable callback function
    def pacmod_enable_callback(self, msg):
        if self.pacmod_enable == False and msg.data == True:
            print("PACMod enabled, enabling gear, brake, accel, steer, and turn")
            self.send_first_command()
        elif self.pacmod_enable == True and msg.data == False:
            print("PACMod disabled")
        self.pacmod_enable = msg.data

    def hardware_faults(self) -> List[str]:
        if self.pacmod_enable == False:
            return self.faults + ["disengaged"]
        return self.faults

    def send_first_command(self):
        # ---------- Enable PACMod ----------

        # enable forward gear
        self.gear_cmd.enable = True
        self.gear_cmd.ui16_cmd = PacmodCmd.SHIFT_FORWARD
        #helps debug whether gear command is being sent since you'll hear the backup beep
        #self.gear_cmd.ui16_cmd = PacmodCmd.SHIFT_REVERSE

        # enable brake
        self.brake_cmd.enable  = True
        self.brake_cmd.clear   = False
        self.brake_cmd.ignore  = False
        self.brake_cmd.f64_cmd = 0.0

        # enable gas 
        self.accel_cmd.enable  = True
        self.accel_cmd.clear   = False
        self.accel_cmd.ignore  = False
        self.accel_cmd.f64_cmd = 0.0

        self.gear_pub.publish(self.gear_cmd)
        self.turn_pub.publish(self.turn_cmd)
        self.brake_pub.publish(self.brake_cmd)
        self.accel_pub.publish(self.accel_cmd)
        self.last_command_time = self.time()

    def send_command(self, command : GEMVehicleCommand):
        #throttle rate at which we send commands
        t = self.time()
        if t < self.last_command_time + 1.0/self.max_send_rate:
            #skip command, PACMod can't handle commands this fast
            return
        self.last_command_time = t
        
        if command.left_turn_signal and command.right_turn_signal:
            self.turn_cmd.ui16_cmd = PacmodCmd.TURN_HAZARDS
        elif command.left_turn_signal:
            self.turn_cmd.ui16_cmd = PacmodCmd.TURN_LEFT 
        elif command.right_turn_signal:
            self.turn_cmd.ui16_cmd = PacmodCmd.TURN_RIGHT
        else:
            self.turn_cmd.ui16_cmd = PacmodCmd.TURN_NONE

        self.accel_cmd.f64_cmd = command.accelerator_pedal_position
        if command.brake_pedal_position > 0.0:
            self.accel_cmd.f64_cmd = 0.0
        self.brake_cmd.f64_cmd = command.brake_pedal_position
        self.steer_cmd.angular_position = command.steering_wheel_angle
        self.steer_cmd.angular_velocity_limit = command.steering_wheel_speed
        print("**************************")
        print("Steer cmd angular position {} velocity limit {}".format(self.steer_cmd.angular_position,self.steer_cmd.angular_velocity_limit))
        print("Accel pedal position {} brake position {}".format(self.accel_cmd.f64_cmd,self.brake_cmd.f64_cmd))
        maxacc = settings.get('vehicle.limits.max_accelerator_pedal')
        maxbrake = settings.get('vehicle.limits.max_brake_pedal')
        if self.accel_cmd.f64_cmd > maxacc:
            print("Warning: commanded acceleration exceeded accel pedal limit")
            self.accel_cmd.f64_cmd = maxacc
        if self.brake_cmd.f64_cmd > maxbrake:
            print("Warning: commanded braking exceeded brake pedal limit")
            self.brake_cmd.f64_cmd = maxbrake
        print("**************************")

        self.brake_cmd.enable  = True
        self.brake_cmd.clear   = False
        self.brake_cmd.ignore  = False

        self.accel_cmd.enable  = True
        self.accel_cmd.clear   = False
        self.accel_cmd.ignore  = False
        
        self.gear_cmd.ui16_cmd = PacmodCmd.SHIFT_FORWARD
        self.gear_cmd.enable = True
        self.gear_pub.publish(self.gear_cmd)
        self.accel_pub.publish(self.accel_cmd)
        self.brake_pub.publish(self.brake_cmd)
        self.steer_pub.publish(self.steer_cmd)
        self.turn_pub.publish(self.turn_cmd)

        self.last_command = command