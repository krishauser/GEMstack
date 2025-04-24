from .gem import *
from ...utils import settings
import math
import time 

# ROS Headers
import rospy
from sensor_msgs.msg import Image, PointCloud2, Imu, NavSatFix
from geometry_msgs.msg import Vector3Stamped
from sensor_msgs.msg import JointState  # For reading joint states from Gazebo
# Changed from AckermannDriveStamped
from ackermann_msgs.msg import AckermannDrive
from rosgraph_msgs.msg import Clock
from tf.transformations import euler_from_quaternion


from ...state import ObjectPose,ObjectFrameEnum
from ...knowledge.vehicle.geometry import steer2front
from ...knowledge.vehicle.dynamics import pedal_positions_to_acceleration

# OpenCV and cv2 bridge
import cv2
import numpy as np
from ...utils import conversions
from ...mathutils import transforms


@dataclass
class GNSSReading:
    pose: ObjectPose
    speed: float
    status: str


class GEMGazeboInterface(GEMInterface):
    """Interface for connecting to the GEM e2 vehicle in Gazebo simulation."""

    def __init__(self):
        GEMInterface.__init__(self)
        self.max_send_rate = settings.get('vehicle.max_command_rate', 10.0)
        self.ros_sensor_topics = settings.get('vehicle.sensors.ros_topics')
        self.last_command_time = 0.0
        self.last_reading = GEMVehicleReading()
        self.last_reading.speed = 0.0
        self.last_reading.steering_wheel_angle = 0.0
        self.last_reading.accelerator_pedal_position = 0.0
        self.last_reading.brake_pedal_position = 0.0
        self.last_reading.gear = 1
        self.last_reading.left_turn_signal = False
        self.last_reading.right_turn_signal = False
        self.last_reading.horn_on = False
        self.last_reading.wiper_level = 0
        self.last_reading.headlights_on = False

      

        

        # IMU data subscriber
        self.imu_sub = None
        self.imu_data = None

        # GNSS data subscriber
        self.gnss_sub = None

        # Other sensors
        self.front_camera_sub = None
        self.front_depth_sub = None
        self.top_lidar_sub = None
        self.front_radar_sub = None

        self.faults = []

        # Gazebo vehicle control
        self.ackermann_pub = rospy.Publisher(
            '/ackermann_cmd', AckermannDrive, queue_size=1)
        self.ackermann_cmd = AckermannDrive()
        self.last_command = None  # Store the last command

        # Subscribe to IMU topic by default
        self.imu_sub = rospy.Subscriber("/imu", Imu, self.imu_callback)

        # Subscribe to GNSS Velocitu
        self.gnss_vel_sub = rospy.Subscriber("/gps/fix_velocity", Vector3Stamped, self.gnss_vel_callback)

        # Add clock subscription for simulation time
        self.sim_time = rospy.Time(0)
        self.clock_sub = rospy.Subscriber('/clock', Clock, self.clock_callback)

    def start(self):
        print("Starting GEM Gazebo Interface")

    def clock_callback(self, msg):
        self.sim_time = msg.clock

    def time(self):
        # Return Gazebo simulation time
        return self.sim_time.to_sec()

    def imu_callback(self, msg: Imu):
        self.imu_data = msg

    def gnss_vel_callback(self, msg):
        self.last_reading.speed = np.linalg.norm([msg.vector.x, msg.vector.y] )

    def get_reading(self) -> GEMVehicleReading:
        return self.last_reading
    

    def subscribe_sensor(self, name, callback, type=None):
        if name == 'gnss':
            topic = self.ros_sensor_topics['gps']
            # Fuse IMU orientation with GNSS position
            def gnss_callback_wrapper(gps_msg: NavSatFix):
                if self.imu_data is None:
                    return  # Wait for IMU data

                # Get orientation from IMU
                quaternion = (
                    self.imu_data.orientation.x,
                    self.imu_data.orientation.y,
                    self.imu_data.orientation.z,
                    self.imu_data.orientation.w
                )
                print(f"[IMU] Orientation: {quaternion}")
                roll, pitch, yaw = euler_from_quaternion(quaternion)

                # Transform yaw to correct frame - Gazebo typically uses ROS standard frame (x-forward)
                # while navigation uses x-east reference frame
                # Need to convert from Gazebo's frame to navigation heading, then to navigation yaw

                # Assuming Gazebo's yaw is 0 when facing east (ROS REP 103 convention)
                # Convert IMU's yaw to heading (CW from North), then to navigation yaw (CCW from East)
                # This handles the coordinate frame differences between Gazebo and the navigation frame
                # Negate yaw to convert from ROS to heading
                heading = transforms.yaw_to_heading(-yaw, degrees=False)
                navigation_yaw = transforms.heading_to_yaw(
                    heading, degrees=False)

                # Create fused pose with transformed yaw
                pose = ObjectPose(
                    frame=ObjectFrameEnum.GLOBAL,
                    t=gps_msg.header.stamp,
                    x=gps_msg.longitude,
                    y=gps_msg.latitude,
                    z=gps_msg.altitude,
                    roll=roll,
                    pitch=pitch,
                    yaw=navigation_yaw
                )

                # Create GNSS reading with fused data
                reading = GNSSReading(
                    pose=pose,
                    speed= self.last_reading.speed,
                    status='FIX' if gps_msg.status.status >= 0 else 'NO_FIX'
                )
                # Added debug
                print(
                    f"[GNSS] Raw coordinates: Lat={gps_msg.latitude:.6f}, Lon={gps_msg.longitude:.6f}")
                # Added debug
                print(
                    f"[GNSS-FUSED] Orientation: Roll={roll:.2f}, Pitch={pitch:.2f}, Yaw={yaw:.2f} rad")
                # Added debug
                print(f"[GNSS-FUSED] Speed: {self.last_reading.speed:.2f} m/s")

                callback(reading)

            self.gnss_sub = rospy.Subscriber(topic, NavSatFix, gnss_callback_wrapper)

        elif name == 'top_lidar':
            topic = self.ros_sensor_topics[name]
            if type is not None and (type is not PointCloud2 and type is not np.ndarray):
                raise ValueError("GEMGazeboInterface only supports PointCloud2 or numpy array for top lidar")
            if type is None or type is PointCloud2:
                self.top_lidar_sub = rospy.Subscriber(topic, PointCloud2, callback)
            else:
                def callback_with_numpy(msg: PointCloud2):
                    points = conversions.ros_PointCloud2_to_numpy(msg, want_rgb=False)
                    callback(points)
                self.top_lidar_sub = rospy.Subscriber(topic, PointCloud2, callback_with_numpy)

        elif name == 'imu':
            topic = self.ros_sensor_topics[name]
            if type is not None and type is not Imu:
                raise ValueError("GEMGazeboInterface only supports Imu for IMU data")
            self.imu_sub = rospy.Subscriber(topic, Imu, callback)

        elif name == 'front_camera':
            topic = self.ros_sensor_topics[name]
            if type is not None and (type is not Image and type is not cv2.Mat):
                raise ValueError("GEMGazeboInterface only supports Image or OpenCV for front camera")
            if type is None or type is Image:
                self.front_camera_sub = rospy.Subscriber(topic, Image, callback)
            else:
                def callback_with_cv2(msg: Image):
                    cv_image = conversions.ros_Image_to_cv2(msg, desired_encoding="bgr8")
                    callback(cv_image)
                self.front_camera_sub = rospy.Subscriber(topic, Image, callback_with_cv2)
        # Front depth sensor has not been added to gazebo yet.
        # This code is placeholder until we add front depth sensor.
        elif name == 'front_depth':
            topic = self.ros_sensor_topics[name]
            if type is not None and (type is not Image and type is not cv2.Mat):
                raise ValueError("GEMGazeboInterface only supports Image or OpenCV for front depth")
            if type is None or type is Image:
                self.front_depth_sub = rospy.Subscriber(topic, Image, callback)
            else:
                def callback_with_cv2(msg: Image):
                    cv_image = conversions.ros_Image_to_cv2(msg, desired_encoding="passthrough")
                    callback(cv_image)
                self.front_depth_sub = rospy.Subscriber(topic, Image, callback_with_cv2)

    def hardware_faults(self) -> List[str]:
        # In simulation, we don't have real hardware faults
        return self.faults

    def send_command(self, command : GEMVehicleCommand):
        # Throttle rate at which we send commands
        t = self.time()
        if t < self.last_command_time + 1.0/self.max_send_rate:
            # Skip command, similar to hardware interface
            return
        self.last_command_time = t
        
        # Get current speed
        v = self.last_reading.speed


        #update last reading
        self.last_reading.accelerator_pedal_position = command.accelerator_pedal_position
        self.last_reading.brake_pedal_position = command.brake_pedal_position
        self.last_reading.steering_wheel_angle = command.steering_wheel_angle
        
        # Convert pedal to acceleration
        accelerator_pedal_position = np.clip(command.accelerator_pedal_position, 0.0, 1.0)
        brake_pedal_position = np.clip(command.brake_pedal_position, 0.0, 1.0)
        
        # Zero out accelerator if brake is active (just like hardware interface)
        if brake_pedal_position > 0.0:
            accelerator_pedal_position = 0.0
            
        # Calculate acceleration from pedal positions
        acceleration = pedal_positions_to_acceleration(accelerator_pedal_position, brake_pedal_position, v, 0, 1)
        
        # Apply reasonable limits to acceleration
        max_accel = settings.get('vehicle.limits.max_acceleration', 1.0)
        max_decel = settings.get('vehicle.limits.max_deceleration', -2.0)
        acceleration = np.clip(acceleration, max_decel, max_accel)
        
        # Convert wheel angle to steering angle (front wheel angle)
        phides = steer2front(command.steering_wheel_angle)
        
        # Apply steering angle limits
        min_wheel_angle = settings.get('vehicle.geometry.min_wheel_angle', -0.6)
        max_wheel_angle = settings.get('vehicle.geometry.max_wheel_angle', 0.6)
        phides = np.clip(phides, min_wheel_angle, max_wheel_angle)
        
        # Calculate target speed based on acceleration
        # Don't use infinite speed, instead calculate a reasonable target speed
        current_speed = v
        target_speed = current_speed
        
        if acceleration > 0:
            # Accelerating - set target speed to current speed plus some increment
            # This is more realistic than infinite speed
            max_speed = settings.get('vehicle.limits.max_speed', 10.0)
            target_speed = min(current_speed + acceleration * 0.5, max_speed)
        elif acceleration < 0:
            # Braking - set target speed to zero if deceleration is significant
            if brake_pedal_position > 0.1:
                target_speed = 0.0
        
        # Create and publish drive message
        msg = AckermannDrive()
        msg.acceleration = acceleration
        msg.speed = target_speed
        msg.steering_angle = phides
        msg.steering_angle_velocity = command.steering_wheel_speed  # Respect steering velocity limit
        
        # Debug output
        print(f"[ACKERMANN] Speed: {msg.speed:.2f}, Accel: {msg.acceleration:.2f}, Steer: {msg.steering_angle:.2f}")
        
        self.ackermann_pub.publish(msg)
        self.last_command = command