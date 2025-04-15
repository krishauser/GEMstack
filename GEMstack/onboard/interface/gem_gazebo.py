from .gem import *
from ...utils import settings
import math

# ROS Headers
import rospy
from std_msgs.msg import String, Bool, Float32, Float64
from sensor_msgs.msg import Image, PointCloud2, Imu, NavSatFix
from sensor_msgs.msg import JointState  # For reading joint states from Gazebo
# Changed from AckermannDriveStamped
from ackermann_msgs.msg import AckermannDrive
from rosgraph_msgs.msg import Clock
from tf.transformations import euler_from_quaternion

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
        self.last_reading.gear = 0
        self.last_reading.left_turn_signal = False
        self.last_reading.right_turn_signal = False
        self.last_reading.horn_on = False
        self.last_reading.wiper_level = 0
        self.last_reading.headlights_on = False

        # Gazebo joint state subscriber for wheel positions/velocities
        self.joint_state_sub = rospy.Subscriber(
            "/joint_states", JointState, self.joint_state_callback)

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

        # Subscribe to IMU topic by default
        self.imu_sub = rospy.Subscriber("/imu", Imu, self.imu_callback)

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

    def joint_state_callback(self, msg: JointState):
        # Extract steering and speed from joint states
        # This implementation depends on the specific joint names in your Gazebo model
        # Typically find steering joints and wheel rotation joints
        try:
            # Match your simulation's joint names from rostopic list
            if 'left_steering_joint' in msg.name:  # Changed from front_left_steering_joint
                idx = msg.name.index('left_steering_joint')
                self.last_reading.steering_wheel_angle = msg.position[idx]
                # Added debug
                print(f"[JOINT] Steering angle: {msg.position[idx]:.3f} rad")

            # Update wheel joint names to match your simulation
            wheel_radius = 0.2
            rear_wheel_indices = []
            for name in ['left_rear_wheel_joint', 'right_rear_wheel_joint']:  # Updated names
                if name in msg.name:
                    rear_wheel_indices.append(msg.name.index(name))

            if rear_wheel_indices:
                # Average rear wheel velocities to get vehicle speed
                avg_wheel_vel = sum(
                    msg.velocity[i] for i in rear_wheel_indices) / len(rear_wheel_indices)
                self.last_reading.speed = avg_wheel_vel * \
                    wheel_radius  # Convert to linear velocity
                # Added debug
                print(
                    f"[JOINT] Wheel speed: {self.last_reading.speed:.2f} m/s")
        except (ValueError, IndexError) as e:
            rospy.logwarn(f"Error processing joint states: {e}")

    def imu_callback(self, msg: Imu):
        self.imu_data = msg

    def get_reading(self) -> GEMVehicleReading:
        return self.last_reading

    def subscribe_sensor(self, name, callback, type=None):
        if name == 'gnss':
            topic = self.ros_sensor_topics['gps']
            if type is not None and (type is not GNSSReading and type is not NavSatFix):
                raise ValueError("Gazebo GEM e2 only supports NavSatFix/GNSSReading for GNSS")
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
                    speed=self.last_reading.speed,
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

    def send_command(self, command: GEMVehicleCommand):
        # throttle rate at which we send commands
        t = self.time()
        if t < self.last_command_time + 1.0/self.max_send_rate:
            # skip command, Gazebo can't handle commands this fast
            return
        self.last_command_time = t

        # Create an AckermannDrive message
        self.ackermann_cmd.steering_angle = steer2front(
            command.steering_wheel_angle)

        # Calculate linear/speed from accelerator/brake
        # For simplicity, map 0-1 accelerator to 0-max_speed
        max_speed = 3.0  # m/s

        # In gazebo we use speed instead of acceleration
        desired_speed = 0.0
        if command.accelerator_pedal_position > 0.0:
            desired_speed = command.accelerator_pedal_position * max_speed

        # Apply brake (reduce speed)
        if command.brake_pedal_position > 0.0:
            brake_factor = 1.0 - command.brake_pedal_position
            desired_speed *= max(0.0, brake_factor)

        self.ackermann_cmd.speed = desired_speed

        # Set acceleration limit if needed
        acceleration_limit = 2.0  # m/s²
        self.ackermann_cmd.acceleration = acceleration_limit

        # Set jerk/steering_angle_velocity if needed
        self.ackermann_cmd.steering_angle_velocity = command.steering_wheel_speed

        # Print debug info
        print("**************************")
        print(
            f"Gazebo command: speed={desired_speed:.2f} m/s, steering={self.ackermann_cmd.steering_angle:.2f} rad")
        print("**************************")

        # Publish the command
        self.ackermann_pub.publish(self.ackermann_cmd)

        self.last_command = command
