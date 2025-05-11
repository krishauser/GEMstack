from GEMstack.state.obstacle import ObstacleMaterialEnum, ObstacleState, ObstacleStateEnum
from .gem import *
from ...utils import settings
import math
import time

# ROS Headers
import rospy
from sensor_msgs.msg import Image, PointCloud2, Imu, NavSatFix
from septentrio_gnss_driver.msg import INSNavGeod
from geometry_msgs.msg import Vector3Stamped
from sensor_msgs.msg import JointState  # For reading joint states from Gazebo
# Changed from AckermannDriveStamped
from ackermann_msgs.msg import AckermannDrive
from rosgraph_msgs.msg import Clock
from gazebo_msgs.msg import ModelStates
from tf.transformations import euler_from_quaternion


from ...state import ObjectPose,ObjectFrameEnum
from ...knowledge.vehicle.geometry import steer2front
from ...knowledge.vehicle.dynamics import pedal_positions_to_acceleration
from ...state import AgentState, AgentEnum, AgentActivityEnum

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


# Agent dimensions similar to what's in gem_simulator.py
AGENT_DIMENSIONS = {
    'pedestrian' : (0.5,0.5,1.6),
    'bicyclist' : (1.8,0.5,1.6),
    'car' : (4.0,2.5,1.4),
    'medium_truck': (6.0,2.5,3.0),
    'large_truck': (10.0,2.5,3.5)
}

# Map model prefixes to agent types
MODEL_PREFIX_TO_AGENT_TYPE = {
    'pedestrian': 'pedestrian',
    'person': 'pedestrian',
    'bicycle': 'bicyclist',
    'bike': 'bicyclist',
    'car': 'car',
    'vehicle': 'car',
    'truck': 'medium_truck',
    'large_truck': 'large_truck',
    'cone': 'traffic_cone'
}

# Map model prefixes to obstacle types
MODEL_PREFIX_TO_OBSTACLE_TYPE = {
    'cone': 'traffic_cone'
}

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





        # GNSS data subscriber
        self.gnss_sub = None

        # Other sensors
        self.front_camera_sub = None
        self.front_depth_sub = None
        self.top_lidar_sub = None
        self.front_radar_sub = None

        # Agent detection
        self.model_states_sub = None
        self.tracked_model_prefixes = settings.get('simulator.agent_tracker.model_prefixes', 
                                                 ['pedestrian', 'bicycle', 'car'])
        self.tracked_obstacle_prefixes = settings.get('simulator.obstacle_tracker.model_prefixes', 
                                                 ['cone'])
        self.agent_detector_callback = None
        self.obstacle_detector_callback = None
        self.last_agent_positions = {}
        self.last_agent_velocities = {}
        self.last_model_states_time = 0.0
        self.agent_detection_rate = settings.get('simulator.agent_tracker.rate', 10.0)  # Hz
        self.obstacle_detection_rate = settings.get('simulator.obstacle_tracker.rate', 50.0)  # Hz
        
        # Frame transformation variables
        self.start_pose_abs = None  # Initial vehicle pose in GLOBAL frame
        self.vehicle_model_pose = None  # Current vehicle pose from model_states
        self.vehicle_gps_pose = None  # Current vehicle pose from GPS
        self.t_start = None  # Start time
        
        # Stable transformation variables
        self.transform_initialized = False
        self.initial_vehicle_model_pose = None  # Vehicle pose in model_states at start
        
        self.faults = []

        # Gazebo vehicle control
        self.ackermann_pub = rospy.Publisher(
            '/ackermann_cmd', AckermannDrive, queue_size=1)
        self.ackermann_cmd = AckermannDrive()
        self.last_command = None  # Store the last command

        # Add clock subscription for simulation time
        self.sim_time = rospy.Time(0)
        self.clock_sub = rospy.Subscriber('/clock', Clock, self.clock_callback)

        # Subscribe to model states for agent detection
        self.model_states_sub = rospy.Subscriber('/gazebo/model_states', ModelStates, self.model_states_callback)

    def start(self):
        print("Starting GEM Gazebo Interface")

    def clock_callback(self, msg):
        self.sim_time = msg.clock

    def time(self):
        # Return Gazebo simulation time
        return self.sim_time.to_sec()

    def get_reading(self) -> GEMVehicleReading:
        return self.last_reading

    def model_states_callback(self, msg: ModelStates):
        current_time = self.time()
        
        # Check if we should process this update (rate limiting)
        if ((current_time - self.last_model_states_time < 1.0/self.agent_detection_rate) and (current_time - self.last_model_states_time < 1.0/self.obstacle_detection_rate)):
            return
            
        # Calculate time delta since last update
        dt = current_time - self.last_model_states_time
        self.last_model_states_time = current_time
        
        # Skip if no callback is registered
        if self.agent_detector_callback is None and self.obstacle_detector_callback is None:
            return
        
        # Find vehicle in model states
        vehicle_idx = -1
        for i, name in enumerate(msg.name):
            if name.lower() in ['gem_e4', 'gem_e2']:
                vehicle_idx = i
                break
                
        # If vehicle not found, cannot proceed
        if vehicle_idx < 0:
            return
            
        # Get vehicle position and orientation from model states
        vehicle_pos = msg.pose[vehicle_idx].position
        vehicle_ori = msg.pose[vehicle_idx].orientation
        quaternion = (vehicle_ori.x, vehicle_ori.y, vehicle_ori.z, vehicle_ori.w)
        _, _, vehicle_yaw = euler_from_quaternion(quaternion)
        
        # Create vehicle model pose in ABSOLUTE_CARTESIAN frame (Gazebo's native frame)
        self.vehicle_model_pose = ObjectPose(
            frame=ObjectFrameEnum.ABSOLUTE_CARTESIAN,
            t=current_time,
            x=vehicle_pos.x,
            y=vehicle_pos.y,
            z=vehicle_pos.z,
            yaw=vehicle_yaw
        )
        
        # Initialize stable transformation when we have both GPS and model data
        if not self.transform_initialized and self.vehicle_gps_pose is not None:
            # Initialize start pose and transformation data
            self.start_pose_abs = self.vehicle_gps_pose
            self.initial_vehicle_model_pose = self.vehicle_model_pose
            self.t_start = current_time
            self.transform_initialized = True
            
            print("STABLE TRANSFORMATION INITIALIZED:")
            print(f"  GPS position: ({self.start_pose_abs.x:.4f}, {self.start_pose_abs.y:.4f}, {self.start_pose_abs.z:.4f})")
            print(f"  Model position: ({self.initial_vehicle_model_pose.x:.4f}, {self.initial_vehicle_model_pose.y:.4f}, {self.initial_vehicle_model_pose.z:.4f})")
            print(f"  GPS orientation: {self.start_pose_abs.yaw:.4f} radians")
            print(f"  Model orientation: {self.initial_vehicle_model_pose.yaw:.4f} radians")
            
        # Process all models except the vehicle itself
        for i, model_name in enumerate(msg.name):
            # Skip the vehicle model itself
            if i == vehicle_idx:
                continue
                
            # Check if this model should be tracked as an agent
            agent_type = None
            for prefix in self.tracked_model_prefixes:
                if model_name.lower().startswith(prefix.lower()):
                    for key, value in MODEL_PREFIX_TO_AGENT_TYPE.items():
                        if prefix.lower().startswith(key.lower()):
                            agent_type = value
                            break
                    break

            obstacle_type = None
            for prefix in self.tracked_obstacle_prefixes:
                if model_name.lower().startswith(prefix.lower()):
                    for key, value in MODEL_PREFIX_TO_OBSTACLE_TYPE.items():
                        if prefix.lower().startswith(key.lower()):
                            obstacle_type = value
                            break
                    break
                    
            if agent_type is None and obstacle_type is None:
                continue  # Not an agent we're tracking

            # Get position and orientation from model states
            position = msg.pose[i].position
            orientation = msg.pose[i].orientation
            
            # Get velocity from twist
            linear_vel = msg.twist[i].linear
            angular_vel = msg.twist[i].angular
            
            # Convert orientation quaternion to euler angles
            quaternion = (orientation.x, orientation.y, orientation.z, orientation.w)
            roll, pitch, yaw = euler_from_quaternion(quaternion)
            # Create agent pose in ABSOLUTE_CARTESIAN frame (Gazebo's native frame)
            agent_global_pose = ObjectPose(
                frame=ObjectFrameEnum.ABSOLUTE_CARTESIAN,
                t=current_time,
                x=position.x,
                y=position.y,
                z=position.z,
                roll=roll,
                pitch=pitch,
                yaw=yaw
            )
            
            # Transform agent pose to START frame using stable transformation
            agent_pose = None
            if self.transform_initialized:
                # Calculate agent position relative to the *initial* vehicle position (from when START frame was established)
                rel_x = position.x - self.initial_vehicle_model_pose.x
                rel_y = position.y - self.initial_vehicle_model_pose.y
                rel_z = position.z - self.initial_vehicle_model_pose.z
                
                # Rotate by the *initial* vehicle orientation
                cos_yaw = math.cos(-self.initial_vehicle_model_pose.yaw)
                sin_yaw = math.sin(-self.initial_vehicle_model_pose.yaw)
                rot_x = rel_x * cos_yaw - rel_y * sin_yaw
                rot_y = rel_x * sin_yaw + rel_y * cos_yaw
                
                # Adjust yaw relative to *initial* vehicle orientation
                rel_yaw = yaw - self.initial_vehicle_model_pose.yaw
                
                # Create the pose in START frame using the stable transformation
                agent_pose = ObjectPose(
                    frame=ObjectFrameEnum.START,
                    t=current_time - self.t_start if self.t_start is not None else 0,
                    x=rot_x,
                    y=rot_y,
                    z=rel_z,
                    roll=roll,
                    pitch=pitch,
                    yaw=rel_yaw
                )
            else:
                # If transformation not initialized yet, just use the model_states pose
                agent_pose = agent_global_pose
            
            if agent_type is not None:
                # Calculate velocity manually if twist data is zero or missing
                velocity = (linear_vel.x, linear_vel.y, linear_vel.z)
                velocity_is_zero = abs(linear_vel.x) < 1e-6 and abs(linear_vel.y) < 1e-6 and abs(linear_vel.z) < 1e-6
                
                if velocity_is_zero and model_name in self.last_agent_positions and dt > 0:
                    # Calculate velocity from position difference
                    prev_pos = self.last_agent_positions[model_name]
                    dx = position.x - prev_pos[0]
                    dy = position.y - prev_pos[1]
                    dz = position.z - prev_pos[2]
                    
                    # Calculate velocity (position change / time)
                    calculated_vel = (dx/dt, dy/dt, dz/dt)
                    
                    # Apply some smoothing with the previous velocity if available
                    if model_name in self.last_agent_velocities:
                        prev_vel = self.last_agent_velocities[model_name]
                        # Apply exponential smoothing (0.7 current + 0.3 previous)
                        velocity = (
                            0.7 * calculated_vel[0] + 0.3 * prev_vel[0],
                            0.7 * calculated_vel[1] + 0.3 * prev_vel[1],
                            0.7 * calculated_vel[2] + 0.3 * prev_vel[2]
                        )
                    else:
                        velocity = calculated_vel
                
                # Determine activity state based on velocity magnitude
                velocity_magnitude = np.linalg.norm(velocity)
                if velocity_magnitude < 0.1:
                    activity = AgentActivityEnum.STOPPED
                elif velocity_magnitude > 5.0:  # Arbitrary threshold for "fast"
                    activity = AgentActivityEnum.FAST
                else:
                    activity = AgentActivityEnum.MOVING
                    
                # Get agent dimensions
                dimensions = AGENT_DIMENSIONS.get(agent_type, (1.0, 1.0, 1.0))  # Default if unknown
            
                # Create agent state
                agent_state = AgentState(
                    pose=agent_pose,  # Using START frame pose
                    dimensions=dimensions,
                    outline=None,
                    type=getattr(AgentEnum, agent_type.upper()),
                    activity=activity,
                    velocity=velocity,
                    yaw_rate=angular_vel.z
                )
                
                # Store current position for next velocity calculation (using raw positions)
                self.last_agent_positions[model_name] = (position.x, position.y, position.z)
                self.last_agent_velocities[model_name] = velocity
                # Call the callback with the agent state
                self.agent_detector_callback(model_name, agent_state)

            if obstacle_type is not None:
                # Create obstacle state 
                obstacle_state = ObstacleState(
                    dimensions=(0,0,0),
                    outline=None,
                    pose=agent_pose,
                    type=getattr(ObstacleMaterialEnum, obstacle_type.upper()),
                    activity=ObstacleStateEnum.STANDING,
                )
                self.obstacle_detector_callback(model_name, obstacle_state)

    def subscribe_sensor(self, name, callback, type=None):
        if name == 'gnss':
            topic = self.ros_sensor_topics['gnss']
            def gnss_callback_wrapper(gnss_msg: INSNavGeod):
                roll, pitch, yaw = gnss_msg.roll, gnss_msg.pitch, gnss_msg.heading
                # Convert from degrees to radians
                roll, pitch, yaw = math.radians(roll), math.radians(pitch), math.radians(yaw)

                # Transform yaw to correct frame - Gazebo typically uses ROS standard frame (x-forward)
                # while navigation uses x-east reference frame
                # Need to convert from Gazebo's frame to navigation heading, then to navigation yaw

                # Assuming Gazebo's yaw is 0 when facing east (ROS REP 103 convention)
                # Convert IMU's yaw to heading (CW from North), then to navigation yaw (CCW from East)
                # This handles the coordinate frame differences between Gazebo and the navigation frame
                # Negate yaw to convert from ROS to heading
                heading = transforms.yaw_to_heading(-yaw - np.pi/2, degrees=False)
                navigation_yaw = transforms.heading_to_yaw(
                    heading, degrees=False)

                # Create fused pose with transformed yaw
                pose = ObjectPose(
                    frame=ObjectFrameEnum.GLOBAL,
                    t=gnss_msg.header.stamp,
                    x=gnss_msg.longitude,
                    y=gnss_msg.latitude,
                    z=gnss_msg.height,
                    roll=roll,
                    pitch=pitch,
                    yaw=navigation_yaw
                )
                
                # Save the vehicle's GPS pose for coordinate transformation
                self.vehicle_gps_pose = pose

                # Calculate speed from GNSS
                self.last_reading.speed = np.linalg.norm([gnss_msg.ve, gnss_msg.vn])

                # Create GNSS reading with fused data
                reading = GNSSReading(
                    pose=pose,
                    speed=self.last_reading.speed,
                    status='error' if gnss_msg.error else 'ok'
                )
                # Added debug
                print(
                    f"[GNSS] Raw coordinates: Lat={gnss_msg.latitude:.6f}, Lon={gnss_msg.longitude:.6f}")
                # Added debug
                print(
                    f"[GNSS-FUSED] Orientation: Roll={roll:.2f}, Pitch={pitch:.2f}, Yaw={yaw:.2f} rad")
                # Added debug
                print(f"[GNSS-FUSED] Speed: {self.last_reading.speed:.2f} m/s")

                callback(reading)

            self.gnss_sub = rospy.Subscriber(topic, INSNavGeod, gnss_callback_wrapper)
        
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
        
        elif name == 'agent_detector':
            if type is not None and type is not AgentState:
                raise ValueError("GEMGazeboInterface only supports AgentState for agent_detector")
            self.agent_detector_callback = callback

        elif name == 'obstacle_detector':
            if type is not None and type is not ObstacleState:
                raise ValueError("GEMGazeboInterface only supports ObstacleState for obstacle_detector")
            self.obstacle_detector_callback = callback

    def hardware_faults(self) -> List[str]:
        # In simulation, we don't have real hardware faults
        return self.faults

    def sensors(self):
        # Add agent_detector to the list of available sensors
        return super().sensors() + ['agent_detector']

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
        print("acceleration before", acceleration)
        # Apply reasonable limits to acceleration
        max_accel = settings.get('vehicle.limits.max_acceleration', 1.0)
        max_decel = -1 * settings.get('vehicle.limits.max_deceleration', 2.0) # cuz ackermann expects neg but pure pursiut wants positive decel val
        print("max_accel", max_accel)
        print("max_decel", max_decel)
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
        print("acceleration ", acceleration)
        if acceleration > 0:
            # Accelerating - set target speed to current speed plus some increment
            # This is more realistic than infinite speed
            max_speed = settings.get('vehicle.limits.max_speed', 10.0)
            target_speed = min(current_speed + acceleration * 0.5, max_speed)
        elif acceleration < 0:
            # Braking - set target speed to zero if deceleration is significant
            print("braking ", acceleration)
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