from .gem import *
from ...utils import config,conversions,settings
import math
import time
import copy
from typing import List
from ...mathutils.dubins import SecondOrderDubinsCar
from ...mathutils.dynamics import simulate
from ...mathutils import transforms
from ...state import VehicleState,ObjectPose,ObjectFrameEnum,Roadgraph,AgentState,AgentEnum,AgentActivityEnum,Obstacle,Sign,AllState
from ...knowledge.vehicle.geometry import front2steer,steer2front,heading_rate
from ...knowledge.vehicle.dynamics import pedal_positions_to_acceleration, acceleration_to_pedal_positions
from ...utils.loops import TimedLooper
from dataclasses import replace
from threading import Thread,Lock

from gazebo_msgs.srv import GetModelState, GetModelStateResponse
from gazebo_msgs.msg import ModelState

from ackermann_msgs.msg import AckermannDrive
import roslaunch
# ROS Headers
import rospy
from std_msgs.msg import String, Bool, Float32, Float64
from sensor_msgs.msg import Image,PointCloud2,NavSatFix
try:
    from novatel_gps_msgs.msg import NovatelPosition, NovatelXYZ, Inspva
except ImportError:
    print("novatel_gps_msgs package missing ")
    pass
try:
    from septentrio_gnss_driver.msg import INSNavGeod
except ImportError:
    print("septentrio_gnss_driver package missing ")
    pass

from radar_msgs.msg import RadarTracks
from tf.transformations import euler_from_quaternion, quaternion_from_euler

# GEM PACMod Headers
from pacmod_msgs.msg import PositionWithSpeed, PacmodCmd, SystemRptFloat, VehicleSpeedRpt, GlobalRpt

# OpenCV and cv2 bridge
import cv2
import numpy as np





AGENT_DIMENSIONS = {
    'pedestrian' : (0.5,0.5,1.6),
    'bicyclist' : (1.8,0.5,1.6),
    'car' : (4.0,2.5,1.4),
    'medium_truck': (6.0,2.5,3.0),
    'large_truck': (10.0,2.5,3.5)
}

AGENT_TYPE_TO_ENUM = {
    'pedestrian' : AgentEnum.PEDESTRIAN,
    'bicyclist' : AgentEnum.BICYCLIST,
    'car' : AgentEnum.CAR,
    'medium_truck': AgentEnum.MEDIUM_TRUCK,
    'large_truck': AgentEnum.LARGE_TRUCK
}

AGENT_NOMINAL_VELOCITY = {
    'pedestrian' : 1.5,
    'bicyclist' : 5.0,
    'car' : 20.0,
    'medium_truck': 15.0,
    'large_truck': 10.0
}

AGENT_NOMINAL_ACCELERATION = {
    'pedestrian' : 2.0,
    'bicyclist' : 2.0,
    'car' : 5.0,
    'medium_truck': 3.0,
    'large_truck': 2.0
}

class AgentSimulation:
    def __init__(self, config):
        self.type = config['type']
        self.position = config['position'][:]
        self.velocity = config.get('velocity',[0,0])
        self.nominal_velocity = config.get('nominal_velocity',AGENT_NOMINAL_VELOCITY[self.type])
        self.target = config.get('target',None)
        self.target_radius = config.get('target_radius',0.1)
        self.target_path = config.get('target_path',None)
        self.behavior = config['behavior']
        self.start = self.position[:]
        self.yaw = config.get('yaw',0)


    def to_agent_state(self) -> AgentState:
        pose = ObjectPose(frame=ObjectFrameEnum.ABSOLUTE_CARTESIAN,t=time.time(),x=self.position[0],y=self.position[1],yaw=self.yaw)
        activity = AgentActivityEnum.MOVING if self.velocity[0] != 0 or self.velocity[1] != 0 else AgentActivityEnum.STOPPED
        return AgentState(pose=pose,dimensions=AGENT_DIMENSIONS[self.type],outline=None,
                          type=AGENT_TYPE_TO_ENUM[self.type],
                        activity=activity,velocity=(self.velocity[0],self.velocity[1],0),yaw_rate=0.0)

    def advance(self,dt):
        if self.behavior == 'stationary':
            self.velocity = [0,0]
            return
        elif self.behavior == 'target':
            if self.target is not None:
                self.seek_target(self.target,dt)
            elif self.target_path is not None:
                raise NotImplementedError("Path following not implemented yet")
        elif self.behavior == 'loop':
            if self.target is not None:
                self.seek_target(self.target,dt)
                if np.linalg.norm((self.position[0]-self.target[0],self.position[1]-self.target[1])) < self.target_radius:
                    self.target,self.start = self.start,self.target
            elif self.target_path is not None:
                raise NotImplementedError("Path following not implemented yet")
        else:
            raise ValueError("Unknown behavior "+self.behavior)
        
    def seek_target(self,target,dt):
        dx = target[0] - self.position[0]
        dy = target[1] - self.position[1]
        d = np.linalg.norm((dx,dy))
        v = np.linalg.norm(self.velocity)
        if d < 0.02:
            self.velocity = [0,0]
            return True
        direction = (dx/d,dy/d)
        dleft = d - self.target_radius
        if v < self.nominal_velocity:
            v += AGENT_NOMINAL_ACCELERATION[self.type]*dt
        if 0.5*v**2/AGENT_NOMINAL_ACCELERATION[self.type] > dleft:
            v -= AGENT_NOMINAL_ACCELERATION[self.type]*dt
        self.velocity = [v*direction[0],v*direction[1]]
        self.position = transforms.vector_madd(self.position,self.velocity,dt)
    


# class GEMDoubleIntegratorSimulation:
#     """Standard simulation of a second-order Dubins car with a double
#     integrator controller.  The simulation is deterministic and accepts
#     GEMVehicleReading and GEMVehicleCommand objects.

#     Gear switching is instantaneous.  Signals are activated instantly.
#     """
#     def __init__(self, scene : str = None):
#         self.dubins = SecondOrderDubinsCar(
#             wheelAngleMin=settings.get('vehicle.geometry.min_wheel_angle'),
#             wheelAngleMax=settings.get('vehicle.geometry.max_wheel_angle'),
#             velocityMin=-settings.get('vehicle.limits.max_reverse_speed'),
#             velocityMax=settings.get('vehicle.limits.max_speed'),
#             accelMin=-settings.get('vehicle.limits.max_acceleration'),
#             accelMax=settings.get('vehicle.limits.max_deceleration'),
#             wheelAngleRateMin=-settings.get('vehicle.limits.max_steering_rate'),
#             wheelAngleRateMax=settings.get('vehicle.limits.max_steering_rate'),
#             wheelBase=settings.get('vehicle.geometry.wheelbase'))
        
#         self.dt = settings.get('simulator.dt',0.01)
#         self.roadgraph = None
#         self.agents = []
#         if scene is None:
#             scene = settings.get('simulator.scene',None)
#         if isinstance(scene,str):
#             print("Loading simulator from scene",scene)
#             scene = config.load_config_recursive(scene)
#         self.agents = {}
#         if scene is None:
#             self.simulation_time = time.time()
#             self.start_state = (0.0,0.0,0.0)
#         else:
#             self.simulation_time = scene.get('time',time.time())
#             start_state = scene.get('vehicle_state',[0.0,0.0,0.0,0.0,0.0])
#             while len(start_state) < 5:
#                 start_state.append(0.0)
#             for k,a in scene.get('agents',{}).items():
#                 self.agents[k] = AgentSimulation(a)
#         self.cur_vehicle_state = np.array(start_state,dtype=float)

#         self.last_reading = GEMVehicleReading()
#         self.last_reading.speed = 0.0
#         self.last_reading.steering_wheel_angle = 0.0
#         self.last_reading.accelerator_pedal_position = 0.0
#         self.last_reading.brake_pedal_position = 0.0
#         self.last_reading.gear = 0
#         self.last_reading.left_turn_signal = False
#         self.last_reading.right_turn_signal = False
#         self.last_reading.horn_on = False
#         self.last_reading.wiper_level = 0
#         self.last_reading.headlights_on = False
#         #initialize last command
#         gear = -2 if self.cur_vehicle_state[3] == 0 else -1 if self.cur_vehicle_state[3] < 0 else 1
#         steering_wheel_angle = front2steer(self.cur_vehicle_state[4])
#         self.last_command = GEMVehicleCommand(gear,0,0,0,0,steering_wheel_angle,0)
    
#     def time(self) -> float:
#         return self.simulation_time

#     def simulate(self, T : float, command : Optional[GEMVehicleCommand]):
#         if command is not None:
#             self.last_command = command
#         for k,a in self.agents.items():
#             a.advance(T)
#         x,y,theta,v,phi = self.cur_vehicle_state
#         #print("x %.2f y %.2f theta %.2f v %.2f" % (x,y,theta,v))
#         #simulate actuators
#         accelerator_pedal_position = np.clip(self.last_command.accelerator_pedal_position,0.0,1.0)
#         brake_pedal_position = np.clip(self.last_command.brake_pedal_position,0.0,1.0)
#         acceleration = pedal_positions_to_acceleration(accelerator_pedal_position,brake_pedal_position,v,0,1)
#         acceleration = np.clip(acceleration,*self.dubins.accelRange)
#         phides = steer2front(self.last_command.steering_wheel_angle)
#         phides = np.clip(phides,*self.dubins.wheelAngleRange)
#         h = 0.01  #just for finite differencing
#         front_wheel_angle_rate = (steer2front(self.last_command.steering_wheel_angle + h*self.last_command.steering_wheel_speed) - steer2front(self.last_command.steering_wheel_angle)) / h
#         front_wheel_angle_rate_max = 2.0  #TODO: get from vehicle model
#         phi_deadband = 2*self.dt*front_wheel_angle_rate_max
#         steering_angle_rate = front_wheel_angle_rate if phides > phi + phi_deadband else \
#             (-front_wheel_angle_rate if phides < phi - phi_deadband else 0.0)
        
#         #simulate dynamics
#         u = np.array([acceleration,steering_angle_rate])  #acceleration, steering angle rate
#         #print("Accel %.2f, steering angle current %.2f, desired %.2f, rate %.2f" % (acceleration,phi,phides,steering_angle_rate))
#         next = simulate(self.dubins, self.cur_vehicle_state, (lambda x,t: u), T, self.dt)
#         next_state = next['x'][-1]
#         #braking deadband
#         if v > 0 and next_state[3] < 0:
#             next_state[3] = 0
#         if v < 0 and next_state[3] > 0:
#             next_state[3] = 0
#         x,y,theta,v,phi = next_state
#         v = np.clip(v,*self.dubins.velocityRange)
#         next_state = np.array([x,y,theta,v,phi])

#         #simulate sensors
#         reading = copy.copy(self.last_reading)
#         reading.steering_wheel_angle = front2steer(phi)
#         if acceleration > 0:
#             reading.brake_pedal_position = 0.0
#             reading.accelerator_pedal_position = accelerator_pedal_position
#         else:
#             reading.brake_pedal_position = brake_pedal_position
#             reading.accelerator_pedal_position = 0
#         reading.speed = v
#         if v > 0:
#             reading.gear = 1
#         else:
#             reading.gear = -1
#         #copy signals
#         reading.left_turn_signal = self.last_command.left_turn_signal
#         reading.right_turn_signal = self.last_command.right_turn_signal
#         reading.headlights_on = self.last_command.headlights_on
#         reading.horn_on = self.last_command.horn_on
#         reading.wiper_level = self.last_command.wiper_level
#         self.last_reading = reading

#         self.cur_vehicle_state = next_state
#         self.simulation_time += T

#     def pose(self) -> ObjectPose: 
#         x,y,theta,v,phi = self.cur_vehicle_state
#         return ObjectPose(frame=ObjectFrameEnum.ABSOLUTE_CARTESIAN,t=self.simulation_time,x=x,y=y,yaw=theta)

#     def state(self) -> VehicleState:
#         return self.last_reading.to_state(self.pose())

#     def set_pose(self,pose : ObjectPose):
#         self.cur_vehicle_state[0] = pose.x
#         self.cur_vehicle_state[1] = pose.y
#         self.cur_vehicle_state[2] = pose.yaw
    
#     def set_state(self,state : VehicleState):
#         self.set_pose(state.pose)
#         self.cur_vehicle_state[3] = state.v
#         self.cur_vehicle_state[4] = state.front_wheel_angle
    
#     def advance_vehicle_state(self, state : AllState, command : GEMVehicleCommand, T : float) -> AllState:
#         """Advances the vehicle state by the given amount of time T
#         under the given command.  Agents are not touched.
#         """
#         self.simulation_time = state.t
#         abs_pose = state.vehicle.to_frame(ObjectFrameEnum.ABSOLUTE_CARTESIAN, state.vehicle.pose, state.start_vehicle_pose)
#         self.set_state(abs_pose)
#         self.simulate(T, command)
#         return replace(state,t=self.simulation_time,vehicle=self.state())



class GEMDoubleIntegratorSimulationInterface(GEMInterface):
    """Standard GEMInterface for a second-order Dubins car model.
    The simulator is run in a separate thread so it acts like a real
    vehicle interface. 
    
    TODO: agent simulation?
    """
    def __init__(self, scene : str = None):
        GEMInterface.__init__(self)
        #self.simulator = GEMDoubleIntegratorSimulation(scene)
        self.real_time_multiplier = settings.get('simulator.real_time_multiplier',1.0)
        self.gnss_emulator_settings = settings.get('simulator.gnss_emulator',{})
        self.imu_emulator_settings = settings.get('simulator.imu_emulator',{})
        self.agent_emulator_settings = settings.get('simulator.agent_emulator',{})
        self.gnss_dt = self.gnss_emulator_settings.get('dt',0.1)
        self.imu_dt = self.imu_emulator_settings.get('dt',0.05)
        self.agent_dt = self.agent_emulator_settings.get('dt',0.1)
        # self.last_reading = self.simulator.last_reading
        # self.last_command =GEMRealSensorsWithSimMotionInterface self.simulator.last_command
        self.last_gnss_time = 0
        self.last_imu_time = 0
        self.last_agent_time = 0
        self.gnss_callback = None
        self.imu_callback = None
        self.agent_detector_callback = None
        self.thread_lock = Lock()
        self.thread_data = dict()
        self.thread = None 

        self.max_send_rate = settings.get('vehicle.max_command_rate',10.0)
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
        
        self.speed_sub  = rospy.Subscriber("/pacmod/parsed_tx/vehicle_speed_rpt", VehicleSpeedRpt, self.speed_callback)
        self.steer_sub = rospy.Subscriber("/pacmod/parsed_tx/steer_rpt", SystemRptFloat, self.steer_callback)
        self.global_sub = rospy.Subscriber("/pacmod/parsed_tx/global_rpt", GlobalRpt, self.global_callback)
        self.gnss_sub = None
        self.imu_sub = None
        self.front_radar_sub = None
        self.front_camera_sub = None
        self.front_depth_sub = None
        self.top_lidar_sub = None
        self.stereo_sub = None
        self.faults = []
        self.dt = settings.get('simulator.dt',0.01)


        self.dubins = SecondOrderDubinsCar(
            wheelAngleMin=settings.get('vehicle.geometry.min_wheel_angle'),
            wheelAngleMax=settings.get('vehicle.geometry.max_wheel_angle'),
            velocityMin=-settings.get('vehicle.limits.max_reverse_speed'),
            velocityMax=settings.get('vehicle.limits.max_speed'),
            accelMin=-settings.get('vehicle.limits.max_acceleration'),
            accelMax=settings.get('vehicle.limits.max_deceleration'),
            wheelAngleRateMin=-settings.get('vehicle.limits.max_steering_rate'),
            wheelAngleRateMax=settings.get('vehicle.limits.max_steering_rate'),
            wheelBase=settings.get('vehicle.geometry.wheelbase'))


        # # -------------------- PACMod setup --------------------
        # # GEM vehicle enable
        # self.enable_pub = rospy.Publisher('/pacmod/as_rx/enable', Bool, queue_size=1)
        # self.pacmod_enable = False

        # # GEM vehicle gear control, neutral, forward and reverse, publish once
        # self.gear_pub = rospy.Publisher('/pacmod/as_rx/shift_cmd', PacmodCmd, queue_size=1)
        # self.gear_cmd = PacmodCmd()
        # self.gear_cmd.enable = True
        # self.gear_cmd.ui16_cmd = PacmodCmd.SHIFT_NEUTRAL 

        # # GEM vehicle brake control
        # self.brake_pub = rospy.Publisher('/pacmod/as_rx/brake_cmd', PacmodCmd, queue_size=1)
        # self.brake_cmd = PacmodCmd()
        # self.brake_cmd.enable = False
        # self.brake_cmd.clear  = True
        # self.brake_cmd.ignore = True

        # # GEM vehicle forward motion control
        # self.accel_pub = rospy.Publisher('/pacmod/as_rx/accel_cmd', PacmodCmd, queue_size=1)
        # self.accel_cmd = PacmodCmd()
        # self.accel_cmd.enable = False
        # self.accel_cmd.clear  = True
        # self.accel_cmd.ignore = True

        # # GEM vehicle turn signal control
        # self.turn_pub = rospy.Publisher('/pacmod/as_rx/turn_cmd', PacmodCmd, queue_size=1)
        # self.turn_cmd = PacmodCmd()
        # self.turn_cmd.ui16_cmd = 1 # None

        # # GEM vechile steering wheel control
        # self.steer_pub = rospy.Publisher('/pacmod/as_rx/steer_cmd', PositionWithSpeed, queue_size=1)
        # self.steer_cmd = PositionWithSpeed()
        # self.steer_cmd.angular_position = 0.0 # radians, -: clockwise, +: counter-clockwise
        # self.steer_cmd.angular_velocity_limit = 2.0 # radians/second


        self.ackermann_pub = rospy.Publisher("ackermann_cmd", AckermannDrive, queue_size=1)
    def getModelState(self):
            # Get the current state of the vehicle
            # Input: None
            # Output: ModelState, the state of the vehicle, contain the
            #   position, orientation, linear velocity, angular velocity
            #   of the vehicle
        rospy.wait_for_service('/gazebo/get_model_state')
        try:
            serviceResponse = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            resp = serviceResponse(model_name='gem_e4')
        except rospy.ServiceException as exc:
            rospy.loginfo("Service did not process request: "+str(exc))
            resp = GetModelStateResponse()
            resp.success = False
        return resp
    

    # Need to double check on this time
    def time(self) -> float:
        # return self.simulator.time()
        seconds = rospy.get_time()
        return seconds
    
    # Pac mod call backs, need to double check
    def speed_callback(self,msg : VehicleSpeedRpt):
        self.last_reading.speed = msg.vehicle_speed   # forward velocity in m/s

    def steer_callback(self, msg):
        self.last_reading.steering_wheel_angle = msg.output
    
    def global_callback(self, msg):
        self.faults = []
        if msg.override_active:
            self.faults.append("override_active")
        if msg.config_fault_active:
            self.faults.append("config_fault_active")
        if msg.user_can_timeout:
            self.faults.append("user_can_timeout")
        if msg.user_can_read_errors:
            self.faults.append("user_can_read_errors")
        if msg.brake_can_timeout:
            self.faults.append("brake_can_timeout")
        if msg.steering_can_timeout:
            self.faults.append("steering_can_timeout")
        if msg.vehicle_can_timeout:
            self.faults.append("vehicle_can_timeout")
        if msg.subsystem_can_timeout:
            self.faults.append("subsystem_can_timeout")
        
    #TODO: Start gazebo
    def start(self):
        # assert self.thread is None
        # print("Running simulator thread...")
        # self.thread_data['stop'] = False
        # self.thread = Thread(target=self.simulate,args=(self.thread_lock,self.thread_data))
        # self.thread.start()
        # rospy.init_node("gazebo_launcher", anonymous=True)
        # uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        # roslaunch.configure_logging(uuid)
        # launch = roslaunch.parent.ROSLaunchParent(
        #     uuid, ["/path/to/your/launch/file.launch"]
        # )
        # launch.start()
        # rospy.loginfo("Gazebo started via Python launch.")

        # rospy.spin()
        pass

    def stop(self):
        print("Stopping simulator thread...")
        self.thread_data['stop']= True
        self.thread.join()
        self.thread = None
        print("Done.")

    def hardware_faults(self) -> list:
        return []    
    
    def sensors(self):
        #TODO: simulate other sensors?
        return ['gnss','imu','agent_detector']

    def subscribe_sensor(self, name, callback, type = None):
        if name == 'gnss':
            topic = self.ros_sensor_topics[name]
            if topic.endswith('fix'):
                print("Working")
                #GEM e2 Gazebo Simulator
                if type is not None and (type is not GNSSReading and type is not NavSatFix):
                    raise ValueError("Gazebo GEM e2 only supports NavSatFix for GNSS")
                if type is NavSatFix:
                    self.gnss_sub = rospy.Subscriber(topic, NavSatFix, callback)
                else:
                    def callback_with_gnss_reading(NavSatFix_msg: NavSatFix):
                        # pose = ObjectPose(ObjectFrameEnum.GLOBAL,
                        #             x=NavSatFix_msg.longitude,
                        #             y=NavSatFix_msg.latitude,
                        #             z=NavSatFix_msg.altitude,
                        #             yaw=math.radians(NavSatFix_msg.azimuth),  #heading from north in degrees
                        #             roll=math.radians(NavSatFix_msg.roll),
                        #             pitch=math.radians(NavSatFix_msg.pitch),
                        #             )
                        # speed = np.sqrt(NavSatFix_msg.east_velocity**2 + NavSatFix_msg.north_velocity**2)

                        # unknown roll, pitch, yaw and speed
                        pose = ObjectPose(ObjectFrameEnum.GLOBAL,
                                    x=NavSatFix_msg.longitude,
                                    y=NavSatFix_msg.latitude,
                                    z=NavSatFix_msg.altitude,
                                    yaw=math.radians(0),  #heading from north in degrees
                                    roll=math.radians(0),
                                    pitch=math.radians(0),
                                    t = 0
                                    )
                        speed = 0
                        callback(GNSSReading(pose,speed,NavSatFix_msg.status))
                    self.gnss_sub = rospy.Subscriber(topic, NavSatFix, callback_with_gnss_reading)
            else:
                print("not Working")


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
        elif name == 'front_radar':
            if type is not None and type is not RadarTracks:
                raise ValueError("GEMHardwareInterface only supports RadarTracks for front radar")
            self.front_radar_sub = rospy.Subscriber("/front_radar/front_radar/radar_tracks", RadarTracks, callback)
        elif name == 'front_camera':
            topic = self.ros_sensor_topics[name]
            if type is not None and (type is not Image and type is not cv2.Mat):
                raise ValueError("GEMHardwareInterface only supports Image or OpenCV for front camera")
            if type is None or type is Image:
                self.front_camera_sub = rospy.Subscriber(topic, Image, callback)
            else:
                def callback_with_cv2(msg : Image):
                    #print("received image with size",msg.width,msg.height,"encoding",msg.encoding)                    
                    cv_image = conversions.ros_Image_to_cv2(msg, desired_encoding="bgr8")
                    callback(cv_image)
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


        
    def send_command(self, command : GEMVehicleCommand):

        rospy.loginfo(f"got command: {command}")

        def extract_vehicle_info( currentPose):

            #TODO: replace with already proviced function and make sure convention is correct
            def quaternion_to_euler(x, y, z, w):
                t0 = +2.0 * (w * x + y * z)
                t1 = +1.0 - 2.0 * (x * x + y * y)
                roll = np.arctan2(t0, t1)
                t2 = +2.0 * (w * y - z * x)
                t2 = +1.0 if t2 > +1.0 else t2
                t2 = -1.0 if t2 < -1.0 else t2
                pitch = np.arcsin(t2)
                t3 = +2.0 * (w * z + x * y)
                t4 = +1.0 - 2.0 * (y * y + z * z)
                yaw = np.arctan2(t3, t4)
                return [roll, pitch, yaw]
            pos_x, pos_y, vel, yaw = 0, 0, 0, 0

            pos_x = currentPose.pose.position.x
            pos_y = currentPose.pose.position.y

            vel =  np.linalg.norm([currentPose.twist.linear.x, currentPose.twist.linear.y, currentPose.twist.linear.z])

            #TODO: figure out velocity sign Is this correct

            # if currentPose.twist.linear.x<0:
            #   vel = -vel
            _,_, yaw = quaternion_to_euler(currentPose.pose.orientation.x, currentPose.pose.orientation.y, currentPose.pose.orientation.z, currentPose.pose.orientation.w)


            return pos_x, pos_y, vel, yaw # note that yaw is in radian
        

        _,_,v,phi = extract_vehicle_info(self.getModelState())


        accelerator_pedal_position = np.clip(command.accelerator_pedal_position,0.0,1.0)
        brake_pedal_position = np.clip(command.brake_pedal_position,0.0,1.0)
        acceleration = pedal_positions_to_acceleration(accelerator_pedal_position,brake_pedal_position,v,0,1)
        acceleration = np.clip(acceleration,*self.dubins.accelRange)
        phides = steer2front(command.steering_wheel_angle)
        phides = np.clip(phides,*self.dubins.wheelAngleRange)
        h = 0.01  #just for finite differencing
        front_wheel_angle_rate = (steer2front(command.steering_wheel_angle + h*command.steering_wheel_speed) - steer2front(command.steering_wheel_angle)) / h
        front_wheel_angle_rate_max = 2.0  #TODO: get from vehicle model
        phi_deadband = 2*self.dt*front_wheel_angle_rate_max
        steering_angle_rate = front_wheel_angle_rate if phides > phi + phi_deadband else \
            (-front_wheel_angle_rate if phides < phi - phi_deadband else 0.0)
        

        

        msg = AckermannDrive()
        msg.acceleration = acceleration
        msg.speed = float('inf') if acceleration >0 else 0  #acceleration * self.dt 
        msg.steering_angle = -phides
        msg.steering_angle_velocity = -steering_angle_rate


        self.ackermann_pub.publish(msg)
        rospy.loginfo(f"Published AckermannDrive: {msg}")
        self.last_command = command



        # incorporate controller publisher

    def get_reading(self) -> GEMVehicleReading:
        """Returns current read state of the vehicle"""
        return self.last_reading
    
    # # PACMod enable callback function
    # def pacmod_enable_callback(self, msg):
    #     if self.pacmod_enable == False and msg.data == True:
    #         print("PACMod enabled, enabling gear, brake, accel, steer, and turn")
    #         self.send_first_command()
    #     elif self.pacmod_enable == True and msg.data == False:
    #         print("PACMod disabled")
    #     self.pacmod_enable = msg.data

    # def hardware_faults(self) -> List[str]:
    #     if self.pacmod_enable == False:
    #         return self.faults + ["disengaged"]
    #     return self.faults

    # def simulate(self,lock : Lock, data : dict):
    #     looper = TimedLooper(self.simulator.dt / self.real_time_multiplier,name="Simulation thread")
    #     while looper and not data['stop']:
    #         with lock:
    #             self.simulator.simulate(self.simulator.dt, self.last_command)
    #             self.last_reading = self.simulator.last_reading

    #             if self.gnss_callback is not None and self.simulator.simulation_time - self.last_gnss_time > self.gnss_dt:
    #                 vehicle_state = self.simulator.state()
    #                 self.gnss_callback(self.gnss_emulator(vehicle_state))
    #                 self.last_gnss_time = self.simulator.simulation_time
    #             if self.imu_callback is not None and self.simulator.simulation_time - self.last_imu_time > self.imu_dt:
    #                 pose = ObjectPose(frame=ObjectFrameEnum.CURRENT,t=self.simulation_time,x=0,y=0,yaw=0)
    #                 vehicle_state = self.last_reading.to_state(pose)
    #                 self.imu_callback(vehicle_state)
    #                 self.last_imu_time = self.simulator.simulation_time
    #             if self.agent_detector_callback is not None and self.simulator.simulation_time - self.last_agent_time > self.agent_dt:
    #                 for k,a in self.simulator.agents.items():
    #                     self.agent_detector_callback(k,a.to_agent_state())
    #                 self.last_agent_time = self.simulator.simulation_time

    # def gnss_emulator(self, vehicle_state: VehicleState):
    #     position_noise = self.gnss_emulator_settings.get('position_noise',0.0)
    #     orientation_noise = self.gnss_emulator_settings.get('orientation_noise',0.0)
    #     velocity_noise_const = self.gnss_emulator_settings.get('velocity_noise.constant',0.0)
    #     velocity_noise_linear = self.gnss_emulator_settings.get('velocity_noise.linear',0.0)
    #     if position_noise > 0 or orientation_noise > 0 or velocity_noise_const > 0 or velocity_noise_linear > 0:
    #         # Add noise to the vehicle state
    #         position = vehicle_state.pose.x,vehicle_state.pose.y
    #         yaw = vehicle_state.pose.yaw
    #         v = vehicle_state.v
    #         position = (np.random.normal(position[0],position_noise), np.random.normal(position[1],position_noise))
    #         yaw = np.random.normal(vehicle_state.pose.yaw,orientation_noise)
    #         v = np.random.normal(v,velocity_noise_const + abs(v)*velocity_noise_linear)
    #         pose = replace(vehicle_state.pose, x=position[0], y=position[1], yaw=yaw)
    #         return replace(vehicle_state, pose=pose, v=v)
    #     return vehicle_state
    

    #TODO: agent emulator, imu emulator
