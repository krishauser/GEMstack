from typing import List
from .gem import *
from ...mathutils.dubins import SecondOrderDubinsCar
from ...mathutils.dynamics import simulate
from ...state import VehicleState,ObjectPose,ObjectFrameEnum,Roadgraph,AgentState,AgentEnum,AgentActivityEnum,Obstacle,Sign,AllState
from ...knowledge.vehicle.geometry import front2steer,steer2front,heading_rate
from ...knowledge.vehicle.dynamics import pedal_positions_to_acceleration, acceleration_to_pedal_positions
from ...utils.loops import TimedLooper
from ...utils import config
from dataclasses import replace
from threading import Thread,Lock
import time
import numpy as np
import copy

class GEMDoubleIntegratorSimulation:
    """Standard simulation of a second-order Dubins car with a double
    integrator controller.  The simulation is deterministic and accepts
    GEMVehicleReading and GEMVehicleCommand objects.

    Gear switching is instantaneous.  Signals are activated instantly.
    """
    def __init__(self, scene : str = None):
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
        
        self.dt = settings.get('simulator.dt',0.01)
        self.roadgraph = None
        self.agents = []
        if scene is None:
            scene = settings.get('simulator.scene',None)
        if isinstance(scene,str):
            print("Loading simulator from scene",scene)
            scene = config.load_config_recursive(scene)
        if scene is None:
            self.simulation_time = time.time()
            self.start_state = (0.0,0.0,0.0)
        else:
            self.simulation_time = scene.get('time',time.time())
            start_state = scene.get('vehicle_state',[0.0,0.0,0.0,0.0,0.0])
            while len(start_state) < 5:
                start_state.append(0.0)
        self.cur_vehicle_state = np.array(start_state,dtype=float)

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
        #initialize last command
        gear = -2 if self.cur_vehicle_state[3] == 0 else -1 if self.cur_vehicle_state[3] < 0 else 1
        steering_wheel_angle = front2steer(self.cur_vehicle_state[4])
        self.last_command = GEMVehicleCommand(gear,0,0,0,0,steering_wheel_angle,0)
    
    def time(self) -> float:
        return self.simulation_time

    def simulate(self, T : float, command : Optional[GEMVehicleCommand]):
        if command is not None:
            self.last_command = command
        x,y,theta,v,phi = self.cur_vehicle_state
        #print("x %.2f y %.2f theta %.2f v %.2f" % (x,y,theta,v))
        #simulate actuators
        accelerator_pedal_position = np.clip(self.last_command.accelerator_pedal_position,0.0,1.0)
        brake_pedal_position = np.clip(self.last_command.brake_pedal_position,0.0,1.0)
        acceleration = pedal_positions_to_acceleration(accelerator_pedal_position,brake_pedal_position,v,0,1)
        acceleration = np.clip(acceleration,*self.dubins.accelRange)
        phides = steer2front(self.last_command.steering_wheel_angle)
        phides = np.clip(phides,*self.dubins.wheelAngleRange)
        phi_deadband = 0.01
        steering_angle_rate = self.last_command.steering_wheel_speed if phides > phi + phi_deadband else \
            (-self.last_command.steering_wheel_speed if phides < phi - phi_deadband else 0.0)
        
        #simulate dynamics
        u = np.array([acceleration,steering_angle_rate])  #acceleration, steering angle rate
        #print("Accel %.2f, steering angle current %.2f, desired %.2f, rate %.2f" % (acceleration,phi,phides,steering_angle_rate))
        next = simulate(self.dubins, self.cur_vehicle_state, (lambda x,t: u), T, self.dt)
        next_state = next['x'][-1]
        x,y,theta,v,phi = next_state
        v = np.clip(v,*self.dubins.velocityRange)
        next_state = np.array([x,y,theta,v,phi])

        #simulate sensors
        reading = copy.copy(self.last_reading)
        reading.steering_wheel_angle = front2steer(phi)
        if acceleration > 0:
            reading.brake_pedal_position = 0.0
            reading.accelerator_pedal_position = acceleration
        else:
            reading.brake_pedal_position = -acceleration
            reading.accelerator_pedal_position = 0
        reading.speed = v
        if v > 0:
            reading.gear = 1
        else:
            reading.gear = -1
        #copy signals
        reading.left_turn_signal = self.last_command.left_turn_signal
        reading.right_turn_signal = self.last_command.right_turn_signal
        reading.headlights_on = self.last_command.headlights_on
        reading.horn_on = self.last_command.horn_on
        reading.wiper_level = self.last_command.wiper_level
        self.last_reading = reading

        self.cur_vehicle_state = next_state
        self.simulation_time += self.dt

    def pose(self) -> ObjectPose: 
        x,y,theta,v,phi = self.cur_vehicle_state
        return ObjectPose(frame=ObjectFrameEnum.ABSOLUTE_CARTESIAN,t=self.simulation_time,x=x,y=y,yaw=theta)

    def state(self) -> VehicleState:
        return self.last_reading.to_state(self.pose())

    def set_pose(self,pose : ObjectPose):
        self.cur_vehicle_state[0] = pose.x
        self.cur_vehicle_state[1] = pose.y
        self.cur_vehicle_state[2] = pose.yaw
    
    def set_state(self,state : VehicleState):
        self.set_pose(state.pose)
        self.cur_vehicle_state[3] = state.v
        self.cur_vehicle_state[4] = state.front_wheel_angle
    
    def advance_state(self, state : AllState, command : GEMVehicleCommand, T : float) -> AllState:
        """Advances the vehicle state by the given amount of time T
        under the given command.  Agents are not touched.
        """
        self.simulation_time = state.t
        abs_pose = state.vehicle.to_frame(ObjectFrameEnum.ABSOLUTE_CARTESIAN, state.vehicle.pose, state.start_vehicle_pose)
        self.set_state(abs_pose)
        self.simulate(T, command)
        return replace(state,t=self.simulation_time,vehicle=self.state())



class GEMDoubleIntegratorSimulationInterface(GEMInterface):
    """Standard GEMInterface for a second-order Dubins car model.
    The simulator is run in a separate thread so it acts like a real
    vehicle interface. 
    
    TODO: agent simulation?
    """
    def __init__(self, scene : str = None):
        GEMInterface.__init__(self)
        self.simulator = GEMDoubleIntegratorSimulation(scene)
        self.real_time_multiplier = settings.get('simulator.real_time_multiplier',1.0)
        self.last_reading = self.simulator.last_reading
        self.last_command = self.simulator.last_command
        self.gnss_callback = None
        self.imu_callback = None
        self.thread_lock = Lock()
        self.thread_data = dict()
        self.thread = None 
    
    def time(self) -> float:
        return self.simulator.time()
        
    def start(self):
        assert self.thread is None
        print("Running simulator thread...")
        self.thread_data['stop'] = False
        self.thread = Thread(target=self.simulate,args=(self.thread_lock,self.thread_data))
        self.thread.start()

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
        return ['gnss','imu']

    def subscribe_sensor(self, name, callback, type = None):
        if name == 'gnss':
            if type is not None and type is not VehicleState:
                raise ValueError("GEMDoubleIntegratorSimulationInterface only supports VehicleState for GNSS")
            self.gnss_callback = callback
        elif name == 'imu':
            if type is not None and type is not VehicleState:
                raise ValueError("GEMDoubleIntegratorSimulationInterface only supports VehicleState for IMU")
            self.imu_callback = callback
        else:
            print("Warning, GEM simulator doesn't provide sensor",name)
        
    def send_command(self, command : GEMVehicleCommand):
        self.last_command = command

    def get_reading(self) -> GEMVehicleReading:
        """Returns current read state of the vehicle"""
        return self.last_reading

    def simulate(self,lock : Lock, data : dict):
        looper = TimedLooper(self.simulator.dt / self.real_time_multiplier,name="Simulation thread")
        while looper and not data['stop']:
            with lock:
                self.simulator.simulate(self.simulator.dt, self.last_command)
                self.last_reading = self.simulator.last_reading

                if self.gnss_callback is not None:
                    vehicle_state = self.simulator.state()
                    self.gnss_callback(vehicle_state)
                if self.imu_callback is not None:
                    pose = ObjectPose(frame=ObjectFrameEnum.CURRENT,t=self.simulation_time,x=0,y=0,yaw=0)
                    vehicle_state = self.last_reading.to_state(pose)
                    self.imu_callback(vehicle_state)
