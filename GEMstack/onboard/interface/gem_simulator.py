from .gem import *
from ...mathutils.dubins import SecondOrderDubinsCar
from ...mathutils.dynamics import simulate
from ...state import VehicleState,ObjectPose,ObjectFrameEnum,Roadgraph,AgentState,AgentEnum,AgentActivityEnum,Obstacle,Sign
from ...knowledge.vehicle.geometry import front2steer,steer2front,heading_rate
from ...utils.loops import TimedLooper
from ...utils import config
from threading import Thread,Lock
import time
import numpy as np
import copy

class GEMDoubleIntegratorSimulationInterface(GEMInterface):
    def __init__(self, scene : str = None):
        GEMInterface.__init__(self)
        self.dubins = SecondOrderDubinsCar(
            wheelAngleMin=settings.get('vehicle.geometry.min_wheel_angle'),
            wheelAngleMax=settings.get('vehicle.geometry.max_wheel_angle'),
            velocityMin=-settings.get('vehicle.limits.max_reverse_speed'),
            velocityMax=-settings.get('vehicle.limits.max_speed'),
            accelMin=-settings.get('vehicle.limits.max_acceleration'),
            accelMax=settings.get('vehicle.limits.max_deceleration'),
            wheelAngleRateMin=-settings.get('vehicle.limits.max_steering_rate'),
            wheelAngleRateMax=settings.get('vehicle.limits.max_steering_rate'),
            wheelBase=settings.get('vehicle.geometry.wheelbase'))
        
        self.dt = settings.get('simulator.dt',0.01)
        self.real_time_multiplier = settings.get('simulator.real_time_multiplier',1.0)
        self.roadgraph = None
        self.agents = []
        if scene is None:
            scene = settings.get('simulator.scene',None)
        if isinstance(scene,str):
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
        self.gnss_callback = None
        self.imu_callback = None
        self.thread_lock = Lock()
        self.thread_data = dict()
        self.thread = None 
    
    def time(self) -> float:
        return self.simulation_time
        
    def start(self):
        assert self.thread is None
        self.thread_data['stop'] = False
        self.thread = Thread(target=self.simulate,args=(self.thread_lock,self.thread_data))

    def stop(self):
        self.thread_data['stop']= True
        self.thread.join()
        self.thread = None

    def hardware_faults(self) -> list:
        return []    
    
    def subscribe_gnss(self, callback):
        self.gnss_callback = callback
    
    def subscribe_imu(self, callback):
        self.imu_callback = callback
        
    def send_command(self, command : GEMVehicleCommand):
        self.last_command = command

    def get_reading(self) -> GEMVehicleReading:
        """Returns current read state of the vehicle"""
        return self.last_reading

    def simulate(self,lock : Lock, data : dict):
        looper = TimedLooper(self.dt / self.real_time_multiplier,name="Simulation thread")
        while looper and not data['stop']:
            lock.acquire()
            x,y,theta,v,phi = self.cur_vehicle_state
            #simulate actuators
            acceleration = self.last_command.accelerator_pedal_position - self.last_command.brake_pedal_position
            acceleration = np.clip(acceleration,*self.dubins.accelRange)
            phides = steer2front(self.last_command.steering_wheel_angle)
            phides = np.clip(phides,*self.dubins.wheelAngleRange)
            phi_deadband = 0.01
            steering_angle_rate = self.last_command.steering_wheel_speed if phides > phi + phi_deadband else \
                (-self.last_command.steering_wheel_speed if phides < phi - phi_deadband else 0.0)
            
            #simulate dynamics
            u = np.array([acceleration,steering_angle_rate])  #acceleration, steering angle rate
            next_state = simulate(self.dubins, self.cur_vehicle_state, u, self.dt, self.dt)
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
            last_reading = reading

            if self.gnss_callback is not None:
                pose = ObjectPose(frame=ObjectFrameEnum.ABSOLUTE_CARTESIAN,t=self.simulation_time,x=x,y=y,yaw=theta)
                vehicle_state = last_reading.to_state(pose)
                self.gnss_callback(vehicle_state)

            self.cur_vehicle_state = next_state
            self.simulation_time += self.dt
            lock.release()
