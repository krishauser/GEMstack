from dataclasses import dataclass
from ...utils import settings
from ...state import VehicleState, ObjectPose, ObjectFrameEnum
from ...knowledge.vehicle.geometry import front2steer,steer2front,heading_rate
from ...knowledge.vehicle.dynamics import pedal_positions_to_acceleration, acceleration_to_pedal_positions

@dataclass
class GEMVehicleReading:
    speed : float = 0
    gear : int = 0
    accelerator_pedal_position : float = 0
    brake_pedal_position : float = 0
    steering_wheel_angle : float = 0
    left_turn_signal : bool = False
    right_turn_signal : bool = False
    headlights_on : bool = False
    horn_on : bool = False
    wiper_level : int = 0

    def from_state(self, state: VehicleState):
        self.speed = state.v
        self.steering_wheel_angle = state.steering_wheel_angle
        pitch = state.pose.pitch if state.pose.pitch is not None else 0.0
        acc_pos,brake_pos,gear = acceleration_to_pedal_positions(state.acceleration, state.v, pitch, state.gear)
        self.accelerator_pedal_position = acc_pos
        self.brake_pedal_position = brake_pos
        self.gear = state.gear
        self.left_turn_signal = state.left_turn_indicator
        self.right_turn_signal = state.right_turn_indicator
        self.horn_on = state.horn_on
        self.wiper_level = state.wiper_level
        self.headlights_on = state.headlights_on

    def to_state(self, pose : ObjectPose = None) -> VehicleState:
        if pose is None:
            pose = ObjectPose(frame = ObjectFrameEnum.CURRENT,t=0,x=0,y=0,yaw=0)
        pitch = pose.pitch if pose.pitch is not None else 0.0
        wheel_base = settings.get('vehicle.geometry.wheelbase')
        front_wheel_angle=front2steer(self.steering_wheel_angle)
        heading_rate=heading_rate(front_wheel_angle,self.speed,wheel_base)
        acc = pedal_positions_to_acceleration(self.accelerator_pedal_position, self.brake_pedal_position, self.speed, pitch, self.gear)
        return VehicleState(pose,v=self.speed,acceleration=acc,gear=self.gear,steering_wheel_angle=self.steering_wheel_angle,
                            front_wheel_angle=front_wheel_angle,turn_rate=heading_rate,
                            left_turn_indicator=self.left_turn_signal,right_turn_indicator=self.right_turn_signal,
                            horn_on=self.horn_on,wiper_level=self.wiper_level,headlights_on=self.headlights_on)


@dataclass
class GEMVehicleCommand:
    gear : int                             #follows convention in state.vehicle.VehicleState. -2: park, -1 reverse: 0: neutral, 1..n: forward
    accelerator_pedal_position : float
    accelerator_pedal_speed : float
    brake_pedal_position : float
    brake_pedal_speed : float
    steering_wheel_angle : float
    steering_wheel_speed : float
    left_turn_signal : bool = False
    right_turn_signal : bool = False
    headlights_on : bool = False
    horn_on : bool = False
    wiper_level : int = 0


class GEMInterface:
    """Base class for simulated / physical GEM vehicle.
    """
    def __init__(self):
        self.last_command = None  # type: GEMVehicleCommand
        self.last_reading = None  # type: GEMVehicleReading

    def start(self):
        pass

    def stop(self):
        pass

    def time(self) -> float:
        """Returns the current time"""
        raise NotImplementedError()

    def get_reading(self) -> GEMVehicleReading:
        """Returns current read state of the vehicle"""
        raise NotImplementedError()

    def send_command(cmd : GEMVehicleCommand):
        """Sends a command to the vehicle"""
        raise NotImplementedError()
           
    def subscribe_gnss(self, callback):
        raise NotImplementedError()

    def subscribe_imu(self, callback):
        raise NotImplementedError()

    def subscribe_lidar(self, callback):
        raise NotImplementedError()

    def subscribe_stereo(self, callback):
        raise NotImplementedError()

    def subscribe_radar(self, callback):
        raise NotImplementedError()
    
    def hardware_faults(self) -> list:
        """Returns a list of hardware faults (by component)"""
        raise NotImplementedError()

    def simple_command(self, acceleration_amount : float, steering_wheel_angle : float) -> GEMVehicleCommand:
        cmd = GEMVehicleCommand()
        cmd.brake_pedal_speed = settings.get('vehicle.control_defaults.brake_pedal_speed')
        cmd.steering_wheel_speed = settings.get('vehicle.control_defaults.steering_wheel_speed')
        cmd.accelerator_pedal_speed = settings.get('vehicle.control_defaults.accelerator_pedal_speed')
        cmd.gear = 1
        cmd.left_turn_signal = False
        cmd.right_turn_signal = False
        cmd.headlights_on = False
        cmd.horn_on = False
        cmd.wiper_level = 0

        if acceleration_amount < 0:
            cmd.brake_pedal_position = -acceleration_amount
            if cmd.brake_pedal_position > 1:
                cmd.brake_pedal_position = 1
            cmd.accelerator_pedal_position = 0
        else:
            cmd.accelerator_pedal_position = acceleration_amount
            if cmd.accelerator_pedal_position > 1:
                cmd.accelerator_pedal_position = 1
            cmd.brake_pedal_position = 0
        cmd.steering_wheel_angle = steering_wheel_angle
        return cmd
