from ...utils import settings
from ..component import Component
from ...state.vehicle import VehicleState
from ...state.trajectory import Path,Trajectory
from ..interface.gem import GEMVehicleCommand, GEMInterface
import copy

class StopTrajectoryTracker(Component):
    def __init__(self,vehicle_interface : GEMInterface):
        self.vehicle_interface = vehicle_interface

    def rate(self):
        return 50.0

    def state_inputs(self):
        return []

    def state_outputs(self):
        return []

    def update(self):
        print("Stopping, current speed %.3f m/s"%(self.vehicle_interface.last_reading.speed))
        brake_amount = settings.get('control.recovery.brake_amount')
        brake_speed = settings.get('control.recovery.brake_speed')
        if self.vehicle_interface.last_command is not None:
            cmd = copy.copy(self.vehicle_interface.last_command)
            cmd.accelerator_pedal_position = 0.0
            cmd.brake_pedal_position = brake_amount
            cmd.brake_pedal_speed = brake_speed
        else:
            cmd = GEMVehicleCommand(brake_pedal_position=brake_amount)
        if self.vehicle_interface.last_reading.speed == 0:
            #shift to park
            cmd.gear = -2
        self.vehicle_interface.send_command(cmd)
    
    def healthy(self):
        return True
