from ...utils import settings
from ..component import Component
from ...state.vehicle import VehicleState
from ..interface.gem import GEMVehicleCommand, GEMInterface

class AccelerationCalibrationDriver(Component):
    def __init__(self,vehicle_interface : GEMInterface, **args):
        self.vehicle_interface = vehicle_interface
        self.acc_measured = []
        self.acc_output = []
        self.speed = []
        self.acc = args["acc"]

    def rate(self):
        return 50.0

    def state_inputs(self):
        return ['vehicle']

    def state_outputs(self):
        return []

    def update(self, vehicle : VehicleState):

        accel = self.acc
        steering_angle = 0
        self.vehicle_interface.send_command(self.vehicle_interface.simple_command(accel,steering_angle, vehicle))
        
        self.acc_measured.append(vehicle.acceleration)
        self.acc_output.append(accel) 
        self.speed.append(vehicle.v)
        print('acc_measured_list:', self.acc_measured)
        print('acc_out_list:', self.acc_output)
        print('speed_list:', self.speed)
    
    def healthy(self):
        return True