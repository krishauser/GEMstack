from .gem import GEMInterface, GEMVehicleCommand, GEMVehicleReading
from .gem_hardware import GEMHardwareInterface
from .gem_simulator import GEMDoubleIntegratorSimulationInterface
from typing import Callable, List

class GEMRealSensorsWithSimMotionInterface(GEMInterface):
    """Class that uses sensors from the physical GEM vehicle but
    commands and readings go to a simulation.  Just the
    GEMDoubleIntegratorSimulationInterface is supported for sim now.
    """
    def __init__(self, scene:str = None):
        self.sim = GEMDoubleIntegratorSimulationInterface(scene)
        self.real = GEMHardwareInterface()

    def start(self):
        self.sim.start()
        self.real.start()

    def stop(self):
        self.sim.stop()
        self.real.stop()

    def time(self) -> float:
        return self.sim.time()

    def get_reading(self) -> GEMVehicleReading:
        return self.sim.get_reading()

    def send_command(self, cmd : GEMVehicleCommand):
        self.sim.send_command(cmd)

    def sensors(self):
        return self.real.sensors()

    def subscribe_sensor(self, name : str, callback : Callable, type = None) -> None:
        if name in ['gnss','imu']:
            return self.sim.subscribe_sensor(name,callback,type)
        return self.real.subscribe_sensor(name,callback,type)

    def hardware_faults(self) -> List[str]:
        return self.real.hardware_faults() + self.sim.hardware_faults()

