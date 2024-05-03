from .gem import ObjectPose
from .gem import dataclass

@dataclass 
class GNSSReading:
    pose : ObjectPose
    speed : float
    status : str