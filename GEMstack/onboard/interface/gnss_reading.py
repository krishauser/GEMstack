from .gem import *

@dataclass 
class GNSSReading:
    pose : ObjectPose
    speed : float
    status : str