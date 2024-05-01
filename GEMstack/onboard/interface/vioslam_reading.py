from .gem import ObjectPose
from .gem import dataclass

@dataclass 
class VioslamReading:
    pose : ObjectPose
    status : str