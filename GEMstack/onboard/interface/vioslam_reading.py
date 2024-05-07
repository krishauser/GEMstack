from .gem import ObjectPose
from .gem import dataclass

#An object to package to the odometry message from VIO into a reading object
@dataclass 
class VioslamReading:
    pose : ObjectPose
    status : str