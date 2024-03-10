import numpy as np
from sensor_msgs.msg import CameraInfo
from image_geometry import PinholeCameraModel

def Pmatrix(fx,fy,cx,cy):
    """Returns a projection matrix for a given set of camera intrinsics."""
    return np.array([[fx,0,cx,0],
                     [0,fy,cy,0],
                     [0,0,1,0]])

a = np.load("zed_camera_info.pkl", allow_pickle=True)
print(a)
model = PinholeCameraModel()
model.fromCameraInfo(a) 

b= model.K
print(b)

