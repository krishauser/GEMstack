import cv2
import numpy as np
import struct
import ctypes

try:
    import sensor_msgs.point_cloud2 as pc2
except ImportError:
    pc2 = None

try:
    from cv_bridge import CvBridge
    cv_bridge = CvBridge()
except ImportError:
    cv_bridge = None


def ros_PointCloud2_to_numpy(pc2_msg, want_rgb = False):
    if pc2 is None:
        raise ImportError("ROS is not installed")
    gen = pc2.read_points(pc2_msg, skip_nans=True)
    if want_rgb:
        xyzpack = np.array(list(gen),dtype=np.float32)
        if xyzpack.shape[1] != 4:
            raise ValueError("PointCloud2 does not have points with color data.")
        xyzrgb = np.empty((xyzpack.shape[0],6))
        xyzrgb[:,:3] = xyzpack[:,:3]
        for i,x in enumerate(xyzpack):
            rgb = x[3] 
            # cast float32 to int so that bitwise operations are possible
            s = struct.pack('>f' ,rgb)
            i = struct.unpack('>l',s)[0]
            # you can get back the float value by the inverse operations
            pack = ctypes.c_uint32(i).value
            r = (pack & 0x00FF0000)>> 16
            g = (pack & 0x0000FF00)>> 8
            b = (pack & 0x000000FF)
            #r,g,b values in the 0-255 range
            xyzrgb[i,3:] = (r,g,b)
        return xyzrgb
    else:
        return np.array(list(gen),dtype=np.float32)[:,:3]


def ros_Image_to_cv2(msg, desired_encoding="passthrough"):
    if cv_bridge is None:
        raise ImportError("cv_bridge is not installed")
    return cv_bridge.imgmsg_to_cv2(msg, desired_encoding=desired_encoding)