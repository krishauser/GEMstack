# ROS Headers
import rospy
from sensor_msgs.msg import Image,PointCloud2, CameraInfo
import sensor_msgs.point_cloud2 as pc2
import ctypes
import struct
import pickle
import image_geometry


# OpenCV and cv2 bridge
import cv2
from cv_bridge import CvBridge
import numpy as np
import os
import time

lidar_points = None
camera_image = None
depth_image = None
bridge = CvBridge()

def lidar_callback(lidar : PointCloud2):
    global lidar_points
    lidar_points = lidar

def camera_callback(info : CameraInfo):
    global camera_image
    camera_image = info

def pc2_to_numpy(pc2_msg, want_rgb = False):
    gen = pc2.read_points(pc2_msg, skip_nans=True)
    if want_rgb:
        xyzpack = np.array(list(gen),dtype=np.float32)
        if xyzpack.shape[1] != 4:
            raise ValueError("PointCloud2 does not have points")
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

def save_scan(cam_path):
    model = image_geometry.PinholeCameraModel()
    model.fromCameraInfo(camera_image)
    print(model.intrinsicMatrix())
    # with open(cam_path, "w") as file:
    #     pickle.dump(, file)
    #     print("Saving scan to", cam_path)

def main(folder='data',start_index=1):
    rospy.init_node("capture_cam_info",disable_signals=True)
    caminfo_sub = rospy.Subscriber("/oak/rgb/camera_info", CameraInfo, camera_callback)
    index = start_index
    print("Press any key to:")
    print("  store camera info ")
    print("Press Escape or Ctrl+C to quit")
    while True:
        if camera_image:
            time.sleep(1)
            # key = cv2.waitKey(0)
            # if key == -1:
            #     #nothing
            #     pass
            # elif key == 27:
            #     #escape
            #     break
            # else:
            #     print("this is what we want")
            #     if lidar_points is None or camera_image is None:
            #         print("Missing some messages?")
            #     else:
            #         files = [
            #             os.path.join(folder,'lidar{}.npz'.format(index)),
            #             os.path.join(folder,'color{}.png'.format(index))]
            #         save_scan(*files)
            #         index += 1

            files = [
                        os.path.join(folder,'cam_info{}.txt'.format(index))]
            save_scan(*files)
            index += 1

if __name__ == '__main__':
    import sys
    folder = 'data'
    start_index = 1
    if len(sys.argv) >= 2:
        folder = sys.argv[1]
    if len(sys.argv) >= 3:
        start_index = int(sys.argv[2])
    main(folder,start_index)
