# ROS Headers
import rospy
from sensor_msgs.msg import CameraInfo
import ctypes
import struct

import numpy as np
import os

camera_info = None
depth_info = None

def camera_callback(info : CameraInfo):
    global camera_info
    camera_info = info

def depth_callback(info : CameraInfo):
    global depth_info
    depth_info = info

def main(folder='data'):
    rospy.init_node("capture_zed",disable_signals=True)
    camera_sub = rospy.Subscriber("/zed2/zed_node/rgb/camera_info", CameraInfo, camera_callback)
    depth_sub = rospy.Subscriber("/zed2/zed_node/depth/camera_info", CameraInfo, depth_callback)

    while camera_info is None or depth_info is None:
        import time
        time.sleep(0.1)
    with open(f'{folder}/cameraIncrinsics.txt', 'w') as f:
        print('\n\n##################RGB##########################', file=f)
        print(camera_info, file=f)
        print('\n\n##################DEPTH##########################', file=f)
        print(depth_info, file=f)
    print(camera_info)
    print(depth_info)
    # image_geometry = image_geometry.PinholeCameraModel().fromCameraInfo(camera_info)
    # fx, fy, cx, cy = image_geometry.fx(), image_geometry.fy(), image_geometry.cx(), image_geometry.cy()
    # print(fx, fy, cx, cy)

if __name__ == '__main__':
    import sys
    folder = 'data'
    start_index = 1
    if len(sys.argv) >= 2:
        folder = sys.argv[1]
    main(folder)
