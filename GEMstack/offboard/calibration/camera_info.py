# ROS Headers
import rospy
from sensor_msgs.msg import Image,PointCloud2, CameraInfo
import sensor_msgs.point_cloud2 as pc2
import ctypes
import struct
import pickle
import image_geometry

import numpy as np
import os
import time

camera_image = None

def camera_callback(info : CameraInfo):
    global camera_image
    camera_image = info

def get_intrinsics():
    model = image_geometry.PinholeCameraModel()
    model.fromCameraInfo(camera_image)
    print(model.intrinsicMatrix())

def main(folder='data',start_index=1):
    rospy.init_node("capture_cam_info",disable_signals=True)
    caminfo_sub = rospy.Subscriber("/oak/rgb/camera_info", CameraInfo, camera_callback)
    while True:
        if camera_image:
            time.sleep(1)
            get_intrinsics()

if __name__ == '__main__':
    import sys
    folder = 'data'
    start_index = 1
    if len(sys.argv) >= 2:
        folder = sys.argv[1]
    if len(sys.argv) >= 3:
        start_index = int(sys.argv[2])
    main(folder,start_index)
