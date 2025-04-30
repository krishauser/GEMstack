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
from functools import partial
from tools.save_cali import save_in

camera_info = {"oak": None, "fr": None, "fl": None, "rr": None, "rl": None, "oak_stereo": None, "oak_right": None, "oak_left": None}

def info_callback(camera, info : CameraInfo):
    global camera_info
    camera_info[camera] = info

def get_intrinsics(folder):
    model = image_geometry.PinholeCameraModel()
    for camera in camera_info:
        model.fromCameraInfo(camera_info[camera])
        print(f"Camera: {camera}\nFocal: [{model.fx()}, {model.fy()}]\nCenter: [{model.cx()}, {model.cy()}]\nDistortion: {str(model.distortionCoeffs())}")
        save_file = input("Save this file?")
        if save_file.lower() == 'y' or save_file.lower() == 'yes':
            print("Saving")
            save_in(path=folder + f"/{camera}.yaml", distort=model.distortionCoeffs(), matrix=model.intrinsicMatrix())
        else:
            print("Not saved")

def main(folder='data'):
    rospy.init_node("capture_cam_info",disable_signals=True)
    caminfo_sub = rospy.Subscriber("/oak/rgb/camera_info", CameraInfo, partial(info_callback, "oak"))
    stereoinfo_sub = rospy.Subscriber("/oak/stereo/camera_info", CameraInfo, partial(info_callback, "oak_stereo"))
    rightinfo_sub = rospy.Subscriber("/oak/right/camera_info", CameraInfo, partial(info_callback, "oak_right"))
    leftinfo_sub = rospy.Subscriber("/oak/left/camera_info", CameraInfo, partial(info_callback, "oak_left"))
    flinfo_sub = rospy.Subscriber("/camera_fl/arena_camera_node/camera_info", CameraInfo, partial(info_callback, "fl"))
    frinfo_sub = rospy.Subscriber("/camera_fr/arena_camera_node/camera_info", CameraInfo, partial(info_callback, "fr"))
    rlinfo_sub = rospy.Subscriber("/camera_rl/arena_camera_node/camera_info", CameraInfo, partial(info_callback, "rl"))
    rrinfo_sub = rospy.Subscriber("/camera_rr/arena_camera_node/camera_info", CameraInfo, partial(info_callback, "rr"))

    have_all = False
    while not have_all:
        have_all = True
        for camera in camera_info:
            if camera_info[camera] == None:
                have_all = False
                time.sleep(0.5)
                break
    get_intrinsics(folder)
    
if __name__ == '__main__':
    import sys
    folder = 'data'
    if len(sys.argv) >= 2:
        folder = sys.argv[1]
    main(folder)
