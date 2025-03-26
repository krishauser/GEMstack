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

oak_info = None
stereo_info = None
right_info = None
left_info = None
fl_info = None
fr_info = None
rl_info = None
rr_info = None

def oak_callback(info : CameraInfo):
    global oak_info
    oak_info = info

def stereo_callback(info : CameraInfo):
    global stereo_info
    stereo_info = info

def right_callback(info : CameraInfo):
    global right_info
    right_info = info

def left_callback(info : CameraInfo):
    global left_info
    left_info = info

def fl_callback(info : CameraInfo):
    global fl_info
    fl_info = info

def fr_callback(info : CameraInfo):
    global fr_info
    fr_info = info

def rl_callback(info : CameraInfo):
    global rl_info
    rl_info = info

def rr_callback(info : CameraInfo):
    global rr_info
    rr_info = info

def get_intrinsics(folder):
    model = image_geometry.PinholeCameraModel()
    model.fromCameraInfo(oak_info)
    with open(os.path.join(folder, "oak.txt"), "w") as f:
        f.write(str(model.intrinsicMatrix()))
    model.fromCameraInfo(stereo_info)
    with open(os.path.join(folder, "stereo.txt"), "w") as f:
        f.write(str(model.intrinsicMatrix()))
    model.fromCameraInfo(right_info)
    with open(os.path.join(folder, "right.txt"), "w") as f:
        f.write(str(model.intrinsicMatrix()))
    model.fromCameraInfo(left_info)
    with open(os.path.join(folder, "left.txt"), "w") as f:
        f.write(str(model.intrinsicMatrix()))
    model.fromCameraInfo(fl_info)
    with open(os.path.join(folder, "fl.txt"), "w") as f:
        f.write(str(model.intrinsicMatrix()))
    model.fromCameraInfo(fr_info)
    with open(os.path.join(folder, "fr.txt"), "w") as f:
        f.write(str(model.intrinsicMatrix()))
    model.fromCameraInfo(rl_info)
    with open(os.path.join(folder, "rl.txt"), "w") as f:
        f.write(str(model.intrinsicMatrix()))
    model.fromCameraInfo(rr_info)
    with open(os.path.join(folder, "rr.txt"), "w") as f:
        f.write(str(model.intrinsicMatrix()))

def main(folder='data'):
    rospy.init_node("capture_cam_info",disable_signals=True)
    caminfo_sub = rospy.Subscriber("/oak/rgb/camera_info", CameraInfo, oak_callback)
    stereoinfo_sub = rospy.Subscriber("/oak/stereo/camera_info", CameraInfo, stereo_callback)
    rightinfo_sub = rospy.Subscriber("/oak/right/camera_info", CameraInfo, right_callback)
    leftinfo_sub = rospy.Subscriber("/oak/left/camera_info", CameraInfo, left_callback)
    flinfo_sub = rospy.Subscriber("/camera_fl/arena_camera_node/camera_info", CameraInfo, fl_callback)
    frinfo_sub = rospy.Subscriber("/camera_fr/arena_camera_node/camera_info", CameraInfo, fr_callback)
    rlinfo_sub = rospy.Subscriber("/camera_rl/arena_camera_node/camera_info", CameraInfo, rl_callback)
    rrinfo_sub = rospy.Subscriber("/camera_rr/arena_camera_node/camera_info", CameraInfo, rr_callback)
    while True:
        if oak_info and fl_info and fr_info and rl_info and rr_info:
            time.sleep(1)
            get_intrinsics(folder)

if __name__ == '__main__':
    import sys
    folder = 'data'
    if len(sys.argv) >= 2:
        folder = sys.argv[1]
    main(folder)
