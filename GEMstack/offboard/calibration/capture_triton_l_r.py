# ROS Headers
import rospy
from sensor_msgs.msg import Image,PointCloud2
import sensor_msgs.point_cloud2 as pc2
import ctypes
import struct
import argparse


# OpenCV and cv2 bridge
import cv2
from cv_bridge import CvBridge
import numpy as np
import os
import time

lidar_points = None
camera_image = None
depth = None
camera_right_img = None
bridge = CvBridge()

def camera_left_callback(img : Image):
    global camera_image
    camera_image = img

def camera_right_callback(img : Image):
    global camera_right_img
    camera_right_img = img

def save_scan(left,right):
    cv2.imwrite(left,bridge.imgmsg_to_cv2(camera_image))
    cv2.imwrite(right,bridge.imgmsg_to_cv2(camera_right_img))


def main(folder='data',start_index=1, frequency=2):
    rospy.init_node("capture_triton_l_r",disable_signals=True)
    camera_sub_left = rospy.Subscriber("/camera_fl/arena_camera_node/image_raw", Image, camera_left_callback)
    camera_sub_right = rospy.Subscriber("/camera_fr/arena_camera_node/image_raw", Image, camera_right_callback)
    index = start_index
    print(" Storing images as png")
    print(" Ctrl+C to quit")
    while True:
        if camera_image and camera_right_img:
            cv2.imshow("result",bridge.imgmsg_to_cv2(camera_image))
            time.sleep(1.0/frequency)
            files = [
                        os.path.join(folder,'left{}.png'.format(index)),
                        os.path.join(folder,'right{}.png'.format(index))]
            save_scan(*files)
            index += 1

if __name__ == '__main__':
    import sys
    parser = argparse.ArgumentParser(description='Capture LiDAR and camera data.')
    parser.add_argument('--folder', type=str, default='data', help='Directory to store data')
    parser.add_argument('--start_index', type=int, default=1, help='Starting index for saved files')
    parser.add_argument('--frequency', type=float, default=2.0, help='Capture frequency in Hz')
    args = parser.parse_args()
    main(args.folder, args.start_index, args.frequency)
