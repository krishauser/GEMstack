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
import ros_numpy

lidar_front_points = None
lidar_top_points = None
front_cam = None
camera_image = None
depth = None
camera_right_img = None
camera_r_right_img = None
camera_r_left = None
bridge = CvBridge()

def camera_callback(img : Image):
    global front_cam
    front_cam = img


def lidar_front_callback(lidar : PointCloud2):
    global lidar_front_points
    lidar_front_points = lidar

def lidar_top_callback(lidar : PointCloud2):
    global lidar_top_points
    lidar_top_points = lidar

def camera_left_callback(img : Image):
    global camera_image
    camera_image = img

def camera_right_callback(img : Image):
    global camera_right_img
    camera_right_img = img

def camera_rear_left_callback(img : Image):
    global camera_r_left
    camera_r_left = img

def camera_rear_right_callback(img : Image):
    global camera_r_right_img
    camera_r_right_img = img

def pc2_to_numpy(pc2_msg, want_rgb=False):
    """
    Convert a ROS PointCloud2 message into a numpy array quickly using ros_numpy.
    This function extracts the x, y, z coordinates from the point cloud.
    """
    # Convert the ROS message to a numpy structured array
    pc = ros_numpy.point_cloud2.pointcloud2_to_array(pc2_msg)
    # Convert each field to a 1D array and stack along axis 1 to get (N, 3)
    pts = np.stack((np.array(pc['x']).ravel(),
                    np.array(pc['y']).ravel(),
                    np.array(pc['z']).ravel()), axis=1)
    # Apply filtering (for example, x > 0 and z < 2.5)
    mask = (pts[:, 0] > 0) & (pts[:, 2] < 2.5)
    return pts[mask]


def save_scan(left,right,rear_left, rear_right, front, front_lidar, top_lidar):
    cv2.imwrite(left,bridge.imgmsg_to_cv2(camera_image))
    cv2.imwrite(right,bridge.imgmsg_to_cv2(camera_right_img))
    cv2.imwrite(rear_left,bridge.imgmsg_to_cv2(camera_r_left))
    cv2.imwrite(rear_right,bridge.imgmsg_to_cv2(camera_r_right_img))
    cv2.imwrite(front,bridge.imgmsg_to_cv2(front_cam))
    pc = pc2_to_numpy(lidar_front_points,want_rgb=False) # convert lidar feed to numpy
    np.savez(front_lidar,pc)

    pc_top = pc2_to_numpy(lidar_top_points,want_rgb=False) # convert lidar feed to numpy
    np.savez(top_lidar,pc_top)




def main(folder='data',start_index=1, frequency=2):
    rospy.init_node("capture_triton_l_r",disable_signals=True)
    top_lidar_sub = rospy.Subscriber("/ouster/points", PointCloud2, lidar_top_callback)
    front_lidar_sub = rospy.Subscriber("/livox/lidar", PointCloud2, lidar_front_callback)
    front_cam_sub = rospy.Subscriber("/oak/rgb/image_raw", Image, camera_callback)
    camera_sub_left = rospy.Subscriber("/camera_fl/arena_camera_node/image_raw", Image, camera_left_callback)
    camera_sub_right = rospy.Subscriber("/camera_fr/arena_camera_node/image_raw", Image, camera_right_callback)
    camera_rear_sub_left = rospy.Subscriber("/camera_rl/arena_camera_node/image_raw", Image, camera_rear_left_callback)
    camera_rear_sub_right = rospy.Subscriber("/camera_rr/arena_camera_node/image_raw", Image, camera_rear_right_callback)
    index = start_index
    print(" Storing images as png")
    print(" Ctrl+C to quit")
    while True:
        if camera_image and camera_right_img and camera_r_right_img and camera_r_left and front_cam and lidar_front_points and lidar_top_points:
            cv2.imshow("result",bridge.imgmsg_to_cv2(camera_image))
            # time.sleep(1.0/frequency)
            files = [
                        os.path.join(folder,'camera_fl{}.png'.format(index)),
                        os.path.join(folder,'camera_fr{}.png'.format(index)), 
                        os.path.join(folder,'camera_rl{}.png'.format(index)),
                        os.path.join(folder,'camera_rr{}.png'.format(index)),
                        os.path.join(folder,'front_cam{}.png'.format(index)),
                        os.path.join(folder,'lidar_front{}.npz'.format(index)),
                        os.path.join(folder,'lidar_top{}.npz'.format(index))
                        ]
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
