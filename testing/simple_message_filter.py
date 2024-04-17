import math
import numpy as np
import cv2
import rospy
from typing import Dict,Tuple, List
import time
import os
from sensor_msgs.msg import Image,PointCloud2
from cv_bridge import CvBridge, CvBridgeError
import sensor_msgs.point_cloud2 as pc2
import message_filters

downsample_topic = '/ouster/points_downsample'
lidar_downsample_pub = rospy.Publisher(downsample_topic, PointCloud2, queue_size=10)

def pointcloud_downsample(pc2_msg, max_degree=120, min_degree=60):
    if pc2 is None:
        raise ImportError("ROS is not installed")
    # gen = pc2.read_points(pc2_msg, skip_nans=True)
    gen = pc2.read_points(pc2_msg, skip_nans=True, field_names=['x', 'y', 'z'])
    
    
    before_cnt = 0
    after_cnt = 0
    angles = []
    filtered_points = []
    for point in gen:
        pc_x, pc_y, pc_z = point[:3]
        angle = np.arctan2(pc_y, pc_x) * (180 / np.pi)
        before_cnt += 1
        # Check if the angle is within the specified range
        if pc_z < 0 and min_degree <= angle <= max_degree:
            angles.append(angle)
            after_cnt += 1
    
    print ('Before filtered cnt:', before_cnt)
    print ('After filtered cnt:', after_cnt)
    
    # Create a new PointCloud2 message with the filtered points
    downsample_cloud = pc2.create_cloud(pc2_msg.header, pc2_msg.fields, filtered_points)
    return downsample_cloud

def sync_callback(point_cloud, image):
    # Process synchronized messages here
    rospy.loginfo("Received synchronized messages.")
    point_cloud = point_cloud
    zed_image = image
    
def camera_callback(image: Image):
    None

def lidar_callback(point_cloud):
    downsample_cloud = pointcloud_downsample(point_cloud)
    lidar_downsample_pub.publish(downsample_cloud)
    print ('publish')

def separate_lidar_downsample_callback(point_cloud):
    print ('Get message in separate_lidar_downsample_callback')
    
e4_lidar_sub = rospy.Subscriber('/ouster/points', PointCloud2, lidar_callback)
# e4_camera_sub = rospy.Subscriber('/oak/rgb/image_raw', Image, camera_callback)


# Define the subscribers (message filters)
sync_lidar_topic = downsample_topic
# sync_lidar_topic = '/livox/lidar'

lidar_sub = rospy.Subscriber(sync_lidar_topic, PointCloud2, separate_lidar_downsample_callback)

sync_lidar_sub = message_filters.Subscriber(sync_lidar_topic, PointCloud2)
sync_camera_sub = message_filters.Subscriber('/oak/rgb/image_raw', Image)

# Synchronize messages based on their timestamps
ts = message_filters.ApproximateTimeSynchronizer([sync_lidar_sub, sync_camera_sub], 10, 0.1)
ts.registerCallback(sync_callback)

print ('Start ...')
rospy.init_node('sync_node')
rospy.spin()