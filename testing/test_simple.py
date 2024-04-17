import rospy
import message_filters
from sensor_msgs.msg import Image, PointCloud2

rospy.init_node("test")

def callback(pc_msg, img_msg):
    print("Hello World!")

# Define the subscribers (message filters)
sync_lidar_sub = message_filters.Subscriber("/ouster/points", PointCloud2)
sync_camera_sub = message_filters.Subscriber("/oak/rgb/image_raw", Image)

# Synchronize messages based on their timestamps
ts = message_filters.ApproximateTimeSynchronizer([sync_lidar_sub, sync_camera_sub], queue_size=10, slop=1)
ts.registerCallback(callback)

rospy.spin()