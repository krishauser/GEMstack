# import rospy
# import message_filters
# from sensor_msgs.msg import Image, PointCloud2

# rospy.init_node("test")

# def callback(pc_msg, img_msg):
#     print("Hello World!")

# # Define the subscribers (message filters)
# sync_lidar_sub = message_filters.Subscriber("/ouster/points_downsample", PointCloud2)
# sync_camera_sub = message_filters.Subscriber("/oak/rgb/image_raw", Image)

# # Synchronize messages based on their timestamps
# ts = message_filters.ApproximateTimeSynchronizer([sync_lidar_sub, sync_camera_sub], queue_size=3, slop=1)
# ts.registerCallback(callback)

# rospy.spin()

import rospy
import message_filters
from sensor_msgs.msg import Image, CameraInfo, PointCloud2

rospy.init_node('sync_node')

lidar_republisher = rospy.Publisher('/ouster/points_with_timestamp', PointCloud2, queue_size=30)

def add_stamp_to_lidar(lidar_msg):
    print("receive outster points")
    stamped_msg = lidar_msg
    stamped_msg.header.stamp = rospy.Time.now()
    stamped_msg.header.frame_id = 'oo_link'
    lidar_republisher.publish(stamped_msg)
    print("publish outster time_stamped points")

def callback(image_msg, camera_info_msg, stamped_lidar_msg):
    # Do your thing here
    print("hello world!")
    pass

lidar_sub = rospy.Subscriber('/ouster/points', PointCloud2, add_stamp_to_lidar)
# Process synchronized data here
image_sub = message_filters.Subscriber('/oak/rgb/image_raw', Image)
stamped_lidar_sub = message_filters.Subscriber('/ouster/points_with_timestamp', PointCloud2)
ts = message_filters.ApproximateTimeSynchronizer([image_sub, stamped_lidar_sub], 30, 10)
ts.registerCallback(callback)


rospy.spin()