import rospy
import message_filters
from sensor_msgs.msg import Image, PointCloud2

rospy.init_node('sync_node')

lidar_republisher = rospy.Publisher('/ouster/points', PointCloud2, queue_size=1)

def add_stamp_to_lidar(lidar_msg):
    print("receive outster points")
    stamped_msg = lidar_msg
    stamped_msg.header.stamp = rospy.Time.now()
    stamped_msg.header.frame_id = 'oo_link'
    lidar_republisher.publish(stamped_msg)
    print("publish outster time_stamped points")

def callback(image_msg, stamped_lidar_msg):
    print("Reveice in message_filter callback!")

# lidar_sub = rospy.Subscriber('/ouster/points', PointCloud2, add_stamp_to_lidar)
# Process synchronized data here
image_sub = message_filters.Subscriber('/oak/rgb/image_raw', Image)
lidar_sub = message_filters.Subscriber('/ouster/points', PointCloud2)
ts = message_filters.ApproximateTimeSynchronizer([image_sub, lidar_sub], 3, 0.1)
ts.registerCallback(callback)


rospy.spin()