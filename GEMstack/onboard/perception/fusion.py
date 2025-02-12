from cv_bridge import CvBridge
from sensor_msgs.msg import Image, PointCloud2
import rospy
import message_filters


class Fusion3D():
    def __init__(self):
        self.bridge = CvBridge()
        self.stereo_rosbag = message_filters.Subscriber('/oak/stereo/image_raw', Image, self.stereo_callback, queue_size=1)
        self.top_lidar_rosbag = message_filters.Subscriber('/ouster/points', PointCloud2, self.top_lidar_callback, queue_size=1)

        ts = message_filters.ApproximateTimeSynchronizer([self.stereo_rosbag, self.top_lidar_rosbag], queue_size=10, slop=0.1)
        ts.registerCallback(self.callback)
        rospy.spin()

    def callback(left_img, right_img, lidar_msg, camera_info):
    # Process synchronized data here
        pass

    def stereo_callback(self, image: Image):
        image = self.bridge.imgmsg_to_cv2(image, "16UC1")
        print(f"stereo: {image.shape}")        

    def top_lidar_callback(self, point_cloud: PointCloud2):
        # point_cloud = self.bridge.imgmsg_to_cv2(point_cloud, "16UC1")
        print(f"point_cloud: {point_cloud.fields}")       

if __name__ == '__main__':
    rospy.init_node('fusion_node', anonymous=True)
    Fusion3D()
    while not rospy.core.is_shutdown():
        rospy.rostime.wallsleep(0.5)

