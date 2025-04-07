import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from readImageFromRosbag import readImageFromRosbag  # 你已有的函数


def publish_image_loop(image, topic="/oak/rgb/image_raw", frame_id="oak_camera", rate_hz=10):
    rospy.init_node("manual_image_publisher", anonymous=True)
    bridge = CvBridge()
    pub = rospy.Publisher(topic, Image, queue_size=1)
    
    msg = bridge.cv2_to_imgmsg(image, encoding="bgr8")
    msg.header.frame_id = frame_id

    rate = rospy.Rate(rate_hz)
    while not rospy.is_shutdown():
        msg.header.stamp = rospy.Time.now()
        pub.publish(msg)
        rate.sleep()

if __name__ == "__main__":
    bag_path = "/your/path/front.bag"
    image_topic = "/oak/rgb/image_raw"
    
    # 从 bag 中读取一张图
    image = readImageFromRosbag(bag_path, imageTopic=image_topic)
    
    # 循环持续发布该图像
    publish_image_loop(image, topic=image_topic, frame_id="oak_camera", rate_hz=10)
