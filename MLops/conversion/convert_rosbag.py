import os
import rosbag
import cv2
from cv_bridge import CvBridge

def convert_rosbag(bag_file):
    path = os.path.join(os.getcwd(), 'output')
    os.makedirs(path, exist_ok=True)

    bag = rosbag.Bag(bag_file)
    topics = bag.get_type_and_topic_info().topics
    for topic in topics:
        if 'image' in topics[topic].msg_type.lower():
            to_image(bag, topic)
            
    bag.close()

def to_image(bag, topic):
    bridge = CvBridge()
    path = os.path.join(os.getcwd(), 'output/' + topic[1:].replace('/', '_'))
    os.makedirs(path, exist_ok=True)

    for _, msg, t in bag.read_messages(topics=[topic]):
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        cv2.imwrite(os.path.join(path, str(t) + '.png'), cv_image)

# sensor_msgs/Image
# sensor_msgs/PointCloud2
# sensor_msgs/NavSatFix
# sensor_msgs/NavSatStatus
# convert_rosbag('left_parking_detect.bag')
