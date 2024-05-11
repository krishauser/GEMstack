#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

def webcam_publisher():
    # Initialize the ROS Node
    rospy.init_node('webcam_publisher', anonymous=True)
    # Create a Publisher object, publishing to the 'camera/image' topic
    pub = rospy.Publisher('zed2/zed_node/rgb/image_rect_color', Image, queue_size=10)
    # Set the loop rate
    rate = rospy.Rate(10)  # 10hz
    # Initialize the CvBridge
    bridge = CvBridge()

    # Capture video from the webcam (usually the webcam is on video0)
    cap = cv2.VideoCapture(0)

    while not rospy.is_shutdown():
        # Capture frame-by-frame
        ret, frame = cap.read()

        if ret:
            try:
                # Convert the OpenCV image to a ROS Image message
                ros_image = bridge.cv2_to_imgmsg(frame, "bgr8")
                # Publish the ROS Image message
                pub.publish(ros_image)
            except CvBridgeError as e:
                print(e)

        # Sleep for the remainder of the loop rate
        rate.sleep()

    # When everything done, release the capture
    cap.release()

if __name__ == '__main__':
    try:
        webcam_publisher()
    except rospy.ROSInterruptException:
        pass
