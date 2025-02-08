#!/usr/bin/env python
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def webcam_publisher():
    rospy.init_node('webcam_publisher', anonymous=True)
    
    image_pub = rospy.Publisher('/webcam', Image, queue_size=10)
    
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        rospy.logerr("cannot open camera")
        return
    
    bridge = CvBridge()
    
    rate = rospy.Rate(30)  # 30 Hz
    
    try:
        while not rospy.is_shutdown():
            ret, frame = cap.read()
            if not ret:
                rospy.logerr("cannnot read frames")
                break
            
            cv2.imshow("Webcam", frame)
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                rospy.loginfo("press q to exit")
                break
            
            try:
                ros_image = bridge.cv2_to_imgmsg(frame, "bgr8")
            except Exception as e:
                rospy.logerr(f"failed to convert image: {e}")
                continue
            
            image_pub.publish(ros_image)
            rospy.loginfo("publish frame")
            
            rate.sleep()
    
    except KeyboardInterrupt:
        rospy.loginfo("user key interrupt")
    
    finally:
        cap.release()
        cv2.destroyAllWindows()
        rospy.loginfo("freed camera resources")

if __name__ == '__main__':
    webcam_publisher()