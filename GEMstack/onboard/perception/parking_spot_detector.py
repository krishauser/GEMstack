from sensor_msgs.msg import Image
import os
import cv2
import torch
import rospy
import message_filters

try:
    from cv_bridge import CvBridge
    cv_bridge = CvBridge()
except ImportError:
    cv_bridge = None

def ros_Image_to_cv2(msg, desired_encoding="passthrough"):
    if cv_bridge is None:
        raise ImportError("cv_bridge is not installed")
    return cv_bridge.imgmsg_to_cv2(msg, desired_encoding=desired_encoding)

def cv2_to_ros_Image(cv2_img):
    if cv_bridge is None:
        raise ImportError("cv_bridge is not installed")
    return cv_bridge.cv2_to_imgmsg(cv2_img)

class ParkingSpotDetector():
    def __init__(self) -> None:
        # Variables
        self.model_path = os.getcwd() + '/GEMstack/knowledge/detection/cone_detector.pt'
        self.model = torch.hub.load('ultralytics/yolov5', 'custom', path=self.model_path) 
        self.model.eval()
        self.visualization = True

        # Subscribers and sychronizers
        self.fl_camera_msg = message_filters.Subscriber('/camera/fl/image_raw', Image)
        self.fr_camera_msg = message_filters.Subscriber('/camera/fr/image_raw', Image)
        self.sync = message_filters.ApproximateTimeSynchronizer([self.fl_camera_msg, self.fr_camera_msg], queue_size=10, slop=0.1)
        self.sync.registerCallback(self.cameras_callback)

        # Publishers
        if(self.visualization):
            self.pub_cone_detections = rospy.Publisher("/detection/annotated_cones", Image, queue_size=1)

    def cameras_callback(self, fl_camera_msg: Image, fr_camera_msg: Image):
        fl_cv_image = ros_Image_to_cv2(fl_camera_msg, desired_encoding="bgr8")
        results = self.model(fl_cv_image, augment=True).pred[0]

        # Draw bounding boxes
        for det in results:
            x1, y1, x2, y2, _, cls = map(int, det[:6])  # Extract values
            label = f'{self.model.names[int(cls)]}'  # Class name + confidence

            # Draw rectangle
            cv2.rectangle(fl_cv_image, (x1, y1), (x2, y2), (0, 255, 0), 2)  # Green box
            cv2.putText(fl_cv_image, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

        # Save or display the image
        cone_detections_ros_img = cv2_to_ros_Image(fl_cv_image)
        self.pub_cone_detections.publish(cone_detections_ros_img)

if __name__ == '__main__':
    rospy.init_node('parking_spot_detector_node', anonymous=True)
    ParkingSpotDetector()
    while not rospy.core.is_shutdown():
        rospy.rostime.wallsleep(0.5)