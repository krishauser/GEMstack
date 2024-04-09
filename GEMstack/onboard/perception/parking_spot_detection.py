from ultralytics import YOLO
import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

MODEL_WEIGHT_PATH = '../../knowledge/detection/last.pt'
model = YOLO(MODEL_WEIGHT_PATH)
bridge = CvBridge()

def empty_detect(img: cv2.Mat, empty_spot=0):
    global model
    results = model(img)
    bboxes = []

    for box in results[0].boxes:
        class_id = int(box.cls[0].item())
        if class_id == empty_spot:
            bboxes.append(box.xywh[0].tolist())
    return bboxes

def image_callback(msg):
    global bridge
    try:
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
        print(e)
    
    bboxes = empty_detect(cv_image)
    print("Detected", len(bboxes), "empty spot")
    for bb in bboxes:
        x, y, w, h = bb
        print("Corner", (x, y), "size", (w, h))
        cv2.rectangle(cv_image, (int(x-w/2), int(y-h/2)), (int(x+w/2), int(y+h/2)), (255, 0, 255), 3)
    
    # Convert the annotated image back to a ROS message and publish it
    try:
        ros_image = bridge.cv2_to_imgmsg(cv_image, "bgr8")
        image_pub.publish(ros_image)
    except CvBridgeError as e:
        print(e)

if __name__ == '__main__':
    rospy.init_node('image_listener', anonymous=True)
    
    # Subscriber
    image_topic = "/oak/right/image_raw"  # Change to your ROS image topic
    rospy.Subscriber(image_topic, Image, image_callback)
    
    # Publisher
    annotated_image_topic = "/oak/right/annotated"  # Change to your desired ROS topic for the annotated images
    image_pub = rospy.Publisher(annotated_image_topic, Image, queue_size=10)
    
    rospy.spin()
