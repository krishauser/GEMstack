from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import rospy
import cv2
import numpy as np
from ultralytics import YOLO

MODEL_WEIGHT_PATH = '../../knowledge/detection/parking_spot_detection.pt'
model = YOLO(MODEL_WEIGHT_PATH)
bridge = CvBridge()

# Define the rest of your functions here (get_rotated_box_points, empty_detect)

def get_rotated_box_points(x, y, width, height, angle):
    rectangle = np.array([[-width / 2, -height / 2], [width / 2, -height / 2],
                          [width / 2, height / 2], [-width / 2, height / 2]])
    rotation_matrix = np.array([[np.cos(angle), -np.sin(angle)],
                                [np.sin(angle), np.cos(angle)]])
    rotated_rectangle = np.dot(rectangle, rotation_matrix) + np.array([x, y])
    return np.int0(rotated_rectangle)

def empty_detect(img: cv2.Mat, empty_spot=0, conf_threshold=0.835):
    global model, bbox_id_counter
    results = model(img)
    for box, conf in zip(results[0].obb, results[0].obb.conf):
        class_id = int(box.cls[0].item())
        confidence = float(conf.item())
        if class_id == empty_spot and confidence >= conf_threshold:
            x, y, w, h, r = box.xywhr[0].tolist()
            print("angle:",r)
            return (x, y, w, h, r)  
    return None 

def image_callback(img_msg, pub):
    global model
    try:
        cv_image = bridge.imgmsg_to_cv2(img_msg, "bgr8")
    except CvBridgeError as e:
        print(e)

    bbox_info = empty_detect(cv_image)
    if bbox_info:
        x, y, w, h, r = bbox_info
        points = get_rotated_box_points(x, y, w, h, -r)
        # Draw all sides with one color
        cv2.polylines(cv_image, [points], isClosed=True, color=(255, 0, 255), thickness=3)
        # Highlight the two lines aligned with the rotation axis in different color
        cv2.line(cv_image, tuple(points[0]), tuple(points[1]), (0, 255, 0), 3)  # Aligned with rotation axis
        cv2.line(cv_image, tuple(points[2]), tuple(points[3]), (0, 255, 0), 3)  # Aligned with rotation axis
        # Add an arrow indicating the positive direction of the rotation axis
        mid_point_start = (points[0] + points[1]) // 2
        mid_point_end = (points[2] + points[3]) // 2
        cv2.arrowedLine(cv_image, tuple(mid_point_start), tuple(mid_point_end), (0, 0, 255), 3, tipLength=0.5)

    try:
        ros_image = bridge.cv2_to_imgmsg(cv_image, "bgr8")
        pub.publish(ros_image)
    except CvBridgeError as e:
        print(e)

def main():
    rospy.init_node('image_processor', anonymous=True)
    pub = rospy.Publisher("/annotated", Image, queue_size=10)
    rospy.Subscriber("/oak/rgb/image_raw", Image, lambda img_msg: image_callback(img_msg, pub))
    rospy.spin()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()