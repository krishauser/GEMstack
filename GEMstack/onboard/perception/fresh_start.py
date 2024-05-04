from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import rospy
import cv2
import numpy as np
from ultralytics import YOLO

MODEL_WEIGHT_PATH = '../../knowledge/detection/parking_spot_detection.pt'
model = YOLO(MODEL_WEIGHT_PATH)
bridge = CvBridge()

def get_rotated_box_points(x, y, width, height, angle):
    rectangle = np.array([[-width / 2, -height / 2], [width / 2, -height / 2],
                          [width / 2, height / 2], [-width / 2, height / 2]])
    rotation_matrix = np.array([[np.cos(angle), -np.sin(angle)],
                                [np.sin(angle), np.cos(angle)]])
    rotated_rectangle = np.dot(rectangle, rotation_matrix) + np.array([x, y])
    return np.int0(rotated_rectangle)

def empty_detect(img: cv2.Mat, empty_spot=0, conf_threshold=0.835):
    global model
    results = model(img)
    for box, conf in zip(results[0].obb, results[0].obb.conf):
        class_id = int(box.cls[0].item())
        confidence = float(conf.item())
        if class_id == empty_spot and confidence >= conf_threshold:
            x, y, w, h, r = box.xywhr[0].tolist()
            return (x, y, w, h, r)  
    return None 

def isolate_and_draw_lines(img, bbox_info):
    if bbox_info is None:
        return img

    x, y, w, h, r = bbox_info
    points = get_rotated_box_points(x, y, w, h, -r)
    rect = cv2.boundingRect(points)
    x, y, w, h = rect
    cropped_img = img[y:y+h, x:x+w]
    rotation_matrix = cv2.getRotationMatrix2D((w/2, h/2), r, 1)
    aligned_img = cv2.warpAffine(cropped_img, rotation_matrix, (w, h))
    gray = cv2.cvtColor(aligned_img, cv2.COLOR_BGR2GRAY)
    _, thresholded = cv2.threshold(gray, 155, 255, cv2.THRESH_BINARY)
    lines = cv2.HoughLinesP(thresholded, 1, np.pi/180, threshold=50, minLineLength=40, maxLineGap=5)
    if lines is not None:
        for line in lines:
            for x1, y1, x2, y2 in line:
                cv2.line(img, (x+x1, y+y1), (x+x2, y+y2), (0, 255, 0), 4)
    return img

def image_callback(img_msg, pub):
    global model
    try:
        cv_image = bridge.imgmsg_to_cv2(img_msg, "bgr8")
    except CvBridgeError as e:
        print(e)
        return

    bbox_info = empty_detect(cv_image)
    if bbox_info:
        cv_image = isolate_and_draw_lines(cv_image, bbox_info)

    try:
        ros_image = bridge.cv2_to_imgmsg(cv_image, "bgr8")
        pub.publish(ros_image)
    except CvBridgeError as e:
        print(e)


def main():
    rospy.init_node('image_processor', anonymous=True)
    pub = rospy.Publisher("/annotated_lines", Image, queue_size=10)
    rospy.Subscriber("/oak/rgb/image_raw", Image, lambda img_msg: image_callback(img_msg, pub))
    rospy.spin()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()