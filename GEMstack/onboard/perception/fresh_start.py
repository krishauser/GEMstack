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
        lines = [line[0] for line in lines]  # Flatten the line list
        # Calculate the slope and y-intercept for each line
        line_params = []
        for x1, y1, x2, y2 in lines:
            if x2 != x1:
                slope = (y2 - y1) / (x2 - x1)
                y_intercept = y1 - slope * x1
                line_params.append((slope, y_intercept, x1, y1, x2, y2))
        
        # Filter and group lines by their slope and proximity to group parallel lines
        if len(line_params) >= 2:
            # Group by slope with a tolerance, and then find the two groups with the most lines
            slope_tolerance = 0.001
            grouped_lines = {}
            for params in line_params:
                matched = False
                for key in list(grouped_lines.keys()):
                    if abs(key - params[0]) < slope_tolerance:
                        grouped_lines[key].append(params)
                        matched = True
                        break
                if not matched:
                    grouped_lines[params[0]] = [params]

            # Select the two largest groups representing the most found parallel lines
            largest_groups = sorted(grouped_lines.values(), key=len, reverse=True)[:2]
            if len(largest_groups) >= 2:
                # Calculate average lines from these groups
                avg_lines = []
                for group in largest_groups:
                    avg_x1 = int(np.mean([line[2] for line in group]))
                    avg_y1 = int(np.mean([line[3] for line in group]))
                    avg_x2 = int(np.mean([line[4] for line in group]))
                    avg_y2 = int(np.mean([line[5] for line in group]))
                    avg_lines.append((avg_x1, avg_y1, avg_x2, avg_y2))
                
                # Calculate the middle line points by averaging the endpoints of the average lines
                if len(avg_lines) == 2:
                    line1, line2 = avg_lines
                    mid_x1 = (line1[0] + line2[0]) // 2
                    mid_y1 = (line1[1] + line2[1]) // 2
                    mid_x2 = (line1[2] + line2[2]) // 2
                    mid_y2 = (line1[3] + line2[3]) // 2

                    # Draw the middle line on the original image in red
                    cv2.line(img, (x + mid_x1, y + mid_y1), (x + mid_x2, y + mid_y2), (0, 0, 255), 4)

                    # Optional: Draw the original two average lines in green for reference
                    for x1, y1, x2, y2 in avg_lines:
                        cv2.line(img, (x + x1, y + y1), (x + x2, y + y2), (0, 255, 0), 4)

    return img

def image_callback(img_msg, pub):
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