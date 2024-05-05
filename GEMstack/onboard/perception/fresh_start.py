import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge, CvBridgeError
from ultralytics import YOLO
from pixelwise_3D_lidar_coord_handler import PixelWise3DLidarCoordHandler

MODEL_WEIGHT_PATH = '../../knowledge/detection/parking_spot_detection.pt'

class ImageProcessorNode:
    def __init__(self):
        self.model = YOLO(MODEL_WEIGHT_PATH)
        self.bridge = CvBridge()
        self.handler = PixelWise3DLidarCoordHandler()
        self.grey_thresh = 180
        self.conf_thresh = 0.83
        self.pub = rospy.Publisher("/annotated_lines", Image, queue_size=10)
        self.image_sub = rospy.Subscriber("/oak/rgb/image_raw", Image, self.image_callback)
        self.lidar_sub = rospy.Subscriber("/ouster/points", PointCloud2, self.lidar_callback)
        self.point_cloud = None
        self.image = None
    
    def lidar_callback(self, point_cloud):
        self.point_cloud = point_cloud

    def detect_empty(self, img):
        results = self.model(img)
        for box, conf in zip(results[0].obb, results[0].obb.conf):
            class_id = int(box.cls[0].item())
            confidence = float(conf.item())
            if class_id == 0 and confidence >= self.conf_thresh:
                x, y, w, h, r = box.xywhr[0].tolist()
                return (x, y, w, h, r)
        return None

    def get_rotated_box_points(self, x, y, width, height, angle):
        rectangle = np.array([[-width / 2, -height / 2], [width / 2, -height / 2],
                              [width / 2, height / 2], [-width / 2, height / 2]])
        rotation_matrix = np.array([[np.cos(angle), -np.sin(angle)],
                                    [np.sin(angle), np.cos(angle)]])
        rotated_rectangle = np.dot(rectangle, rotation_matrix) + np.array([x, y])
        return np.int0(rotated_rectangle)

    def isolate_and_draw_lines(self, img, bbox_info):
        if bbox_info is None:
            return img
        x, y, w, h, r = bbox_info
        points = self.get_rotated_box_points(x, y, w, h, -r)
        rect = cv2.boundingRect(points)
        x, y, w, h = rect
        cropped_img = img[y:y+h, x:x+w]
        rotation_matrix = cv2.getRotationMatrix2D((w/2, h/2), r, 1)
        aligned_img = cv2.warpAffine(cropped_img, rotation_matrix, (w, h))
        gray = cv2.cvtColor(aligned_img, cv2.COLOR_BGR2GRAY)
        _, thresholded = cv2.threshold(gray, self.grey_thresh, 255, cv2.THRESH_BINARY)
        lines = cv2.HoughLinesP(thresholded, 1, np.pi/180, threshold=50, minLineLength=40, maxLineGap=5)

        if lines is not None:
            lines = [line[0] for line in lines]
            line_params = []
            for x1, y1, x2, y2 in lines:
                if x2 != x1:
                    slope = (y2 - y1) / (x2 - x1)
                    y_intercept = y1 - slope * x1
                    line_params.append((slope, y_intercept, x1, y1, x2, y2))

            if len(line_params) >= 2:
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

                largest_groups = sorted(grouped_lines.values(), key=len, reverse=True)[:2]
                if len(largest_groups) >= 2:
                    avg_lines = []
                    for group in largest_groups:
                        avg_x1 = int(np.mean([line[2] for line in group]))
                        avg_y1 = int(np.mean([line[3] for line in group]))
                        avg_x2 = int(np.mean([line[4] for line in group]))
                        avg_y2 = int(np.mean([line[5] for line in group]))
                        avg_lines.append((avg_x1, avg_y1, avg_x2, avg_y2))

                    if len(avg_lines) == 2:
                        line1, line2 = avg_lines
                        mid_x1 = (line1[0] + line2[0]) // 2
                        mid_y1 = (line1[1] + line2[1]) // 2
                        mid_x2 = (line1[2] + line2[2]) // 2
                        mid_y2 = (line1[3] + line2[3]) // 2
                        midpoint = ((x + mid_x1 + x + mid_x2) // 2, (y + mid_y1 + y + mid_y2) // 2)

                        cv2.line(img, (x + mid_x1, y + mid_y1), (x + mid_x2, y + mid_y2), (0, 0, 255), 4)
                        for x1, y1, x2, y2 in avg_lines:
                            cv2.line(img, (x + x1, y + y1), (x + x2, y + y2), (0, 255, 0), 4)

            return img

    def image_callback(self, img_msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
        except CvBridgeError as e:
            print(e)
            return
        bbox_info = self.detect_empty(cv_image)
        if bbox_info:
            cv_image = self.isolate_and_draw_lines(cv_image, bbox_info)
        try:
            ros_image = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
            self.pub.publish(ros_image)
        except CvBridgeError as e:
            print(e)

    def run(self):
        rospy.init_node('image_processor', anonymous=True)
        rospy.spin()

if __name__ == '__main__':
    processor_node = ImageProcessorNode()
    processor_node.run()