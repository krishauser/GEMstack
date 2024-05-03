import os
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge, CvBridgeError
from ultralytics import YOLO
from pixelwise_3D_lidar_coord_handler import PixelWise3DLidarCoordHandler
import sensor_msgs.point_cloud2 as pc2

sobel_kernel_size = 3
sobel_min_threshold = 90
conf_val = 0.835
MODEL_WEIGHT_PATH = '../../knowledge/detection/parking_spot_detection.pt'

class ImageProcessorNode:
    def __init__(self):
        self.bridge = CvBridge()
        self.model = YOLO(MODEL_WEIGHT_PATH)
        self.handler = PixelWise3DLidarCoordHandler()
        self.image_pub = rospy.Publisher("/oak/rgb/modified_image", Image, queue_size=10)
        self.image_sub = rospy.Subscriber("/oak/rgb/image_raw", Image, self.image_callback)
        self.lidar_sub = rospy.Subscriber("/ouster/points", PointCloud2, self.lidar_callback)
        self.point_cloud = None
        self.goal_poses = []

    def lidar_callback(self, point_cloud):
        self.point_cloud = point_cloud

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print("CvBridge Error:", e)
            return

        if self.point_cloud is None:
            return  # Wait until the point cloud is available

        self.process_image(cv_image)

    def process_image(self, cv_image):
        canvas = cv_image.copy()
        bbox_info = self.detect_empty(cv_image)
        if bbox_info:
            self.apply_detections(canvas, bbox_info)

        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(canvas, "bgr8"))
        except CvBridgeError as e:
            print("CvBridge Error during modified image publishing:", e)

    def detect_empty(self, img):
        results = self.model(img)
        for box, conf in zip(results[0].obb, results[0].obb.conf):
            class_id, confidence = int(box.cls[0].item()), float(conf.item())
            if class_id == 0 and confidence >= conf_val:
                x, y, w, h, r = box.xywhr[0].tolist()
                return (x, y, w, h, r)            
        return None

    def apply_detections(self, canvas, bbox_info):
        if bbox_info is None:
            return canvas
        x, y, w, h, r = bbox_info
        points = self.get_rotated_box_points(x, y, w, h, -r)
        self.green_lines(canvas, points,r)
        self.middle_line_3d_coords(canvas, points,r)
        return canvas

    def get_rotated_box_points(self, x, y, width, height, angle):
        rectangle = np.array([[-width / 2, -height / 2], [width / 2, -height / 2],
                              [width / 2, height / 2], [-width / 2, height / 2]])
        rotation_matrix = np.array([[np.cos(angle), -np.sin(angle)],
                                    [np.sin(angle), np.cos(angle)]])
        rotated_rectangle = np.dot(rectangle, rotation_matrix) + np.array([x, y])
        return np.int0(rotated_rectangle)

    def green_lines(self, canvas, points, angle):
        angle_threshold = np.pi / 4  # 45 degrees threshold

        if abs(angle) > angle_threshold:
            # Use right and left lines
            left_lines = np.array([points[3], points[2]])
            right_lines = np.array([points[0], points[1]])
        else:
            # Use top and bottom lines
            left_lines = np.array([points[0], points[3]])
            right_lines = np.array([points[1], points[2]])

        # Draw green lines
        cv2.line(canvas, tuple(right_lines[0]), tuple(right_lines[1]), (0, 255, 0), 2)
        cv2.line(canvas, tuple(left_lines[0]), tuple(left_lines[1]), (0, 255, 255), 2)

    def middle_line_3d_coords(self, canvas, points, angle):
        angle_threshold = np.pi / 4  # 45 degrees threshold

        if abs(angle) > angle_threshold:
            # Calculate mid points of left and right lines
            top_mid = ((points[3][0] + points[0][0]) // 2, (points[3][1] + points[0][1]) // 2)
            bottom_mid = ((points[2][0] + points[1][0]) // 2, (points[2][1] + points[1][1]) // 2)
        else:
            # Calculate mid points of top and bottom lines
            top_mid = ((points[0][0] + points[1][0]) // 2, (points[0][1] + points[1][1]) // 2)
            bottom_mid = ((points[3][0] + points[2][0]) // 2, (points[3][1] + points[2][1]) // 2)

        # Draw the red line between the new midpoints
        cv2.line(canvas, top_mid, bottom_mid, (0, 0, 255), 2)
        angle_new = np.pi/2 - angle
        # Find the midpoint of red line
        line_mid = ((top_mid[0] + bottom_mid[0]) // 2, (top_mid[1] + bottom_mid[1]) // 2)

        # Use handler to get 3D coordinates of the red line midpoint
        if self.point_cloud:
            numpy_point_cloud = self.handler.ros_PointCloud2_to_numpy(self.point_cloud)
            coord_3d_map = self.handler.get3DCoord(canvas, numpy_point_cloud)
            if 0 <= line_mid[0] < coord_3d_map.shape[1] and 0 <= line_mid[1] < coord_3d_map.shape[0]:
                red_line_mid_3d = coord_3d_map[line_mid[1], line_mid[0]]
                x = red_line_mid_3d[0]
                y = red_line_mid_3d[1]
                yaw = angle_new
                print(f"3D coordinates of the red line's midpoint: [{x}, {y}, {yaw}]")
                self.goal_poses.append([x,y,yaw])
        return canvas

if __name__ == '__main__':
    rospy.init_node('image_processor', anonymous=True)
    processor_node = ImageProcessorNode()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")