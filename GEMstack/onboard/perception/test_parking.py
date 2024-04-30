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
conf_val = 0.85
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

    def ros_PointCloud2_to_numpy(self, pc2_msg, want_rgb=False):
        if pc2 is None:
            raise ImportError("ROS is not installed")
        # gen = pc2.read_points(pc2_msg, skip_nans=True)
        gen = pc2.read_points(pc2_msg, skip_nans=True, field_names=['x', 'y', 'z'])

        if want_rgb:
            xyzpack = np.array(list(gen), dtype=np.float32)
            if xyzpack.shape[1] != 4:
                raise ValueError(
                    "PointCloud2 does not have points with color data.")
            xyzrgb = np.empty((xyzpack.shape[0], 6))
            xyzrgb[:, :3] = xyzpack[:, :3]
            for i, x in enumerate(xyzpack):
                rgb = x[3]
                # cast float32 to int so that bitwise operations are possible
                s = struct.pack('>f', rgb)
                i = struct.unpack('>l', s)[0]
                # you can get back the float value by the inverse operations
                pack = ctypes.c_uint32(i).value
                r = (pack & 0x00FF0000) >> 16
                g = (pack & 0x0000FF00) >> 8
                b = (pack & 0x000000FF)
                # r,g,b values in the 0-255 range
                xyzrgb[i, 3:] = (r, g, b)
            return xyzrgb
        else:
            return np.array(list(gen), dtype=np.float32)[:, :3]

    def detect_empty(self, img):
        results = self.model(img)
        for box, conf in zip(results[0].obb, results[0].obb.conf):
            class_id, confidence = int(box.cls[0].item()), float(conf.item())
            if class_id == 0 and confidence >= conf_val:
                x, y, w, h, r = box.xywhr[0].tolist()
                return (x, y, w, h, r)
        return None

    def apply_detections(self, canvas, bbox_info):
        x, y, w, h, r = bbox_info
        points = self.get_rotated_box_points(x, y, w, h, -r)
        self.highlight_region(canvas, points)
        self.draw_lines_and_calculate_angles(canvas, points)

    def get_rotated_box_points(self, x, y, width, height, angle):
        rectangle = np.array([[-width / 2, -height / 2], [width / 2, -height / 2],
                              [width / 2, height / 2], [-width / 2, height / 2]])
        rotation_matrix = np.array([[np.cos(angle), -np.sin(angle)],
                                    [np.sin(angle), np.cos(angle)]])
        rotated_rectangle = np.dot(rectangle, rotation_matrix) + np.array([x, y])
        return np.int0(rotated_rectangle)

    def highlight_region(self, canvas, points):
        buffer = 20
        x_min, y_min = np.min(points, axis=0)
        x_max, y_max = np.max(points, axis=0)
        x_min = max(x_min - buffer, 0)
        y_min = max(y_min - buffer, 0)
        x_max = min(x_max + buffer, canvas.shape[1])
        y_max = min(y_max + buffer, canvas.shape[0])
        cropped_image = canvas[y_min:y_max, x_min:x_max]
        gray = cv2.cvtColor(cropped_image, cv2.COLOR_BGR2GRAY)
        sobelx = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=sobel_kernel_size)
        sobely = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=sobel_kernel_size)
        sobel = cv2.magnitude(sobelx, sobely)
        _, sobel_thresholded = cv2.threshold(sobel, sobel_min_threshold, 255, cv2.THRESH_BINARY)
        mask = (sobel_thresholded > 0)
        canvas[y_min:y_max, x_min:x_max][mask] = [0, 255, 0]  # Green mask on detected lines

    def draw_lines_and_calculate_angles(self, canvas, points):
        # Sort points to determine top and bottom midpoints
        points = sorted(points, key=lambda x: x[1])
        p1, p2 = points[0], points[2]  # Top points of the box
        p3, p4 = points[1], points[3]  # Bottom points of the box
        
        # Calculate midpoints of the top and bottom line segments
        top_mid = ((p1[0] + p2[0]) // 2, (p1[1] + p2[1]) // 2)
        bottom_mid = ((p3[0] + p4[0]) // 2, (p3[1] + p4[1]) // 2)
        
        # Draw the red line between midpoints
        cv2.line(canvas, top_mid, bottom_mid, (0, 0, 255), 2)
        
        # Calculate the midpoint of the red line
        red_line_mid = ((top_mid[0] + bottom_mid[0]) // 2, (top_mid[1] + bottom_mid[1]) // 2)
        print(f"Midpoint of the red line: {red_line_mid}")
        
        # Draw the center blue line
        mid_x = canvas.shape[1] // 2
        cv2.line(canvas, (mid_x, 0), (mid_x, canvas.shape[0]), (255, 0, 0), 2)
        
        # Calculate angle of the red line relative to the vertical center line
        dx = top_mid[0] - bottom_mid[0]
        dy = top_mid[1] - bottom_mid[1]
        angle_radians = np.arctan2(dy, dx)
        angle_from_vertical = abs(np.pi / 2 - angle_radians)
        
        # Use handler to get 3D coordinates of the red line midpoint
        if self.point_cloud:
            numpy_point_cloud = self.ros_PointCloud2_to_numpy(self.point_cloud)
            coord_3d_map = self.handler.get3DCoord(canvas, numpy_point_cloud)
            if 0 <= red_line_mid[0] < coord_3d_map.shape[1] and 0 <= red_line_mid[1] < coord_3d_map.shape[0]:
                red_line_mid_3d = coord_3d_map[red_line_mid[1], red_line_mid[0]]
                print(f"3D coordinates of the red line's midpoint: [{red_line_mid_3d[0]}, {red_line_mid_3d[1]}, {angle_from_vertical}]")

        return canvas

if __name__ == '__main__':
    rospy.init_node('image_processor', anonymous=True)
    processor_node = ImageProcessorNode()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")