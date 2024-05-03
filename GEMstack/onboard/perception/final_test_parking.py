import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge
from ultralytics import YOLO
from pixelwise_3D_lidar_coord_handler import PixelWise3DLidarCoordHandler

conf_val = 0.835
MODEL_WEIGHT_PATH = '../../knowledge/detection/parking_spot_detection.pt'

class ImageProcessorNode:
    def __init__(self):
        self.bridge = CvBridge()
        self.model = YOLO(MODEL_WEIGHT_PATH)
        self.image_sub = rospy.Subscriber("/oak/rgb/image_raw", Image, self.image_callback)
        self.lidar_sub = rospy.Subscriber("/ouster/points", PointCloud2, self.lidar_callback)
        self.point_cloud = None
        self.handler = PixelWise3DLidarCoordHandler()

    def lidar_callback(self, point_cloud):
        self.point_cloud = point_cloud

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            return

        if self.point_cloud:
            self.process_image(cv_image)

    def process_image(self, cv_image):
        bbox_info = self.detect_empty(cv_image)
        if bbox_info:
            return self.calculate_goal_pose(cv_image, bbox_info)

    def detect_empty(self, img):
        results = self.model(img)
        for box, conf in zip(results[0].obb, results[0].obb.conf):
            if int(box.cls[0].item()) == 0 and float(conf.item()) >= conf_val:
                x, y, w, h, r = box.xywhr[0].tolist()
                return (x, y, w, h, r)
        return None

    def calculate_goal_pose(self, cv_image, bbox_info):
        x, y, w, h, r = bbox_info
        line_mid = (int((x + w / 2)), int((y + h / 2)))
        angle = np.pi/2 + r

        if self.point_cloud:
            numpy_point_cloud = self.handler.ros_PointCloud2_to_numpy(self.point_cloud)
            coord_3d_map = self.handler.get3DCoord(cv_image, numpy_point_cloud)
            if 0 <= line_mid[0] < coord_3d_map.shape[1] and 0 <= line_mid[1] < coord_3d_map.shape[0]:
                red_line_mid_3d = coord_3d_map[line_mid[1], line_mid[0]]
                return [red_line_mid_3d[0], red_line_mid_3d[1], angle]
        return None

if __name__ == '__main__':
    rospy.init_node('image_processor', anonymous=True)
    processor_node = ImageProcessorNode()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
