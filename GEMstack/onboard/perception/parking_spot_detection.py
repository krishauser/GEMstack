from ...state import AllState, VehicleState, ObjectPose, ObjectFrameEnum, AgentState, AgentEnum, AgentActivityEnum
from sensor_msgs.msg import Image, PointCloud2
from ..interface.gem import GEMInterface
from ..component import Component
from typing import Dict
import threading
import copy
import cv2
import numpy as np
import rospy
from ultralytics import YOLO  # Assuming appropriate relative path or installed package
from GEMstack.onboard.perception.pixelwise_3D_lidar_coord_handler import PixelWise3DLidarCoordHandler

sobel_kernel_size = 3
sobel_min_threshold = 90
conf_thresh = 0.835
MODEL_WEIGHT_PATH = 'GEMstack/knowledge/detection/parking_spot_detection.pt'

# Template for a parking spot detector
class ParkingSpotDetector(Component):
    """Performs Parking Spot Detection"""
    def __init__(self, vehicle_interface : GEMInterface):
        print("initializing")
        self.pub = rospy.Publisher("/annotated_lines", Image, queue_size=10)
        self.vehicle_interface = vehicle_interface
        self.model = YOLO(MODEL_WEIGHT_PATH)
        self.handler = PixelWise3DLidarCoordHandler()
        self.euclidean = None
        self.dist_thresh = 8
        self.front_image = None
        self.point_cloud = None

        # self.x = 14.768
        # self.y = -6.092
        # self.yaw = -1.1
        self.grey_thresh = 180
        # self.parking_spot = ObjectPose(t=0, x=self.x, y=self.y, yaw=self.yaw, frame=ObjectFrameEnum.START)
        self.parking_spot = None

    def rate(self):
        return 0.25

    def state_outputs(self):
        return ['parking_spot']

    def initialize(self):
        # Subscribe to necessary sensors
        self.vehicle_interface.subscribe_sensor('front_camera', self.image_callback, cv2.Mat)
        self.vehicle_interface.subscribe_sensor('top_lidar', self.lidar_callback, PointCloud2)
        print("updating")
        founded = False
        while not founded:
            res = self.parking_spot_detection()  # Attempt to detect and update parking spot
            # x, y = 14.768, -6.092
            # yaw = -1.1
            if res:
                print("Our code")
                print("self.parking_spot: ",self.parking_spot)
                founded = True

            print("Looping......")

    def image_callback(self, image: cv2.Mat):
        self.front_image = image

    def lidar_callback(self, point_cloud):
        self.point_cloud = point_cloud

    def detect_empty(self, img: cv2.Mat, empty_spot=0, conf_threshold=conf_thresh):
        results = self.model(img)
        for box, conf in zip(results[0].obb, results[0].obb.conf):
            class_id = int(box.cls[0].item())
            confidence = float(conf.item())
            if class_id == empty_spot and confidence >= conf_threshold:
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

    def parking_spot_detection(self):
        if self.front_image is None or self.point_cloud is None:
            print("camera or sensors not working")
            return None # Just return without doing anything if data is not ready
        print("GREAT! camera and sensors working")

        if self.euclidean is not None and self.euclidean < self.dist_thresh:
            print(f"euclidean = {self.euclidean} is less than threshold")
            return None # Do nothing if the object is out of range

        bbox_info = self.detect_empty(self.front_image)
        if not bbox_info:
            return None # Do nothing if no empty parking spot is detected

        canvas = self.front_image.copy()
        [x,y,yaw] = self.get_parking_spot(canvas, bbox_info)
        print(f"x = {x}, y = {y}, yaw = {yaw}")

        if x and y and yaw:
            self.euclidean = np.sqrt(x**2 + y**2)
            self.parking_spot = ObjectPose(t=0, x=x, y=y, yaw=yaw, frame=ObjectFrameEnum.START)
            return True

    def get_parking_spot(self, img, bbox):
        [midpoint, angle, img] = self.isolate_and_draw_lines(img, bbox)
        [x,y] = self.get_goal_pose(midpoint)
        if x and y:
            return [x,y,angle]
        return [None,None,angle]

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
        midpoint = None
        angle = None

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

                        angle = np.arctan2(mid_y2 - mid_y1, mid_x2 - mid_x1) - np.pi / 2
                        angle = angle if angle >= 0 else angle + np.pi

                        cv2.line(img, (x + mid_x1, y + mid_y1), (x + mid_x2, y + mid_y2), (0, 0, 255), 4)
                        for x1, y1, x2, y2 in avg_lines:
                            cv2.line(img, (x + x1, y + y1), (x + x2, y + y2), (0, 255, 0), 4)

            return [midpoint, angle, img]
        return [None, None, img]
    
    def get_goal_pose(self, coords_pixel):
        if self.point_cloud and coords_pixel:
            numpy_point_cloud = self.handler.ros_PointCloud2_to_numpy(self.point_cloud)
            coord_3d_map = self.handler.get3DCoord(self.front_image, numpy_point_cloud)
            (x, y) = coords_pixel
            if 0 <= int(x) < coord_3d_map.shape[1] and 0 <= int(y) < coord_3d_map.shape[0]:
                midpoint_3d = coord_3d_map[y, x]
                if midpoint_3d[0] != 0 and midpoint_3d[1] != 0:
                    return [midpoint_3d[0], midpoint_3d[1]]
        return [None,None]

    def update(self):
        # print("updating")
        # founded = False
        # while not founded:
        #     res = self.parking_spot_detection()  # Attempt to detect and update parking spot
        #     # x, y = 14.768, -6.092
        #     # yaw = -1.1
        #     if res:
        #         print("Our code")
        #         print("self.parking_spot: ",self.parking_spot)
        #         founded = True

        #     print("Looping......")

        return self.parking_spot
