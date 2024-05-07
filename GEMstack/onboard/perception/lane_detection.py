from ...state import RoadgraphLane,RoadgraphLaneEnum,RoadgraphSurfaceEnum,RoadgraphCurve,RoadgraphCurveEnum
from ...utils import settings
from ..interface.gem import GEMInterface
from ..component import Component
from .pixelwise_3D_lidar_coord_handler import PixelWise3DLidarCoordHandler

import warnings
import numpy as np
import cv2
import math
import timeit

horizon = settings.get('perception.lane_detection.horizon')
min_angle = settings.get('perception.lane_detection.min_angle')

class LaneDetector(Component):
    """Class to detect lane markings."""

    def __init__(self, vehicle_interface : GEMInterface):
        self.vehicle_interface = vehicle_interface

        self.camera_image = None
        self.height, self.width = None, None
        
        self.lidar_point_cloud = None

    def rate(self):
        return settings.get('perception.lane_detection.rate')
    
    def state_inputs(self):
        return []
    
    def state_outputs(self):
        return ['detected_lane']

    def initialize(self):
        # use image_callback whenever 'front_camera' gets a reading, and it expects images of type cv2.Mat
        self.vehicle_interface.subscribe_sensor('front_camera', self.image_callback, cv2.Mat)
    
        # use lidar_callback whenever 'top_lidar' gets a reading, and it expects numpy arrays
        self.vehicle_interface.subscribe_sensor('top_lidar', self.lidar_callback, np.ndarray)

    def image_callback(self, image : cv2.Mat):
        self.camera_image = image
        self.height, self.width, _ = image.shape

    def lidar_callback(self, point_cloud: np.ndarray):
        self.lidar_point_cloud = point_cloud

    def update(self) -> RoadgraphLane:
        if self.camera_image is None:
            return None   # no image data yet
        
        # debugging
        # self.save_data()
        
        t1 = timeit.default_timer()

        left_poly, right_poly = self.detect_lane()

        t2 = timeit.default_timer()
        print('Detection time: {:.6f} s'.format(t2 - t1))

        return self.polys_to_roadgraph_lane(left_poly, right_poly) 

    def region_of_interest(self, image):
        vertices = [
            (0, self.height),
            (int(self.width * 0.4), int(self.height * (1 - horizon))),
            (int(self.width * 0.6), int(self.height * (1 - horizon))),
            (self.width, self.height)
        ] # trapezium at the bottom of the image

        mask = np.zeros_like(image)
        cv2.fillPoly(mask, np.int32([vertices]), 255)
        
        return cv2.bitwise_and(image, mask)

    def separate_lines(self, detected_lines): # separate lines into 2 categories based on slope
        if detected_lines is None:
            return [], []

        left_pts = []
        right_pts = []

        for line in detected_lines:
            x1, y1, x2, y2 = line[0]
            m = (y2 - y1) / (x2 - x1) # slope
            
            if m < math.tan(-min_angle): # left
                left_pts.extend([(x1, y1), (x2, y2)])
            elif m > math.tan(min_angle): # right
                right_pts.extend([(x1, y1), (x2, y2)])
            else: # not part of the lane markings
                pass

        return left_pts, right_pts

    def compute_polys(self, left_pts, right_pts):
        if len(left_pts) == 0 or len(right_pts) == 0:
            return None, None

        left_coeffs = np.polyfit([p[1] for p in left_pts], [p[0] for p in left_pts], deg=1)
        right_coeffs = np.polyfit([p[1] for p in right_pts], [p[0] for p in right_pts], deg=1)

        return np.poly1d(left_coeffs), np.poly1d(right_coeffs)

    def detect_lane(self):
        # convert color space
        lab_image = cv2.cvtColor(self.camera_image, cv2.COLOR_RGB2LAB)

        # canny edge detection
        edge_image = cv2.Canny(lab_image, 255/3, 255)

        # specify a region of interest
        cropped_image = self.region_of_interest(edge_image)

        # detect straight lines
        detected_lines = cv2.HoughLinesP(cropped_image, rho=20, theta=np.pi/60, threshold=300, 
                                         lines=np.array([]), minLineLength=20, maxLineGap=100)

        # compute equations for the left and right lane lines
        left_pts, right_pts = self.separate_lines(detected_lines)
        left_poly, right_poly = self.compute_polys(left_pts, right_pts) 

        return left_poly, right_poly

    def polys_to_roadgraph_lane(self, left_poly, right_poly):
        if not left_poly or not right_poly:
            return None

        with warnings.catch_warnings():
            warnings.filterwarnings('ignore', category=UserWarning)
            
            # obtain 3d world coordinates for all pixels in the image
            handler = PixelWise3DLidarCoordHandler()
            all_points = handler.get3DCoord(self.camera_image, self.lidar_point_cloud) # img ht x img width x 3

        y_min = int(self.height * (1 - horizon))
        y_max = self.height
        
        # update y_max if the lane lines extend beyond the image
        if left_poly(y_max) < 0 or right_poly(y_max) > self.width:
            y_max = min(left_poly.roots, (right_poly - self.width).roots)

        centerline = []
        for j in range(y_max - 1, y_min - 1, -int((y_max - y_min) / 10)):
            i = int((left_poly(j) + right_poly(j)) / 2)
            if any(all_points[j][i]):
                centerline.append(tuple(all_points[j][i]))

        roadgraph_curve = RoadgraphCurve(type=RoadgraphCurveEnum.LANE_BOUNDARY,
                                         segments=[centerline],
                                         crossable=True)
        
        return RoadgraphLane(type=RoadgraphLaneEnum.LANE,
                             surface=RoadgraphSurfaceEnum.PAVEMENT,
                             route_name='',
                             center=roadgraph_curve)
