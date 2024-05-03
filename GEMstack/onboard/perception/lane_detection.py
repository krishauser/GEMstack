from ...state import VehicleState,Roadgraph,RoadgraphLane,RoadgraphLaneEnum,RoadgraphSurfaceEnum,RoadgraphCurve,RoadgraphCurveEnum
from ...utils import settings
from ..interface.gem import GEMInterface
from ..component import Component
from .pixelwise_3D_lidar_coord_handler import PixelWise3DLidarCoordHandler

import numpy as np
import cv2
import math
import timeit
import copy

horizon = 0.4 # fraction of the image considered (= height of trapezium)
min_angle = np.pi / 6 # min angle subtended by a line in order to be considered as part of a lane marking

class LaneDetector(Component):
    """Class to detect lane markings."""

    def __init__(self, vehicle_interface : GEMInterface):
        self.vehicle_interface = vehicle_interface

        self.camera_image = None
        self.height, self.width = None, None
        self.lidar_point_cloud = None

        self.left_pts = []
        self.right_pts = []

        self.left_poly = None # polynomial
        self.right_poly = None

    def rate(self):
        return settings.get('perception.lane_detection.rate')
    
    def state_inputs(self):
        return ['vehicle', 'roadgraph']
    
    def state_outputs(self):
        return ['roadgraph']

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

    def update(self, roadgraph : Roadgraph) -> Roadgraph:
        if self.camera_image is None:
            return {}   # no image data yet
        
        # debugging
        # self.save_data()
        
        t1 = timeit.default_timer()

        detected_lane = self.detect_lanes()

        t2 = timeit.default_timer()
        print('Detection time: {:.6f} s'.format(t2 - t1))

        r = copy.deepcopy(roadgraph)

        if detected_lane:
            r.lanes = {'lane0' : detected_lane}
        
        return r # updated roadgraph

    def region_of_interest(self, image):
        vertices = [
            (0, self.height),
            (int(self.width * 0.4), int(self.height * (1 - horizon))),
            (int(self.width * 0.6), int(self.height * (1 - horizon))),
            (self.width, self.height)
        ] # trapezium along the lower portion of the image

        mask = np.zeros_like(image)
        cv2.fillPoly(mask, np.int32([vertices]), 255)
        
        return cv2.bitwise_and(image, mask)

    def separate_lines(self, detected_lines): # separate lines into 2 categories based on slope
        if detected_lines is None:
            return

        for line in detected_lines:
            x1, y1, x2, y2 = line[0]
            m = (y2 - y1) / (x2 - x1) # slope
            
            if m < math.tan(-min_angle): # left
                self.left_pts.extend([(x1, y1), (x2, y2)])
            elif m > math.tan(min_angle): # right
                self.right_pts.extend([(x1, y1), (x2, y2)])
            else: # not part of the lane markings
                pass

    def compute_polys(self):
        if len(self.left_pts) == 0 or len(self.right_pts) == 0:
            return

        left_coeffs = np.polyfit([p[1] for p in self.left_pts], [p[0] for p in self.left_pts], deg=1)
        self.left_poly = np.poly1d(left_coeffs)

        right_coeffs = np.polyfit([p[1] for p in self.right_pts], [p[0] for p in self.right_pts], deg=1)
        self.right_poly = np.poly1d(right_coeffs)

    def polys_to_roadgraph_lane(self):
        if not self.left_poly or not self.right_poly:
            return

        # Obtain 3d world coordinates for all pixels in the image
        handler = PixelWise3DLidarCoordHandler()
        all_points = handler.get3DCoord(self.camera_image, self.lidar_point_cloud) # img ht x img width x 3

        y_min = int(self.height * (1 - horizon))
        y_max = self.height
        
        # update y_max if the lane lines extend beyond the image
        if self.left_poly(y_max) < 0 or self.right_poly(y_max) > self.width:
            y_max = min(self.left_poly.roots, (self.right_poly - self.width).roots)

        centerline = []
        for j in range(y_max - 1, y_min - 1, -int((y_max - y_min) / 10)):
            i = int((self.left_poly(j) + self.right_poly(j)) / 2)
            if any(all_points[j][i]):
                centerline.append(all_points[j][i])

        centerline_segments = []
        for i, j in zip(centerline[:-1], centerline[1:]):
            centerline_segments.append([tuple(i), tuple(j)])

        roadgraph_curve = RoadgraphCurve(type=RoadgraphCurveEnum.LANE_BOUNDARY,
                                         segments=centerline_segments,
                                         crossable=True)
        
        return RoadgraphLane(type=RoadgraphLaneEnum.LANE,
                             surface=RoadgraphSurfaceEnum.PAVEMENT,
                             route_name='',
                             center=roadgraph_curve)

    def detect_lanes(self):
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
        self.separate_lines(detected_lines)
        self.compute_polys() 

        lane = self.polys_to_roadgraph_lane()
        return lane
