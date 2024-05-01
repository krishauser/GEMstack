import numpy as np
import cv2
import math

horizon = 0.4 # fraction of the image considered (= height of trapezium)
min_angle = np.pi / 6 # min angle subtended by a line in order to be considered as part of a lane marking

class LaneDetector():
    """Class to detect lane markings."""

    def __init__(self, image):
        self.image = image
        self.height, self.width, _ = self.image.shape

        self.left_pts = []
        self.right_pts = []

        self.left_poly = None # polynomial
        self.right_poly = None

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

    def left_right_poly_to_lane(self):
        if not self.left_poly or not self.right_poly:
            return

        y_min = self.height * (1 - horizon)
        y_max = self.height
        
        # update y_max if the lane lines extend beyond the image
        if self.left_poly(y_max) < 0 or self.right_poly(y_max) > self.width:
            y_max = min(self.left_poly.roots, (self.right_poly - self.width).roots)
        
        get_pt = lambda p, y : (int(p(y)), int(y)) if not isinstance(p, list) else (int((p[0](y) + p[1](y)) / 2), int(y))

        left_near_pt = get_pt(self.left_poly, y_max)
        left_far_pt = get_pt(self.left_poly, y_min)

        right_near_pt = get_pt(self.right_poly, y_max)
        right_far_pt = get_pt(self.right_poly, y_min)

        center_near_pt = get_pt([self.left_poly, self.right_poly], y_max)
        center_far_pt = get_pt([self.left_poly, self.right_poly], y_min)

        # TODO: use pixel to 3d coord handler to obtain the coordinates of these pts w.r.t vehicle

        return [[left_near_pt, left_far_pt], [right_near_pt, right_far_pt], [center_near_pt, center_far_pt]]

    def detect_lanes(self):
        # convert color space
        lab_image = cv2.cvtColor(self.image, cv2.COLOR_RGB2LAB)

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

        lane = self.left_right_poly_to_lane()
        return lane
