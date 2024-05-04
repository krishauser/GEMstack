#needed to import GEMstack from top level directory
import sys
import os
sys.path.append(os.getcwd())

import cv2
import numpy as np

from GEMstack.onboard.perception.lane_detection import LaneDetector

def plot(image, lane_lines):
    if lane_lines is None:
        return

    lane_image = np.copy(image)

    # add left and right lines
    for line in lane_lines[:-1]:
        cv2.line(lane_image, line[0], line[1], color=[255,0,0], thickness=2)
    
    # add centerline
    cv2.line(lane_image, lane_lines[-1][0], lane_lines[-1][1], color=[0,0,255], thickness=2)

    cv2.imshow('Result', lane_image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == '__main__':
    # args: path to image
    image = cv2.imread(sys.argv[1])

    detector = LaneDetector(image)
    
    lane_lines = detector.detect_lanes()
    plot(image, lane_lines)
