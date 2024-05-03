"""
Program for testing the detection components.

Args:
    [folder]: path to the folder containing the paired scans
    [n]: scan number (eg: color[n].png, lidar[n].npz)

Usage: From the main GEMstack folder, run
            python3 testing/test_detection_paired_scan.py [folder] [n]
"""

#needed to import GEMstack from top level directory
import sys
import os
sys.path.append(os.getcwd())

import argparse
import numpy as np
import cv2

from GEMstack.onboard.interface.gem import GEMInterface
from GEMstack.state import AgentState, Route, SceneState, Path, ObjectFrameEnum, SignEnum, Roadgraph
from GEMstack.onboard.perception.agent_detection_v2 import AgentDetector
from GEMstack.onboard.perception.sign_detection import SignDetector
from GEMstack.onboard.perception.lane_detection import LaneDetector

def run_agent_detection_image_only():
    pass

def run_agent_detection_paired_scan(lidar_path, rgb_path):
    print('\nRunning agent detector...')

    agent_detector = AgentDetector(None)
    agent_detector.camera_image = cv2.imread(rgb_path)
    agent_detector.lidar_point_cloud = np.load(lidar_path)['arr_0']

    agent_states = agent_detector.update(None)

    print('Detected {} agents'.format(len(agent_states)))
    for i in range(len(agent_states)):
        print('Agent {}:'.format(i+1))
        s = agent_states[i]
        print('- type:', s.type)
        print('- position: ({0:.3f}, {1:.3f}, {2:.3f})'.format(s.pose.x, s.pose.y, s.pose.z))
        print('- dimensions: ({0:.3f}, {1:.3f}, {2:.3f})'.format(s.dimensions[0], s.dimensions[1], s.dimensions[2]))

def run_sign_detection_image_only():
    pass

def run_sign_detection_paired_scan(lidar_path, rgb_path):
    print('\nRunning sign detector...')

    sign_detector = SignDetector(None)
    sign_detector.camera_image = cv2.imread(rgb_path)
    sign_detector.lidar_point_cloud = np.load(lidar_path)['arr_0']

    signs = sign_detector.update(None, None)

    print('Detected {} signs'.format(len(signs)))
    for i, type in enumerate(signs):
        print('Sign {}:'.format(i+1))
        print('- type:', type)
        s = signs[type]
        print('- position: ({0:.3f}, {1:.3f}, {2:.3f})'.format(s.pose.x, s.pose.y, s.pose.z))
        print('- dimensions: ({0:.3f}, {1:.3f}, {2:.3f})'.format(s.dimensions[0], s.dimensions[1], s.dimensions[2]))

def plot_lane_detection_result(image, lane_lines):
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

def run_lane_detection_image_only(rgb_path):
    print('\nRunning lane detector...')

    image = cv2.imread(rgb_path)
    height, width, _ = image.shape

    lane_detector = LaneDetector(None)
    lane_detector.camera_image = image
    lane_detector.height, lane_detector.width, _ = image.shape

    left_poly, right_poly = lane_detector.detect_lane()

    if not left_poly or not right_poly:
        return

    y_min = height * 0.6
    y_max = height
    
    # update y_max if the lane lines extend beyond the image
    if left_poly(y_max) < 0 or right_poly(y_max) > width:
        y_max = min(left_poly.roots, (right_poly - width).roots)
    
    get_pt = lambda p, y : (int(p(y)), int(y)) if not isinstance(p, list) else (int((p[0](y) + p[1](y)) / 2), int(y))

    left_near_pt = get_pt(left_poly, y_max)
    left_far_pt = get_pt(left_poly, y_min)

    right_near_pt = get_pt(right_poly, y_max)
    right_far_pt = get_pt(right_poly, y_min)

    center_near_pt = get_pt([left_poly, right_poly], y_max)
    center_far_pt = get_pt([left_poly, right_poly], y_min)

    plot_lane_detection_result(image, [[left_near_pt, left_far_pt], 
                                       [right_near_pt, right_far_pt], 
                                       [center_near_pt, center_far_pt]])

def run_lane_detection_paired_scan(lidar_path, rgb_path):
    print('\nRunning lane detector...')

    image = cv2.imread(rgb_path)

    lane_detector = LaneDetector(None)
    lane_detector.camera_image = image
    lane_detector.height, lane_detector.width, _ = image.shape
    lane_detector.lidar_point_cloud = np.load(lidar_path)['arr_0']

    roadgraph = lane_detector.update(Roadgraph.zero())
    print(roadgraph)

def test_detection_image_only(rgb_path, detectors):
    fns = set()
    for char in list(detectors):
        if char == '1':
            fns.add(run_agent_detection_image_only)
        elif char == '2':
            fns.add(run_sign_detection_image_only)
        elif char == '3':
            fns.add(run_lane_detection_image_only)
        else:
            print('Wrong character {}. Running all detectors.'.format(char))
            fns.update([run_agent_detection_image_only, 
                        run_sign_detection_image_only, 
                        run_lane_detection_image_only])

    for fn in fns:
        fn(rgb_path)

def test_detection_paired_scan(data_folder, idx, detectors):
    lidar_path = os.path.join(data_folder, 'lidar{}.npz'.format(idx))
    rgb_path = os.path.join(data_folder, 'color{}.png'.format(idx)) 
    # depth_path = os.path.join(data_folder, 'depth{}.tif'.format(idx))
    
    fns = set()
    for char in list(detectors):
        if char == '1':
            fns.add(run_agent_detection_paired_scan)
        elif char == '2':
            fns.add(run_sign_detection_paired_scan)
        elif char == '3':
            fns.add(run_lane_detection_paired_scan)
        else:
            print('Wrong character {}. Running all detectors.'.format(char))
            fns.update([run_agent_detection_paired_scan, 
                        run_sign_detection_paired_scan, 
                        run_lane_detection_paired_scan])

    for fn in fns:
        fn(lidar_path, rgb_path)

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Detect objects of interest in a paired scan or image(s)', formatter_class=argparse.RawTextHelpFormatter)
    
    parser.add_argument('-p', '--path', required=True, dest='path', help='Path to an image or folder')
    parser.add_argument('-i', '--index', required=False, dest='idx', help='Index number\nThis argument is optional, needed only for paired scan', default=None)
    parser.add_argument('-d', '--detectors', required=False, dest='detectors', help='Detector(s) to run (1 - agent, 2 - sign, 3 - lane)\nTo run multiple detectors, use 12, 23, 13 or 123\nThis argument is optional (default: 123)', default='123')
    
    args = parser.parse_args()

    if args.idx:
        test_detection_paired_scan(args.path, args.idx, args.detectors)
    else:
        if os.path.isfile(args.path):
            paths = [args.path]  
        else:
            exts = ['.jpg', '.jpeg', '.png', '.webp']
            paths = [os.path.join(args.path, f) for f in os.listdir(args.path) if any(ext in f for ext in exts)]
        
        for path in paths:
            test_detection_image_only(path, args.detectors)
