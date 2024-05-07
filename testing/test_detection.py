"""
Program for testing the detection components.

Usage: For a brief description of the arguments needed, run
            python3 testing/test_detection.py -h
       from the main GEMstack folder
"""

#needed to import GEMstack from top level directory
import sys
import os
sys.path.append(os.getcwd())

import argparse
from ultralytics import YOLO
import numpy as np
import cv2
import timeit

from GEMstack.onboard.interface.gem import GEMInterface
from GEMstack.state import RoadgraphLaneEnum,RoadgraphSurfaceEnum,RoadgraphCurveEnum
from GEMstack.onboard.perception.road_agent_detection import AgentDetector
from GEMstack.onboard.perception.sign_detection import SignDetector
from GEMstack.onboard.perception.lane_detection import LaneDetector

agent_dict = dict([
    (0, 'PEDESTRIAN'),
    (1, 'BICYCLIST'),
    (2, 'CAR'),
    (7, 'TRUCK')
])

sign_signal_dict = dict([
    (3, 'NO_LEFT_TURN'),
    (4, 'NO_RIGHT_TURN'),
    (5, 'NO_U_TURN'),
    (7, 'GREEN'),
    (9, 'NO_PARKING'),
    (11, 'PEDESTRIAN_CROSSING'),
    (13, 'RAILROAD_CROSSING'),
    (14, 'RED'),
    (15, 'STOP_SIGN'),
    (20, 'YELLOW')
])


def detect(img, rel_path, class_ids):
    model = YOLO(os.path.join(os.path.dirname(os.path.realpath(__file__)), rel_path))
    return model(img, classes=class_ids, verbose=False)


def plot_detection_bboxes(image, bboxes, obj_types):
    for i in range(len(bboxes)):
        x,y,w,h = bboxes[i]

        check = lambda val : not isinstance(val, (int, float))
        if check(x) or check(y) or check(w) or check(h):
            print('WARNING: make sure to return Python numbers rather than PyTorch Tensors')
        
        print('Corner: ({:.3f},{:.3f}), Size: ({:.3f},{:.3f})'.format(x, y, w, h))

        top_left = (int(x-w/2), int(y-h/2))
        bottom_right = (int(x+w/2), int(y+h/2))
        color = (255, 0, 255)
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.4
        cv2.rectangle(image, top_left, bottom_right, color, thickness=2)
        cv2.putText(image, obj_types[i], tuple(np.subtract(top_left, (0, 10))), font, font_scale, color, thickness=1)
    
    cv2.imshow('Results', image)
    cv2.waitKey(0)


def run_agent_detection_image_only(rgb_path):
    print('\nRunning agent detector on image...')

    image = cv2.imread(rgb_path)

    t1 = timeit.default_timer()
    results = detect(image, '../GEMstack/knowledge/detection/yolov9c.pt', list(agent_dict.keys()))
    t2 = timeit.default_timer()

    bboxes = results[0].boxes.xywh.tolist()
    types = [agent_dict[int(cls)] for cls in results[0].boxes.cls.tolist()]
    
    print('Detected', len(bboxes), 'objects')
    print('Time: {:.9f} seconds'.format(t2-t1))
    
    plot_detection_bboxes(image, bboxes, types)


def run_sign_detection_image_only(rgb_path):
    print('\nRunning sign detector on image...')

    image = cv2.imread(rgb_path)

    t1 = timeit.default_timer()
    results = detect(image, '../GEMstack/knowledge/detection/sign_model.pt', list(sign_signal_dict.keys()))
    t2 = timeit.default_timer()

    bboxes = results[0].boxes.xywh.tolist()
    types = [sign_signal_dict[int(cls)] for cls in results[0].boxes.cls.tolist()]
    
    print('Detected', len(bboxes), 'objects')
    print('Time: {:.9f} seconds'.format(t2-t1))
    
    plot_detection_bboxes(image, bboxes, types)


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
    print('\nRunning lane detector on image...')

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


def run_agent_detection_paired_scan(lidar_path, rgb_path):
    print('\nRunning agent detector on paired scan...')

    agent_detector = AgentDetector(None)
    agent_detector.camera_image = cv2.imread(rgb_path)
    agent_detector.lidar_point_cloud = np.load(lidar_path)['arr_0']

    agents = agent_detector.update(None)

    print('Detected {} agents'.format(len(agents)))
    for key, agent in agents.items():
        print(key)
        print('- type:', agent.type)
        print('- position: ({0:.3f}, {1:.3f}, {2:.3f})'.format(agent.pose.x, agent.pose.y, agent.pose.z))
        print('- dimensions: ({0:.3f}, {1:.3f}, {2:.3f})'.format(agent.dimensions[0], agent.dimensions[1], agent.dimensions[2]))


def run_sign_detection_paired_scan(lidar_path, rgb_path):
    print('\nRunning sign detector on paired scan...')

    sign_detector = SignDetector(None)
    sign_detector.camera_image = cv2.imread(rgb_path)
    sign_detector.lidar_point_cloud = np.load(lidar_path)['arr_0']

    signs = sign_detector.update(None, None)

    print('Detected {} signs'.format(len(signs)))
    for key, sign in signs.items():
        print(key)
        print('- type:', sign.type)
        print('- position: ({0:.3f}, {1:.3f}, {2:.3f})'.format(sign.pose.x, sign.pose.y, sign.pose.z))
        print('- dimensions: ({0:.3f}, {1:.3f}, {2:.3f})'.format(sign.dimensions[0], sign.dimensions[1], sign.dimensions[2]))


def run_lane_detection_paired_scan(lidar_path, rgb_path):
    print('\nRunning lane detector on paired scan...')

    image = cv2.imread(rgb_path)

    lane_detector = LaneDetector(None)
    lane_detector.camera_image = image
    lane_detector.height, lane_detector.width, _ = image.shape
    lane_detector.lidar_point_cloud = np.load(lidar_path)['arr_0']

    roadgraph = lane_detector.update(None)

    if roadgraph:
        lane = roadgraph.lanes['current_lane']
        print(RoadgraphLaneEnum(lane.type).name)
        print('- surface:', RoadgraphSurfaceEnum(lane.surface).name)
        print('- center:')
        print('  - type:', RoadgraphCurveEnum(lane.center.type).name)
        print('  - polyline (in CURRENT frame):')
        for i, pt in enumerate(lane.center.segments[0]):
            print('    {0}. ({1:.3f}, {2:.3f}, {3:.3f})'.format(i+1, pt[0], pt[1], pt[2]))


def test_detection_paired_scan(data_folder, idx, detectors):
    lidar_path = os.path.join(data_folder, 'lidar{}.npz'.format(idx))
    rgb_path = os.path.join(data_folder, 'color{}.png'.format(idx)) 
    
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
