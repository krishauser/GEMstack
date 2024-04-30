"""
Python program for object detection (pedestrians, vehicles, road signs, traffic signals).

Usage: For a brief description of the arguments, run
            python3 testing/test_detection_image_only.py -h
       from the main GEMstack folder
"""

import argparse
import os
from ultralytics import YOLO
import numpy as np
import cv2
import timeit

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

def agent_detector(img : cv2.Mat):
    results = detect(img, '../GEMstack/knowledge/detection/yolov9c.pt', list(agent_dict.keys()))
    types = [agent_dict[int(cls)] for cls in results[0].boxes.cls.tolist()]
    return results[0].boxes.xywh.tolist(), types

def sign_detector(img : cv2.Mat):
    results = detect(img, '../GEMstack/knowledge/detection/sign_model.pt', list(sign_signal_dict.keys()))
    types = [sign_signal_dict[int(cls)] for cls in results[0].boxes.cls.tolist()]
    return results[0].boxes.xywh.tolist(), types

def plot(image, bboxes, obj_types):
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

def main(detector, image_path):
    image = cv2.imread(image_path)

    t1 = timeit.default_timer()
    
    bboxes, obj_types = detector(image)
    
    t2 = timeit.default_timer()

    print('Detected', len(bboxes), 'objects')
    print('Time: {:.9f} seconds'.format(t2-t1))
    
    plot(image, bboxes, obj_types)

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Detect objects of interest in image(s)')
    parser.add_argument('-d', '--detector', required=False, dest='detector', help='Detector to run (1 for pedestrian/ vehicle detection, 2 for road sign/ traffic signal detection). This argument is optional (default: 1)')
    parser.add_argument('-p', '--path', required=True, dest='path', help='Path to the image or folder with images')
    
    args = parser.parse_args()

    if os.path.isfile(args.path):
        paths = [args.path]  
    else:
        exts = ['.jpg', '.jpeg', '.png', '.webp']
        paths = [os.path.join(args.path, f) for f in os.listdir(args.path) if any(ext in f for ext in exts)]

    if args.detector == '2':
        detector = sign_detector
    else:
        detector = agent_detector

    for path in paths:
        main(detector, path)
