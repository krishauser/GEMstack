#!/usr/bin/env python3

import cv2
import sys
from ultralytics import YOLO
from PIL import Image

def person_detector(img : cv2.Mat):
    #TODO: implement me to produce a list of (x,y,w,h) bounding boxes of people in the image
    model = YOLO('yolov8n.pt')
    results = model(img)  # results list
    # Show the results
    ans = []
    for result in results:
        for b in result:
            if b.boxes.cls.item()<0.1:
                print(b.boxes.xywh.tolist()[0])
                ans.append(b.boxes.xywh.tolist()[0])    
    return ans

def main(fn):
    image = cv2.imread(fn)
    bboxes = person_detector(image)
    print("Detected",len(bboxes),"people")
    for bb in bboxes:
        x,y,w,h = bb
        if not isinstance(x,(int,float)) or not isinstance(y,(int,float)) or not isinstance(w,(int,float)) or not isinstance(h,(int,float)):
            print("WARNING: make sure to return Python numbers rather than PyTorch Tensors")
        print("Corner",(x,y),"size",(w,h))
        cv2.rectangle(image, (int(x-w/2), int(y-h/2)), (int(x+w/2), int(y+h/2)), (255, 0, 255), 3)
    cv2.imshow('Results', image)
    cv2.waitKey(0)

def main_webcam(): 
    cap = cv2.VideoCapture(0)
    # cap.set(3, 640)
    # cap.set(4, 480)

    print("Press space to exit")
    while True:
        _, image = cap.read()
        cv2.imshow('weeeee', image) 
        bboxes = person_detector(image)
        for bb in bboxes:
            x,y,w,h = bb
            cv2.rectangle(image, (int(x-w/2), int(y-h/2)), (int(x+w/2), int(y+h/2)), (255, 0, 255), 3)
              
        cv2.imshow('Person detection', image)     
        if cv2.waitKey(1) & 0xFF == ord(' '):
            break

    cap.release()


if __name__ == '__main__':
    fn = 'webcam'
    if fn != 'webcam':
        main(fn)
    else:
        main_webcam()