#from ultralytics import YOLO
import cv2
import sys
from ultralytics import YOLO
import numpy as np


def person_detector(img: cv2.Mat) -> list[tuple[float, float, float, float]]:
    """Detect persons in an image and return their bounding boxes in xywh format.

    Args:
        img (cv2.Mat): Input image in OpenCV BGR format

    Returns:
        List of bounding box tuples (x_center, y_center, width, height)
    """
    # Initialize YOLOv11 model
    model = YOLO("yolo11n.pt")

    # Perform inference with confidence threshold
    results = model(img, conf=0.5)

    # Extract detection boxes from results
    boxes = results[0].boxes
    if len(boxes) == 0:
        return []

    # Convert tensor data to CPU (assuming CUDA acceleration)
    cls_ids = boxes.cls.cpu()  # Class IDs tensor
    xywh = boxes.xywh.cpu()  # Box coordinates in xywh format

    # Create boolean mask for person class (ID 0)
    person_mask = (cls_ids == 0)

    # Convert qualified boxes to Python native types
    return [tuple(map(float, box)) for box in xywh[person_mask]]

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
    cap.set(3, 640)
    cap.set(4, 480)

    print("Press space to exit")
    while True:
        _, image = cap.read()
        
        bboxes = person_detector(image)
        for bb in bboxes:
            x,y,w,h = bb
            cv2.rectangle(image, (int(x-w/2), int(y-h/2)), (int(x+w/2), int(y+h/2)), (255, 0, 255), 3)
              
        cv2.imshow('Person detection', image)     
        if cv2.waitKey(1) & 0xFF == ord(' '):
            break

    cap.release()


if __name__ == '__main__':
    fn = sys.argv[1]
    if fn != 'webcam':
        main(fn)
    else:
        main_webcam()