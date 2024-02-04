""" 
    Usage:
        python person_detector.py ../data/street.jpg
"""
from ultralytics import YOLO
import cv2
import sys

MODEL_PATH = './knowledge/detection/yolov8n.pt'
model = YOLO(MODEL_PATH)  # pretrained YOLOv8n model


def person_detector(img: cv2.Mat, pedestrian_class = 0):
    #TODO: implement me to produce a list of (x,y,w,h) bounding boxes of people in the image
    
    global model
    
    # Run inference on an image
    results = model(img)  # results list
    
    bboxs = []
    for box in results[0].boxes:
        class_id = int(box.cls[0].item())
        if class_id == pedestrian_class:
            bboxs.append(box.xywh[0].tolist())
                    
    return bboxs

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