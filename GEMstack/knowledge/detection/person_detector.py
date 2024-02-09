from ultralytics import YOLO
import cv2
import sys
import torch

PERSON_ID = 0
BOX_COLOR = (0, 255, 0)
THICKNESS = 2 # measured in number of pixels

def person_detector(img : cv2.Mat):
    #TODO: implement me to produce a list of (x,y,w,h) bounding boxes of people in the image

    # Load a pretrained YOLOv8n model
    model = YOLO('yolov8n.pt')

    # Perform object detection on an image using the model
    print("Performing object detection on specified image")
    results = model(img)

    # xywh tensor of the objects
    people = []

    for result in results:
        boxes = result.boxes  # Boxes object for bbox outputs

        for box in boxes:
            # box = box.cpu()
            classID = int(box.cls[0])
            # print(box)
            if classID == PERSON_ID:
                xywhBox = box.xywh[0]
                people.append(xywhBox.numpy().astype(float))

    # Export the model to ONNX format
    success = model.export(format='onnx')
    return people

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
