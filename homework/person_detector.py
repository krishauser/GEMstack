import cv2
import sys
from ultralytics import YOLO

def person_detector(img: cv2.Mat):
    # Load the YOLOv8 model (assuming you  ave the model file 'yolov8n.pt' in the same directory)
    model = YOLO('../GEMstack/knowledge/detection/yolo11n.pt')  # You can also use 'yolov8s.pt', 'yolov8m.pt', etc.

    # Perform detection on the image
    results = model(img)

    # Extract bounding boxes for people (class 0 in COCO dataset)
    bboxes = []
    for result in results:
        for box in result.boxes:
            if box.cls == 0:  # Class 0 corresponds to 'person' in COCO dataset
                x, y, w, h = box.xywh[0].tolist()  # Get the center coordinates, width, and height
                bboxes.append((x, y, w, h))
    return bboxes

def main(fn):
    image = cv2.imread(fn)
    bboxes = person_detector(image)
    print("Detected", len(bboxes), "people")
    for bb in bboxes:
        x, y, w, h = bb
        if not isinstance(x, (int, float)) or not isinstance(y, (int, float)) or not isinstance(w, (int, float)) or not isinstance(h, (int, float)):
            print("WARNING: make sure to return Python numbers rather than PyTorch Tensors")
        print("Corner", (x, y), "size", (w, h))
        cv2.rectangle(image, (int(x - w / 2), int(y - h / 2)), (int(x + w / 2), int(y + h / 2)), (255, 0, 255), 3)
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
            x, y, w, h = bb
            cv2.rectangle(image, (int(x - w / 2), int(y - h / 2)), (int(x + w / 2), int(y + h / 2)), (255, 0, 255), 3)

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