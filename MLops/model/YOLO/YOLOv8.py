from ultralytics import YOLO
from ultralytics.utils.plotting import Annotator
import cv2

class YOLOv8():
    def __init__(self):
        self.model = YOLO('yolov8n.pt')

    def get_model(self):
        return self.model

    def get_object_id(self, name):
        object_list = list(self.model.names.values())
        return object_list.index(name) if name in object_list else -1

    def object_detect_all(self, image):
        results = self.model(image,verbose=False)
        for r in results:
            annotator = Annotator(image)
            boxes = r.boxes
            for box in boxes:
                b = box.xyxy[0]
                c = box.cls
                annotator.box_label(b, self.model.names[int(c)])
        return annotator.result()

    def object_detect_partial(self, image, class_id):
        results = self.model(image,verbose=False, classes=class_id)
        for r in results:
            annotator = Annotator(image)
            boxes = r.boxes
            for box in boxes:
                b = box.xyxy[0]
                c = box.cls
                annotator.box_label(b, self.model.names[int(c)])
        return annotator.result()

    def pedestrian_detect(self, image):
        results = self.model(image,verbose=False, classes=0)
        for r in results:
            annotator = Annotator(image)
            boxes = r.boxes
            for box in boxes:
                b = box.xyxy[0]
                annotator.box_label(b, "Pedestrian")
        return annotator.result()

model = YOLOv8()
#image = cv2.imread('test.jpg')
#cv2.imshow('YOLOv8', model.object_detect_partial(image, [2,1]))
#cv2.waitKey(0)