from ultralytics import YOLO
from PIL import Image

# Load a pretrained YOLOv8n-cls Classify model
model = YOLO('yolov8n.pt')

# Run inference on an image
results = model('bus.jpg')  # results list

# Show the results
for result in results:
    for b in result:
        if b.boxes.cls.item()<0.9:
            print(b.boxes.xywh.tolist())
