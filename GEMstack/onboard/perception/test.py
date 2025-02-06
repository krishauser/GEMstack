import os

current_dir = os.path.dirname(os.path.abspath(__file__))

print(os.path.abspath(os.path.join(current_dir, '../../knowledge/detection/yolov8n.pt')))

