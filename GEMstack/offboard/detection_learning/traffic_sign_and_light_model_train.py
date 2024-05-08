from ultralytics import YOLO

model = YOLO("yolov8m.pt")
model.train(data="data.yaml", epochs=50)
metrics = model.val()

print("Overall mAP:", metrics.box.map)
print("mAP at IoU threshold 0.50:", metrics.box.map50)
print("mAP at IoU threshold 0.75:", metrics.box.map75)
print("mAP values at different IoU thresholds for each category:", metrics.box.maps)


