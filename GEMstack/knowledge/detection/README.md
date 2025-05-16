# Autonomous Vehicles Final Project: Perception Models

Link to Google Drive where all models are stored:  
https://drive.google.com/drive/folders/11oXlVTWNcYcLQbluM8bh6MY80lCBoJ0d?usp=drive_link

## Contributed Models

### Cone Detection – Bounding Box Models

1. **Cone_Detector_30m_Under_YOLOv8.pt**  
   - Same as 'cone.pt' in the folder. Our default model. Good for most test scenarios. 
   - Optimized for detecting cones within 30 meters. This range is similiar to lidar's effective range for cone detection.  
   - Achieves 100% recall and precision with tightly fitting bounding boxes when cone <25m
   - Limitation: fails to consistently detect cones beyond 30 meters  

2. **Cone_Detector_30m_Plus_YOLOv8.pt**  
   - Designed for cone detection beyond 30 meters, with high precision, recall, and mAP  
   - Overfitting observed due to training data redundancy, less reliable for closer range
   - Serves as a strong baseline for long-range cone detection tasks  

## Parking Space Detection – Lane-Based Segmentation Models

Models:  
1. **parking_space_YOLO11n-seg.pt**  
2. **parking_space_YOLO11s-seg.pt**  
3. **parking_space_yolov8n-seg.pt**  

These models perform well on the original parking lot layout, accurately segmenting lanes and identifying available spaces. However, they do not generalize well to newly modified parking lots with different lane markings. To improve generalization, additional labeled images from the updated environment are required.

> **Note:** While **YOLO11s-seg** currently shows the lowest performance among the three models, its larger architecture offers the most potential for improvement. With more diverse training data and fine-tuning, it could outperform both YOLOv8n-seg and YOLO11n-seg in accuracy and generalization.

| Model             | mAP@50 | mAP@50–95 | Benefits & Tradeoffs                              |
|-------------------|--------|-----------|---------------------------------------------------|
| Base model        | N/A    | N/A       | Placeholder for comparison; not evaluated         |
| YOLOv8n-seg       | 0.886  | 0.634     | Fast, modern YOLOv8 variant; efficient and compact |
| YOLO11n-seg       | 0.905  | 0.703     | Very fast and lightweight; great for real-time use |
| YOLO11s-seg       | 0.899  | 0.685     | Balanced option with good accuracy and speed      |
