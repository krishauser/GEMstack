# Autonomous Vehicles Final Project: Perception Models

Link to Google Drive where all models are stored:  
https://drive.google.com/drive/folders/11oXlVTWNcYcLQbluM8bh6MY80lCBoJ0d?usp=drive_link

## Contributed Models

### Cone Detection – Bounding Box Models

1. **Cone_Detector_30m_Under_YOLOv8.pt**  
   - Optimized for detecting cones within 30 meters  
   - Achieves 100% recall and precision with tightly fitting bounding boxes  
   - Limitation: fails to consistently detect cones beyond 30 meters  

2. **Cone_Detector_30m_Plus_YOLOv8.pt**  
   - Designed for cone detection beyond 30 meters, with high precision, recall, and mAP  
   - Overfitting observed due to training data redundancy  
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
| Base model        | N/A    | N/A       | Placeholder for comparison; cannot evaluated         |
| YOLOv8n-seg       | 0.886  | 0.634     | Fast, modern YOLOv8 variant; efficient and compact |
| YOLO11n-seg       | 0.905  | 0.703     | Very fast and lightweight; great for real-time use |
| YOLO11s-seg       | 0.899  | 0.685     | Balanced option with good accuracy and speed      |

---

## Parking Segmentation Analytics 

### 1. Load Model and Run Prediction

```python

from ultralytics import YOLO

# Load model
model = YOLO('')  # Path to parking segmentation weights 

# Run prediction
inference_results = model.predict(source='') #Insert Path to testing image
r = inference_results[0]
```

---

### 2. Mask-to-Centroid + Polygon Extraction Logic

Converts each YOLO mask into an outlined parking space and pinpoints its exact center (red X).

```python
import cv2
import matplotlib.pyplot as plt

# assume you already have:
# results = model.predict(...)
# r = results[0]
h_orig, w_orig = r.orig_shape  # e.g. (1200, 1920)

centers = []
for m in r.masks.data:
    # 1) pull mask off GPU
    mask = m.detach().cpu().numpy().astype('uint8')  # maybe 640×640

    # 2) if it’s not the same shape as your image, scale it
    if mask.shape != (h_orig, w_orig):
        mask = cv2.resize(mask, (w_orig, h_orig), interpolation=cv2.INTER_NEAREST)

    # 3) compute area centroid via moments
    M = cv2.moments(mask)
    cx = M['m10']/(M['m00']+1e-6)
    cy = M['m01']/(M['m00']+1e-6)
    centers.append((cx, cy))

# 4) plot
plt.figure(figsize=(10,6))
plt.imshow(r.orig_img)
for poly in r.masks.xy:
    plt.plot(poly[:,0], poly[:,1], linewidth=2)
xs, ys = zip(*centers)
plt.scatter(xs, ys, marker='x', s=80, c='red')
plt.axis('off')
plt.show()

```
![Example Output](/download-8.png)
---

### 3. Axis-Aligned Bounding Box Overlay

Traced each segmented parking space and overlaid a green dashed axis-aligned bounding box for quick, easy indexing

```python
plt.imshow(r.orig_img)
for poly in r.masks.xy:
    plt.plot(poly[:,0], poly[:,1], 'b-', lw=2)   # outline
    xmin, ymin = poly.min(0); xmax, ymax = poly.max(0)
    plt.plot([xmin,xmax,xmax,xmin,xmin],
             [ymin,ymin,ymax,ymax,ymin], 'g--')  # green AABB
plt.axis('off'); plt.show()

```
![Example Output](/download-9.png)
---

### 4. Min-Area Rotated Bounding Boxes

Fits each parking‑space mask with a red dashed rectangle that rotates to match its true orientation.

```python
import cv2
import numpy as np
import matplotlib.pyplot as plt

# Assume 'r' is the Results object from YOLOv8 segmentation
# and r.masks.xy is a list of N masks, each as an (M, 2) array of polygon vertices.

# Create a plot of the original image
plt.figure(figsize=(10, 6))
plt.imshow(r.orig_img)

# Loop through each polygon to compute and plot the minimum-area rotated rectangle
for poly in r.masks.xy:
    # Convert polygon to float32 array for OpenCV
    pts = poly.astype(np.float32)

    # Compute the minimum area rectangle
    rect = cv2.minAreaRect(pts)  # rect = ((cx, cy), (width, height), angle)

    # Get the 4 corner points of the rectangle
    box = cv2.boxPoints(rect)    # box is a (4, 2) array of corner points

    # Close the loop by re-adding the first point at the end
    box = np.vstack([box, box[0]])

    # Plot the rotated rectangle
    plt.plot(box[:, 0], box[:, 1], 'r--', linewidth=2)

# Remove axes and display
plt.axis('off')
plt.show()
```

---
![Example Output](GEMstack/knowledge/perception/download-10.png)
