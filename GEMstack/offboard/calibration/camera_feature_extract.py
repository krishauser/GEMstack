import cv2
import numpy as np
import pandas as pd
import pathlib
import os

# load image file

OUTPUT_DIR = 'save'
data_idx = 6
image_fn = f'data/step1/color{data_idx}.png'

image = cv2.imread(image_fn)
points = [] # store pixel cords

def click_event(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        points.append((x, y))
        print(f"Recorded point: ({x}, {y})")

cv2.namedWindow('image')
cv2.setMouseCallback('image', click_event)

cv2.imshow('image', image)
cv2.waitKey(0)
cv2.destroyAllWindows()

print("All recorded points:")
for point in points:
    print(point)

# Save points as csv
df_points = pd.DataFrame(points, columns=['X', 'Y'])

pathlib.Path(OUTPUT_DIR).mkdir(parents=True, exist_ok=True)
csv_file_path = os.path.join(OUTPUT_DIR, f'image{data_idx}.csv')
df_points.to_csv(csv_file_path, index=False)
print(f"All recorded points have been saved to {csv_file_path}.")