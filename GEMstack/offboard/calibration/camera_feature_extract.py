import cv2
import numpy as np
import pandas as pd
import pathlib
import os
import argparse
parser = argparse.ArgumentParser()

parser.add_argument('--output_dir', type=str, default='save')
parser.add_argument('--output_fn', '-o', type=str, default='image.csv')
parser.add_argument('--src_fn', '-s', type=str, required=True)
args = parser.parse_args()

assert args.output_fn.endswith('csv')

# load image file
image = cv2.imread(args.src_fn)
points = [] # store pixel cords

def click_event(event, x, y, flags, param):
    global image
    if event == cv2.EVENT_LBUTTONDOWN:
        # image[y, x] = [0, 0, 255] 
        cv2.circle(image, center=(int(x), int(y)), radius=2, color=(0, 0, 255), thickness=cv2.FILLED)
        cv2.imshow('image', image)
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

pathlib.Path(args.output_dir).mkdir(parents=True, exist_ok=True)
csv_file_path = os.path.join(args.output_dir, args.output_fn)
df_points.to_csv(csv_file_path, index=False)
print(f"All recorded points have been saved to {csv_file_path}.")