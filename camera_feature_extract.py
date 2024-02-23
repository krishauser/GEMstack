import cv2
import numpy as np
import pandas as pd
# load image file
image = cv2.imread('hw3_data/color1.png')

# # color filter
# hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
# lower_red = np.array([160, 80, 80])
# upper_red = np.array([180, 255, 255])

# # create mask
# mask = cv2.inRange(hsv, lower_red, upper_red)
# red_objects = cv2.bitwise_and(image, image, mask=mask)
# # cv2.imshow('Red Objects', red_objects)

# # area filter
# gray = cv2.cvtColor(red_objects, cv2.COLOR_BGR2GRAY)
# _, thresh = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
# contours, _ = cv2.findContours(thresh, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
# filtered_contours = [cnt for cnt in contours if cv2.contourArea(cnt) >= 100]

# # draw contours
# cv2.drawContours(image, filtered_contours, -1, (0, 255, 0), 3)

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
csv_file_path = 'save/image1_stopsign.csv'
df_points.to_csv(csv_file_path, index=False)

print(f"All recorded points have been saved to {csv_file_path}.")