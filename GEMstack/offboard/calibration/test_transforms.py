import numpy as np
import cv2
import matplotlib.pyplot as plt

# Load LiDAR points
lidar_data = np.load("./data/lidar1.npz")  # Update filename if needed
lidar_points = lidar_data['arr_0']  # Ensure the correct key

# Load Camera Image
image = cv2.imread("./data/color1.png")  # Update filename if needed
image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)  # Convert BGR to RGB

# Transformation Matrix
T_lidar_camera = np.array([
    [0.001090, -0.999489, -0.031941,  0.149698],
    [-0.007664,  0.031932, -0.999461, -0.397813],
    [0.999970,  0.001334, -0.007625, -0.691405],
    [0.000000,  0.000000,  0.000000,  1.000000]
])

# Convert LiDAR points to homogeneous coordinates
num_points = lidar_points.shape[0]
lidar_homogeneous = np.hstack((lidar_points, np.ones((num_points, 1))))  # (N, 4)

# Transform LiDAR points to Camera Frame
lidar_points_camera = (T_lidar_camera @ lidar_homogeneous.T).T  # (N, 4)

# intrinsic matrix
K = np.array([
    [684.83331299,   0.        , 573.37109375],
    [  0.        , 684.60968018, 363.70092773],
    [  0.        ,   0.        ,   1.        ]
], dtype=np.float32)

# Extract 3D points in camera frame
X_c = lidar_points_camera[:, 0]
Y_c = lidar_points_camera[:, 1]
Z_c = lidar_points_camera[:, 2]  # Depth

# Avoid division by zero
valid = Z_c > 0
X_c, Y_c, Z_c = X_c[valid], Y_c[valid], Z_c[valid]

# Project points onto image plane
u = (K[0, 0] * X_c / Z_c) + K[0, 2]
v = (K[1, 1] * Y_c / Z_c) + K[1, 2]

# Filter points within image bounds
img_h, img_w, _ = image.shape
valid_pts = (u >= 0) & (u < img_w) & (v >= 0) & (v < img_h)
u, v = u[valid_pts], v[valid_pts]

# Plot projected points on camera image
plt.imshow(image)
plt.scatter(u, v, s=2, c='r')  # Red dots for projected points
plt.title("Projected LiDAR Points on Camera Image")
plt.show()
