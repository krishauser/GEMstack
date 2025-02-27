import numpy as np
import cv2
import matplotlib.pyplot as plt
import argparse

def load_lidar_data(lidar_path):
    lidar_data = np.load(lidar_path)
    return lidar_data['arr_0']  # Ensure the correct key

def load_image(image_path):
    image = cv2.imread(image_path)
    return cv2.cvtColor(image, cv2.COLOR_BGR2RGB) #convert BGR2RGB

def main(lidar_path, image_path):
    # Load Data
    lidar_points = load_lidar_data(lidar_path)
    image = load_image(image_path)
    
    # Transformation Matrix
    T_lidar_camera = np.array([
        [ 2.89748006e-02, -9.99580136e-01,  3.68439439e-05, -3.07300513e-02],
        [-9.49930618e-03, -3.12215512e-04, -9.99954834e-01, -3.86689354e-01],
        [ 9.99534999e-01,  2.89731321e-02, -9.50437214e-03, -6.71425124e-01],
        [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.00000000e+00]
    ])

    # Convert LiDAR points to homogeneous coordinates
    num_points = lidar_points.shape[0]
    lidar_homogeneous = np.hstack((lidar_points, np.ones((num_points, 1))))

    # Transform LiDAR points to Camera Frame
    lidar_points_camera = (T_lidar_camera @ lidar_homogeneous.T).T

    # Intrinsic Camera Matrix
    K = np.array([
        [684.83331299,   0.        , 573.37109375],
        [  0.        , 684.60968018, 363.70092773],
        [  0.        ,   0.        ,   1.        ]
    ], dtype=np.float32)

    # Extract 3D points in camera frame
    X_c, Y_c, Z_c = lidar_points_camera[:, 0], lidar_points_camera[:, 1], lidar_points_camera[:, 2]

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
    plt.scatter(u, v, s=2, c='r')
    plt.title("Projected LiDAR Points on Camera Image")
    plt.show()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Project LiDAR points onto a camera image.")
    parser.add_argument("--lidar", type=str, required=True, help="Path to the LiDAR .npz file")
    parser.add_argument("--image", type=str, required=True, help="Path to the camera image")
    args = parser.parse_args()
    
    main(args.lidar, args.image)

