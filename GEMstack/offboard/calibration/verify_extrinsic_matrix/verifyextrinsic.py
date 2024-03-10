import numpy as np
import cv2

def read_coordinates_from_file(file_path, extrinsic_matrix, intrinsic_matrix):
    coordinates = []
    with open(file_path, 'r') as file:
        for line in file:
            if 'Adding point' in line:
                parts = line.split('(')[1].split(')')[0].split(',')
                x, y, z = float(parts[0]), float(parts[1]), float(parts[2])
                coordinates.append([x, y, z])

    # Transform LiDAR points to camera coordinates
    lidar_points = np.array(coordinates)
    ones_column = np.ones((lidar_points.shape[0], 1))
    lidar_points_homogeneous = np.hstack((lidar_points, ones_column))
    
    # Update the transformation order and matrices
    camera_points_homogeneous = np.dot(intrinsic_matrix, np.dot(extrinsic_matrix, lidar_points_homogeneous.T))

    # Project camera coordinates to image coordinates
    image_points = camera_points_homogeneous[:2, :]/camera_points_homogeneous[2, :]
    print(image_points)
    return image_points.T

# Values as provided in the image
focal_length_x = 527.5779418945312
focal_length_y = 527.5779418945312
principal_point_x = 616.2459716796875
principal_point_y = 359.21554524969

# Creating the camera intrinsic matrix with an added column of zeros
intrinsic_matrix = np.array([
    [focal_length_x, 0, principal_point_x, 0],
    [0, focal_length_y, principal_point_y, 0],
    [0, 0, 1, 0]
])

matrix = [
    [0.36282628, -0.9256864, 0.00213801, -1.41436548],
    [-0.04834961, -0.02051524, -0.10001977, -0.02062586],
    [0.93440012, 0.35124484, -0.05239839, -0.15902421],
    [0, 0, 0, 1]
]
Extrinsic_Matrix = np.array(matrix)

# Replace with your file path
file_path = 'data/verify/points_lidar7.txt'
image_points = read_coordinates_from_file(file_path, Extrinsic_Matrix, intrinsic_matrix)

# Read image from file
image_path = 'data/hw3_data/color7.png'
image = cv2.imread(image_path)

for point in image_points:
    cv2.circle(image, (int(point[0]), int(point[1])), 3, (0, 165, 255), -1)

cv2.imshow('Image', image)
cv2.waitKey(0)
cv2.destroyAllWindows()
