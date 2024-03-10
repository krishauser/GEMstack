import numpy as np
import cv2
from scipy.optimize import minimize

# Define the path to your text file
file_path = 'data/opencv_pickedpoints.txt'


# Values as provided in the image
focal_length_x = 527.5779418945312
focal_length_y = 527.5779418945312
principal_point_x = 616.2459716796875
principal_point_y = 359.21554524969

# Creating the camera intrinsic matrix
intrinsic_matrix = np.array([
    [focal_length_x, 0, principal_point_x],
    [0, focal_length_y, principal_point_y],
    [0, 0, 1]
])


# Initialize an empty list to store the coordinates
camera_coordinates = []

# Read the file and extract the coordinates
with open(file_path, 'r') as file:
    for line in file:
        if line.startswith('('):
            parts = line.strip('()\n').split(', ')
            x = int(parts[0].split('=')[1])
            y = int(parts[1].split('=')[1])
            camera_coordinates.append((x, y))

# Convert the list of coordinates to a NumPy array
coordinates_array_camera = np.array(camera_coordinates)

file_path1 = 'data/pickedpoint.txt'

# Initialize an empty list to store the coordinates
lidar_coordinates = []

# Read the file and extract the coordinates
with open(file_path1, 'r') as file:
    for line in file:
        print(line)
        if line.strip().startswith('(') and line.endswith(')\n'):
            parts = line.strip('()\n').split(', ')
            x = float(parts[0])
            y = float(parts[1])
            z = float(parts[2])
            print(f"x:{x} y:{y} z:{z}")
            lidar_coordinates.append((x, y, z))

# Convert the list of coordinates to a NumPy array
coordinates_array_lidar = np.array(lidar_coordinates)

# Print the NumPy array
print(coordinates_array_lidar)
print(coordinates_array_camera)
print(len(coordinates_array_lidar))
print(len(coordinates_array_camera))

def pnpmethod(object_points,image_points):
    # Ensure the points are in the correct shape
    object_points = coordinates_array_lidar.astype(np.float32)
    image_points = coordinates_array_camera.astype(np.float32)

    object_points = object_points.reshape(-1, 3)
    image_points = image_points.reshape(-1, 2)
    # Solve for the rotation and translation vectors
    success, rotation_vector, translation_vector = cv2.solvePnP(object_points, image_points, intrinsic_matrix, None)

    # Check if the solution was found
    if success:
        # Convert the rotation vector to a rotation matrix
        rotation_matrix, _ = cv2.Rodrigues(rotation_vector)

        # Combine the rotation matrix and translation vector into an extrinsic matrix
        extrinsic_matrix = np.hstack((rotation_matrix, translation_vector))
                # Print the extrinsic matrix
        print("Extrinsic Matrix:")
        print(extrinsic_matrix)
        return extrinsic_matrix

    else:
        print("PnP solution not found.")





# 定义损失函数
def reprojection_error(params, object_points, image_points, intrinsic_matrix):
    # 将参数分解为旋转向量和平移向量
    rotation_vector = params[:3].reshape((3, 1))
    translation_vector = params[3:].reshape((3, 1))

    # 计算旋转矩阵
    rotation_matrix, _ = cv2.Rodrigues(rotation_vector)

    # 计算重投影点
    projected_points, _ = cv2.projectPoints(object_points, rotation_vector, translation_vector, intrinsic_matrix, None)

    # 计算重投影误差
    error = image_points - projected_points.squeeze()
    return np.sum(error**2)

def getfinal_extrinsic_maxtrix(initial_extrinsic_matrix):
        # 将旋转矩阵转换为旋转向量
    initial_rotation_vector, _ = cv2.Rodrigues(initial_extrinsic_matrix[:, :3])

    # 将旋转向量和平移向量合并为一个参数向量
    initial_params = np.hstack((initial_rotation_vector.flatten(), initial_extrinsic_matrix[:, 3].flatten()))

    # 使用梯度下降优化外参矩阵
    result = minimize(reprojection_error, initial_params, args=(coordinates_array_lidar, coordinates_array_camera, intrinsic_matrix), method='L-BFGS-B')

    # 获取优化后的参数
    optimized_params = result.x

    # 将优化后的参数分解为旋转向量和平移向量
    optimized_rotation_vector = optimized_params[:3].reshape((3, 1))
    optimized_translation_vector = optimized_params[3:].reshape((3, 1))

    # 将旋转向量转换为旋转矩阵
    optimized_rotation_matrix, _ = cv2.Rodrigues(optimized_rotation_vector)

    # 组合旋转矩阵和平移向量为最终的外参矩阵
    optimized_extrinsic_matrix = np.hstack((optimized_rotation_matrix, optimized_translation_vector))

    print("Optimized Extrinsic Matrix:")
    print(optimized_extrinsic_matrix)

# Define the loss function with regularization
def reprojection_error_with_regularization(params, object_points, image_points, intrinsic_matrix, lambda_reg):
    error = reprojection_error(params, object_points, image_points, intrinsic_matrix)
    rotation_vector = params[:3]
    translation_vector = params[3:]
    regularization = lambda_reg * (np.sum(rotation_vector**2) + np.sum(translation_vector**2))
    return error + regularization

# Define the reprojection error function
def reprojection_error(params, object_points, image_points, intrinsic_matrix):
    rotation_vector = params[:3].reshape((3, 1))
    translation_vector = params[3:].reshape((3, 1))
    rotation_matrix, _ = cv2.Rodrigues(rotation_vector)
    projected_points, _ = cv2.projectPoints(object_points, rotation_vector, translation_vector, intrinsic_matrix, None)
    error = image_points - projected_points.squeeze()
    return np.sum(error**2)

# initial_maxtrix = pnpmethod(coordinates_array_lidar,coordinates_array_camera)
# getfinal_extrinsic_maxtrix(initial_maxtrix)

# Get the initial extrinsic matrix from the PnP method


rotation_vector_init = np.random.normal(loc=0.0, scale=np.pi, size=(3, 1))
translation_vector_init = np.random.normal(loc=0.0, scale=5.0, size=(3, 1))


# 合并为初始参数向量
initial_params_random = np.hstack((rotation_vector_init.flatten(), translation_vector_init.flatten()))

# Set the regularization coefficient
lambda_reg = 0.2


result_with_reg_random_init = minimize(reprojection_error_with_regularization, initial_params_random, args=(coordinates_array_lidar, coordinates_array_camera, intrinsic_matrix, lambda_reg), method='L-BFGS-B')

# Extract the optimized parameters
optimized_params_with_reg = result_with_reg_random_init.x
optimized_rotation_vector_with_reg = optimized_params_with_reg[:3].reshape((3, 1))
optimized_translation_vector_with_reg = optimized_params_with_reg[3:].reshape((3, 1))

# Convert the rotation vector to a rotation matrix
optimized_rotation_matrix_with_reg, _ = cv2.Rodrigues(optimized_rotation_vector_with_reg)

# Combine the rotation matrix and translation vector into the optimized extrinsic matrix with regularization
optimized_extrinsic_matrix_with_reg = np.hstack((optimized_rotation_matrix_with_reg, optimized_translation_vector_with_reg))

print("Optimized Extrinsic Matrix with Regularization:")
print(optimized_extrinsic_matrix_with_reg.tolist())
