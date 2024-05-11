import os
import numpy as np
import open3d as o3d

path = 'data/new_foam3/' 
file_type = 'lidar2.npz'
file_type_perpendicular_points = 'lidar2_picked_points.npy' # indices of points in parallel points
lidar_points = np.load(path + file_type)['arr_0']

perp_point_indices = np.load(path + file_type_perpendicular_points)

all_points = o3d.geometry.PointCloud()
all_points.points = o3d.utility.Vector3dVector(lidar_points)

white_board_point_cloud = all_points.select_by_index(perp_point_indices)
wb_min = np.min(white_board_point_cloud.points, axis=0) - 0.1
wb_max = np.max(white_board_point_cloud.points, axis=0) + 0.1

cropped_lidar_pcd = all_points.crop(o3d.geometry.AxisAlignedBoundingBox(wb_min, wb_max))

plane_model, inliers = cropped_lidar_pcd.segment_plane(distance_threshold=0.01, ransac_n=3, num_iterations=1000)

[a, b, c, d] = plane_model

normal_3 = np.array([a, b, c])

z_axis = np.array([1, 0, 0])
rotation_axis = np.cross(z_axis, normal_3)
rotation_angle = np.arccos(np.dot(z_axis, normal_3) / (np.linalg.norm(z_axis) * np.linalg.norm(normal_3)))
# get 3x3 rotation matrix from rotation angle
rotation = np.array([
    [np.cos(rotation_angle), -np.sin(rotation_angle), 0],\
    [np.sin(rotation_angle), np.cos(rotation_angle), 0],\
    [0, 0, 1]\
    ])
print(rotation) # rotation

translation_vector = np.array([[0.85], [0], [1.68]]) # measured
# print('translation_vector:', translation_vector)

lidar_vehicle_tranformation = np.hstack((rotation, translation_vector))
lidar_vehicle_tranformation = np.vstack((lidar_vehicle_tranformation, np.array([0, 0, 0, 1])))
print('lidar_vehicle_calibration:\n', lidar_vehicle_tranformation)

# save calibration matrix
calibration_matrix_path = 'data/lidar_vehicle_calibration.txt'
np.savetxt(calibration_matrix_path, lidar_vehicle_tranformation, delimiter=',', fmt='%1.4f')

icp_translation = np.array([[0.14], [0], [-0.1]]) # measured
icp_transformation = np.load(path + 'lidar2_translation.npy')
icp_rotation = icp_transformation[:3, :3]
lidar_zed_calibration = np.hstack((icp_rotation, icp_translation))
lidar_zed_calibration = np.vstack((lidar_zed_calibration, np.array([0, 0, 0, 1])))

print(lidar_zed_calibration) # lidar to zed transformation using icp and measured distances

# save calibration matrix
icp_matrix_path = 'data/lidar_zed_calibration.txt'
np.savetxt(icp_matrix_path, lidar_zed_calibration, delimiter=',', fmt='%1.4f')

# camera - vehicle
camera_vehicle_calibration = lidar_vehicle_tranformation @ lidar_zed_calibration
print(camera_vehicle_calibration) # camera to vehicle transformation
camera_vehicle_calibration_path = 'data/zed_vehicle_calibration.txt'
np.savetxt(camera_vehicle_calibration_path, camera_vehicle_calibration, delimiter=',', fmt='%1.4f')