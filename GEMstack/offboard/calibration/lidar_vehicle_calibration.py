import os
import numpy as np
import open3d as o3d

"""
Lidar frame to vehicle frame calibration.
References:
    - Open3D: https://www.open3d.org/docs/latest/tutorial/Basic/pointcloud.html 
    - GEMe2 hardware: https://publish.illinois.edu/robotics-autonomy-resources/gem-e2/hardware/ 
"""

# load files
data_idx = 9
# lidar_data_path = f'/Users/baoyu/baoyu/GEMstack/data/calibration_step2/lidar{data_idx}.npz'
lidar_data_path = f'/Users/baoyu/baoyu/GEMstack/data/calibration_data/lidar{data_idx}.npz' #TODO: change it to uniform path
lidar_data = np.load(lidar_data_path)
lidar_points = lidar_data['arr_0']

lidar_picked_points = np.load(f'/Users/baoyu/baoyu/GEMstack/data/calibration_points/lidar/lidar{data_idx}_picked_points.npy')
print('lidar_picked_points:', lidar_picked_points)

# convert to point cloud
lidar_pcd = o3d.geometry.PointCloud()
lidar_pcd.points = o3d.utility.Vector3dVector(lidar_points)

# crop the point cloud
lidar_pcd_2 = lidar_pcd.select_by_index(lidar_picked_points)
crop_min_bound = np.min(lidar_pcd_2.points, axis=0) - 0.1
crop_max_bound = np.max(lidar_pcd_2.points, axis=0) + 0.1
print('crop_min_bound:', crop_min_bound, 'crop_max_bound:', crop_max_bound)
cropped_lidar_pcd = lidar_pcd.crop(o3d.geometry.AxisAlignedBoundingBox(crop_min_bound, crop_max_bound))
# o3d.visualization.draw_geometries([cropped_lidar_pcd])

# plane segmentation
# plane_model, inliers = cropped_lidar_pcd.segment_plane(distance_threshold=0.01, ransac_n=3, num_iterations=1000)
# [a, b, c, d] = plane_model
# print(f'Plane model: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0')
# inlier_cloud = cropped_lidar_pcd.select_by_index(inliers)
# inlier_cloud.paint_uniform_color([1.0, 0, 0])
# outlier_cloud = cropped_lidar_pcd.select_by_index(inliers, invert=True)
# o3d.visualization.draw_geometries([inlier_cloud, outlier_cloud])

# [a, b, c, d] = np.array([-0.03852875, -0.01000802,  0.99920737, 2.1135])
# print(f'Plane model: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0')

# normal_vector = np.array([a, b, c])
# print('normal_vector:', normal_vector)

# z_axis = np.array([1, 0, 0])
# rotation_axis = np.cross(z_axis, normal_vector)
# rotation_angle = np.arccos(np.dot(z_axis, normal_vector) / (np.linalg.norm(z_axis) * np.linalg.norm(normal_vector)))
# # get 3x3 rotation matrix from rotation angle
# rotation_matrix = np.array([[np.cos(rotation_angle), -np.sin(rotation_angle), 0],
#                             [np.sin(rotation_angle), np.cos(rotation_angle), 0],
#                             [0, 0, 1]])
# print('rotation_matrix:\n', rotation_matrix)

## find translation vector
"""
manual measurement:

lidar - vehicle
x = 85cm
y = 0cm
z = 168cm

"""
translation_vector = np.array([0.85, 0, 1.68])
print('translation_vector:', translation_vector)

rotation_matrix = np.array([[0.9998, 0.0185, 0.0007],
                            [-0.0186, 0.999, 0.0397],
                            [0, -0.0398, 0.9992]])

lidar_vehicle_calibration = np.hstack((rotation_matrix, translation_vector.reshape(-1, 1)))
lidar_vehicle_calibration = np.vstack((lidar_vehicle_calibration, np.array([0, 0, 0, 1])))
print('lidar_vehicle_calibration:\n', lidar_vehicle_calibration)

# save calibration matrix
calibration_matrix_path = '/Users/baoyu/baoyu/GEMstack/data/lidar_vehicle_calibration.txt'
np.savetxt(calibration_matrix_path, lidar_vehicle_calibration, delimiter=',', fmt='%1.4f')

# icp alignment transformation
"""
camera - lidar

x = 14cm
y = 0cm
z = -10cm

"""
icp_translation = np.array([0.14, 0, -0.1])
icp_transformation = np.array([
[-1.44925973e-03, -7.78625378e-04,  9.99998647e-01, -6.62124083e-03],
[-9.99996805e-01, -2.06991791e-03, -1.45086876e-03,  1.56870385e-02],
[ 2.07104479e-03, -9.99997555e-01, -7.75623042e-04,  5.58669647e-04],
[ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.00000000e+00],
])
icp_rotation = icp_transformation[:3, :3]
lidar_zed_calibration = np.hstack((icp_rotation, icp_translation.reshape(-1, 1)))
lidar_zed_calibration = np.vstack((lidar_zed_calibration, np.array([0, 0, 0, 1])))
print('lidar_zed_calibration:\n', lidar_zed_calibration)

# save calibration matrix
icp_matrix_path = '/Users/baoyu/baoyu/GEMstack/data/lidar_zed_calibration.txt'
np.savetxt(icp_matrix_path, lidar_zed_calibration, delimiter=',', fmt='%1.4f')

# camera - vehicle
camera_vehicle_calibration = lidar_vehicle_calibration @ lidar_zed_calibration
print('camera_vehicle_calibration:\n', camera_vehicle_calibration)
camera_vehicle_calibration_path = '/Users/baoyu/baoyu/GEMstack/data/zed_vehicle_calibration.txt'
np.savetxt(camera_vehicle_calibration_path, camera_vehicle_calibration, delimiter=',', fmt='%1.4f')

