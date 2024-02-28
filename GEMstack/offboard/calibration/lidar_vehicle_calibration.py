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

lidar_picked_points = np.load(f'/Users/baoyu/baoyu/GEMstack/data/calibration_points/step2/lidar{data_idx}_picked_points.npy')
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
plane_model, inliers = cropped_lidar_pcd.segment_plane(distance_threshold=0.01, ransac_n=3, num_iterations=1000)
[a, b, c, d] = plane_model
print(f'Plane model: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0')
# inlier_cloud = cropped_lidar_pcd.select_by_index(inliers)
# inlier_cloud.paint_uniform_color([1.0, 0, 0])
# outlier_cloud = cropped_lidar_pcd.select_by_index(inliers, invert=True)
# o3d.visualization.draw_geometries([inlier_cloud, outlier_cloud])

normal_vector = np.array([a, b, c])
print('normal_vector:', normal_vector)

## find rotation matrix
# assume axis is (x,y,z)
z_axis = np.array([0, 0, 1])
rotation_axis = np.cross(z_axis, normal_vector)
rotation_angle = np.arccos(np.dot(z_axis, normal_vector) / (np.linalg.norm(z_axis) * np.linalg.norm(normal_vector)))
# get 3x3 rotation matrix from rotation angle
rotation_matrix = np.array([[np.cos(rotation_angle), -np.sin(rotation_angle), 0],
                            [np.sin(rotation_angle), np.cos(rotation_angle), 0],
                            [0, 0, 1]])
print('rotation_matrix:\n', rotation_matrix)

## find translation vector
# vehicle height: 196cm, lidar z: 155cm, ground z: 155-196=-41cm
# Measure the height of the rear axle over the ground: ?cm
# cube width: 0.1524m, cube height: 0.1524m, cube depth: 0.1524m
# assume the objects on the centerline of the vehicle (y=0).
# x = 1m, z = 
# trans_vector = np.array([0, 0, 0.1524])