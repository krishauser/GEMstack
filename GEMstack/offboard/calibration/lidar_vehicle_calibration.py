import open3d as o3d
import numpy as np

# load file
# load file
lidar_data = np.load('hw3_step2_data/lidar6.npz')
point_cloud_np = lidar_data['arr_0']
# turn numpy to pointcloud
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(point_cloud_np)

# visualize
vis = o3d.visualization.VisualizerWithEditing()
vis.create_window()
vis.add_geometry(pcd)
vis.run() 
vis.destroy_window()
selected_indices = vis.get_picked_points()  

if not selected_indices:
    print("pick bounding points")
else:
    selected_pcd = pcd.select_by_index(selected_indices)
    min_bound = selected_pcd.get_min_bound() - 0.1 
    max_bound = selected_pcd.get_max_bound() + 0.1 
    bounding_box = o3d.geometry.AxisAlignedBoundingBox(min_bound, max_bound)

    cropped_pcd = pcd.crop(bounding_box)
    o3d.visualization.draw_geometries([cropped_pcd])

# plane segmentation
plane_model, inliers = cropped_pcd.segment_plane(distance_threshold=0.01,
                                         ransac_n=4,
                                         num_iterations=1000)
[a, b, c, d] = plane_model
print(f"plan function: {a}x + {b}y + {c}z + {d} = 0")

# find wall
wall_plane = pcd.select_by_index(inliers)
normal_vector = np.array([a, b, c])
print("normal_vector of wall: ", normal_vector)

# find roration matrix
# assume the vehicle's direction is (1,0,0)
x_axis = np.array([1, 0, 0])
angle = np.arccos(np.dot(normal_vector, x_axis) / (np.linalg.norm(normal_vector) * np.linalg.norm(x_axis)))
rotation_matrix = np.array([[np.cos(angle), -np.sin(angle), 0],
                            [np.sin(angle), np.cos(angle), 0],
                            [0, 0, 1]])

print("Rotation Matrix:")
print(rotation_matrix)

# find trasition vector
"""
x = 32.25 in
y = 0 in
z = 77.5 in - 9.5 in
"""
translation_vector_in = np.array([32.25, 0, (77.5 - 9.5)])
translation_vector_meters = translation_vector_in * 0.0254

# find transformation matrix
transform_matrix = np.eye(4)  
transform_matrix[:3, :3] = rotation_matrix  
transform_matrix[:3, 3] = translation_vector_meters  

print("Transformation Matrix:")
print(transform_matrix)

# save matrix
np.savetxt('save/T_Lidar2Vehicle.txt', transform_matrix, fmt='%.6f')

