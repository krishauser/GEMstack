import numpy as np
import open3d as o3d

# Load the .npz file
data = np.load('data/foam_block/lidar1.npz')
lidar_points = data['arr_0']  # Replace 'arr_0' with the actual key.

# Create a point cloud object
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(lidar_points[:, :3])

# Save the point cloud in .ply format
o3d.io.write_point_cloud("output_point_cloud.ply", pcd)

print("Point cloud saved, you can now open it in MeshLab.")
