import os
import numpy as np
import open3d as o3d

path = 'data/new_foam3/'
file_type = 'lidar2.npz'
lidar_points = np.load(path + file_type)['arr_0']

# convert to point cloud
point_cloud = o3d.geometry.PointCloud()
point_cloud.points = o3d.utility.Vector3dVector(lidar_points)

# editable visualization
vis = o3d.visualization.VisualizerWithEditing()
vis.create_window()
vis.add_geometry(point_cloud)
vis.run()
vis.destroy_window()

"""
https://www.open3d.org/html/tutorial/Advanced/interactive_visualization.html
    1) Please pick at least three correspondences using [shift + left click]
    OPTIONAL press [shift + right click] to undo point picking
    2) Afther picking points, press q for close the window
"""
picked_points = np.array(vis.get_picked_points())
picked_points_coords = np.array([point_cloud.points[idx] for idx in picked_points])
os.makedirs(path, exist_ok=True)
np.save(os.path.join(path, 'lidar2_picked_points.npy'), picked_points)