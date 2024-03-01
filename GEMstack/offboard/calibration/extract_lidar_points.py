import os
import numpy as np
import open3d as o3d

"""
Manully extract the feature points from the lidar point cloud. 
"""

# load file
data_idx = 9
# lidar_data_path = f'/Users/baoyu/baoyu/GEMstack/data/calibration_step2/lidar{data_idx}.npz'
lidar_data_path = f'/Users/baoyu/baoyu/GEMstack/data/calibration_data/lidar{data_idx}.npz' #TODO: change it to uniform path
lidar_data = np.load(lidar_data_path)
lidar_points = lidar_data['arr_0']

# convert to point cloud
lidar_pcd = o3d.geometry.PointCloud()
lidar_pcd.points = o3d.utility.Vector3dVector(lidar_points)

# editable visualization
vis = o3d.visualization.VisualizerWithEditing()
vis.create_window()
vis.add_geometry(lidar_pcd)
vis.run()
vis.destroy_window()

# pick feature points
# https://www.open3d.org/html/tutorial/Advanced/interactive_visualization.html 
# "shift + - + mouse button" to pick points
picked_points = vis.get_picked_points()
for idx in picked_points:
    print(f'point {idx} coordinate: {lidar_pcd.points[idx]}')

# Two stacked foam cubes on the ground
# point 14558 coordinate: [ 3.58423948  0.48142311 -0.96901858]
# point 15299 coordinate: [ 3.59164429 -0.10722492 -0.96280694]
# point 14576 coordinate: [ 3.58456111  0.46746051 -0.8345685 ]
# point 15301 coordinate: [ 3.58601427 -0.10768328 -0.82826984]
# point 15319 coordinate: [ 3.58875561 -0.12093244 -0.69797933]
# point 14578 coordinate: [ 3.56461716  0.46422872 -0.69874257]

# save picked points
picked_points = np.array(picked_points)
picked_points_coords = np.array([lidar_pcd.points[idx] for idx in picked_points])
save_path = '/Users/baoyu/baoyu/GEMstack/data/calibration_points/lidar' #TODO: change it to uniform path
os.makedirs(save_path, exist_ok=True)
np.save(os.path.join(save_path, f'lidar{data_idx}_picked_points.npy'), picked_points)
np.save(os.path.join(save_path, f'lidar{data_idx}_picked_points_coords.npy'), picked_points_coords)
