import open3d as o3d
import numpy as np
import pandas as pd
import pathlib
import os

OUTPUT_DIR = 'save'
data_idx = 6
lidar_fn = f'data/step1/lidar{data_idx}.npz'

# load file
lidar_data = np.load(lidar_fn)
point_cloud_np = lidar_data['arr_0']

# turn numpy to pointcloud
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(point_cloud_np)

vis = o3d.visualization.VisualizerWithEditing()
vis.create_window()
vis.add_geometry(pcd)
vis.run()
vis.destroy_window()

# picekd feature point
picked_points = vis.get_picked_points()
for index in picked_points:
    print(f"Point index: {index}, coordinates: {pcd.points[index]}")

# Save points as csv
picked_coords = np.asarray([pcd.points[i] for i in picked_points])
picked_df = pd.DataFrame(picked_coords, columns=['x', 'y', 'z'])

pathlib.Path(OUTPUT_DIR).mkdir(parents=True, exist_ok=True)
csv_file_path = os.path.join(OUTPUT_DIR, f'pointcld{data_idx}.csv')
picked_df.to_csv(csv_file_path, index=False)
print(f"All recorded points have been saved to {csv_file_path}.")



