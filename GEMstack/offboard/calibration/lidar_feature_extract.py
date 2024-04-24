import open3d as o3d
import numpy as np
import pandas as pd
import pathlib
import os

import argparse
parser = argparse.ArgumentParser()

parser.add_argument('--output_dir', type=str, default='save')
parser.add_argument('--output_fn', '-o', type=str, default='lidar.csv')
parser.add_argument('--src_fn', '-s', type=str, required=True)
args = parser.parse_args()

assert args.output_fn.endswith('csv')

# OUTPUT_DIR = 'save'
# data_idx = 6
# src_fn = f'data/step1/lidar{data_idx}.npz'

# load file
lidar_data = np.load(args.src_fn)
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

pathlib.Path(args.output_dir).mkdir(parents=True, exist_ok=True)
csv_file_path = os.path.join(args.output_dir, args.output_fn)
picked_df.to_csv(csv_file_path, index=False)
print(f"All recorded points have been saved to {csv_file_path}.")



