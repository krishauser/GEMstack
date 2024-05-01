import sys
import os
import open3d as o3d
import numpy as np

def main(data_folder, idx):
    lidar_path = os.path.join(data_folder, 'lidar{}.npz'.format(idx))
    lidar_data = np.load(lidar_path)['arr_0']

    lidar_pcd = o3d.geometry.PointCloud()
    lidar_pcd.points = o3d.utility.Vector3dVector(lidar_data)

    vis = o3d.visualization.Visualizer()
    vis.create_window()
    vis.add_geometry(lidar_pcd)
    vis.run()
    vis.destroy_window()
    
if __name__ == '__main__':
    data_folder = sys.argv[1]
    idx = sys.argv[2]
    main(data_folder, idx)
