""" 
    This script is for selecting point cloud by rect box.

    ref:
        https://medium.com/@rdadlaney/basics-of-3d-point-cloud-data-manipulation-in-python-95b0a1e1941e

    With draw_geometries_with_editing, one can view the x,y,z co-ordinates of an individual point. 
    Press “Shift+left mouse click” to view co-ordinates. 
    “Shift + mouse left drag-> Defines a rectangle, which will add all the points in it
"""
import open3d as o3d
import numpy as np
import pandas as pd

# load file
# data_idx = 4
# image = cv2.imread(f'data/people_with_board/color{data_idx}.png')

lidar_data = np.load('data/people_with_board/lidar4.npz')
point_cloud_np = lidar_data['arr_0']

# turn numpy to pointcloud
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(point_cloud_np)

# vis = o3d.visualization.VisualizerWithEditing()
vis = o3d.visualization.draw_geometries_with_vertex_selection([pcd])
# vis.create_window()
# vis.add_geometry(pcd)
# vis.run()
# vis.destroy_window()

# picekd feature point
picked_points = vis.get_picked_points()
for index in picked_points:
    print(f"Point index: {index}, coordinates: {pcd.points[index]}")

# # Save points as csv
# picked_coords = np.asarray([pcd.points[i] for i in picked_points])
# picked_df = pd.DataFrame(picked_coords, columns=['x', 'y', 'z'])
# csv_file_path = 'save/pointcld1_stopsign.csv'
# picked_df.to_csv(csv_file_path, index=False)
# print(f"All recorded points have been saved to {csv_file_path}.")



