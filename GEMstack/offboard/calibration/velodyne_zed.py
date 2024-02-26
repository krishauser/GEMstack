"""To compute the relative transform that converts points from the velodyne to the zed frame

References:
- https://www.open3d.org/html/tutorial/Advanced/interactive_visualization.html
"""

import numpy as np
import open3d as o3d
import matplotlib.pyplot as plt

lidar_data = np.load('../../offboard/calibration/lidar8.npz')['arr_0']

point_cloud = o3d.geometry.PointCloud()
point_cloud.points = o3d.utility.Vector3dVector(lidar_data)

vis = o3d.visualization.VisualizerWithVertexSelection()
vis.create_window()
vis.add_geometry(point_cloud)
vis.run()
vis.destroy_window()
points = vis.get_picked_points() # each point is an object of open3d.visualization.PickedPoint class

indices = [point.index for point in points]
coords = [point.coord for point in points]

print(coords)
