# import numpy as np

# import os
# import open3d as o3d

# # 加载点云数据
# data = np.load('data/hw3_data/lidar1.npz')
# points = data['arr_0']
# pcd = o3d.geometry.PointCloud()
# pcd.points = o3d.utility.Vector3dVector(points)

# # 可视化点云并选择点
# o3d.visualization.draw_geometries_with_editing([pcd])


import numpy as np
import open3d as o3d

# 加载点云数据
# data = np.load('data/hw3_data/lidar1.npz')
data = np.load('data/hw3_data/lidar7.npz')
points = data['arr_0']

# 创建 PointCloud 对象并设置点云数据
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(points)

# 创建一个 VisualizerWithVertexSelection 对象
vis = o3d.visualization.VisualizerWithVertexSelection()

# 设置窗口和添加几何体
vis.create_window()
vis.add_geometry(pcd)

# 运行可视化器，用户可以选择点
vis.run()

# 关闭窗口
vis.destroy_window()




