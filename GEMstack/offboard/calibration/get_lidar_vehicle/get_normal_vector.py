import open3d as o3d
import numpy as np

pcd = o3d.io.read_point_cloud("data/hw3_data/step3/lidar1.npz")
o3d.visualization.draw_geometries([pcd])
plane_model, inliers = pcd.segment_plane(distance_threshold=0.01,
                                         ransac_n=3,
                                         num_iterations=1000)

inlier_cloud = pcd.select_by_index(inliers)
o3d.visualization.draw_geometries([inlier_cloud])
o3d.io.write_point_cloud("plane.pcd", inlier_cloud)
