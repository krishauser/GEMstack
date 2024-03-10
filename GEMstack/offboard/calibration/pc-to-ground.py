# -*- coding: utf-8 -*-
"""
Created on Mon Feb 26 19:25:58 2024

@author: Zhengyuan Li
"""

import open3d as o3d
import numpy as np
import copy
import open3d.core as o3c
import sys

def display_inlier_outlier(cloud : o3d.t.geometry.PointCloud, mask : o3c.Tensor):
    inlier_cloud = cloud.select_by_mask(mask)
    outlier_cloud = cloud.select_by_mask(mask, invert=True)

    print("Showing outliers (red) and inliers (gray): ")
    outlier_cloud = outlier_cloud.paint_uniform_color([1.0, 0, 0])
    inlier_cloud.paint_uniform_color([0.8, 0.8, 0.8])
    inlier_cloud = o3d.visualization.draw_geometries([inlier_cloud.to_legacy(), outlier_cloud.to_legacy()],
                                      zoom=0.3412,
                                      front=[0.4257, -0.2125, -0.8795],
                                      lookat=[2.6172, 2.0475, 1.532],
                                      up=[-0.0694, -0.9768, 0.2024])
target = np.load(r"/GEMstack/data/calibration_pcd/lidar2.npy")
target = o3d.t.geometry.PointCloud(target)
# print(target)
# aabb = target.get_axis_aligned_bounding_box()
# print(aabb)
# obb = target.get_oriented_bounding_box()
# box_points = obb.get_box_points()
# print(box_points)
# draw points
# lines = o3d.geometry.LineSet.create_from_oriented_bounding_box(obb.to_legacy())
# print(lines)
target.paint_uniform_color([1, 0, 0])
print("Statistical oulier removal")
cl, ind = target.remove_radius_outliers(nb_points=8,
                                                     search_radius=0.15)
print(target)
target = target.select_by_mask(ind)

# display_inlier_outlier(target, ind)
print(target)
aabb = target.get_axis_aligned_bounding_box()
print(aabb)
obb = target.get_oriented_bounding_box()
box_points = obb.get_box_points()
print(box_points)
points = box_points.numpy()
lines = [[3, 4], [4, 5], [5,6], [6, 3]]
line_set = o3d.geometry.LineSet()
line_set.points = o3d.utility.Vector3dVector(points)
line_set.lines = o3d.utility.Vector2iVector(lines)


# print(aabb_pc)
o3d.visualization.draw_geometries([line_set, target.to_legacy()])

# print(obb.get_box_points())
# print('center')
# print(obb.to_legacy().get_center())
# ground_plane = box_points[0]
# # plane_model, inliers = target.segment_plane(distance_threshold=5,
# #                                          ransac_n=50,
# #                                          num_iterations=1000)
# # [a, b, c, d] = plane_model
pc_sparse = np.array(points)[[3,4,5,6]]
plane = np.array([[1,1,0], [1, 0,0], [0,1,0]])
source = o3d.t.geometry.PointCloud(pc_sparse)
target =  o3d.t.geometry.PointCloud(plane)
plane_model, inliers = source.segment_plane(distance_threshold=0.1,
                                            ransac_n=3,
                                            num_iterations=10,
                                            probability=0.99)
print(inliers)
[a, b, c, d] = plane_model.numpy().tolist()
print(plane_model.numpy())
sys.exit(0)
# current_transformation = np.identity(4)
# print("2. Point-to-plane ICP registration is applied on original point")
# print("   clouds to refine the alignment. Distance threshold 0.02.")
# result_icp = o3d.pipelines.registration.registration_icp(
#     source.to_legacy(), target.to_legacy(), 0.5, current_transformation,
#     o3d.pipelines.registration.TransformationEstimationPointToPlane())