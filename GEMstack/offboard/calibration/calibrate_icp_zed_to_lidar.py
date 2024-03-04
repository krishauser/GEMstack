import open3d as o3d
import numpy as np
import copy, os
from klampt.model.sensing import image_to_points
import cv2
from klampt.io import numpy_convert
from klampt.math import se3, so3

"""
check out https://www.open3d.org/html/tutorial/Advanced/interactive_visualization.html
"""

# This comes from camera info
zed_K = np.array([[527.1734008789062, 0.0, 616.2451782226562],
                [0.0, 527.1734008789062, 359.2149353027344],
                [0.0, 0.0, 1.0]])
zed_intrinsics = [zed_K[0,0],zed_K[1,1],zed_K[0,2],zed_K[1,2]]
zed_w = 1280
zed_h = 720

zed_xfov = 2*np.arctan(zed_w/(2*zed_intrinsics[0]))
zed_yfov = 2*np.arctan(zed_h/(2*zed_intrinsics[1]))

def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0, 0])
    target_temp.paint_uniform_color([0, 0, 1])
    source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target_temp])

path = "data/new_foam3/"
lidar_pattern = os.path.join(path,"lidar2.npz")
depth_pattern = os.path.join(path,"depth2.tif")

arr_compressed = np.load(lidar_pattern)
arr = arr_compressed['arr_0']
arr_compressed.close()
pc = numpy_convert.from_numpy(arr,'PointCloud')

depth = cv2.imread(depth_pattern,cv2.IMREAD_UNCHANGED)
depth = depth.astype(np.float32)

source_pts = image_to_points(depth,None,zed_xfov,zed_yfov,depth_scale=4000.0/0xffff, points_format='numpy')
# transfer to point cloud

source_pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(source_pts))
target_pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(arr))

threshold = 0.02
trans_init = np.array([[0,0,1, 0],[-1,0,0, 0],[0,-1,0, 0], [0, 0, 0, 1]])

draw_registration_result(source_pcd, target_pcd, trans_init)
print("Apply point-to-point ICP")
reg_p2p = o3d.pipelines.registration.registration_icp(
    source_pcd, target_pcd, threshold, trans_init,
    o3d.pipelines.registration.TransformationEstimationPointToPoint())

print(reg_p2p)
print("Transformation is:")
print(reg_p2p.transformation)
print("")

draw_registration_result(source_pcd, target_pcd, trans_init)

translation = 'lidar2_translation.npy'

np.save(os.path.join(path, translation), trans_init)

