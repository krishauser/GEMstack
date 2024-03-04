from klampt import vis
from klampt.math import so3,se3
from klampt.vis.colorize import colorize
from klampt import PointCloud,Geometry3D
from klampt.io import numpy_convert
from klampt.model.sensing import image_to_points
import cv2
import os
import numpy as np
import math
import time
import open3d as o3d

#from sklearn.decomposition import PCA

#uncalibrated values -- TODO: load these from a calibration file
zed_K = np.array([[527.5779418945312, 0.0, 616.2459716796875],
                [0.0, 527.5779418945312, 359.2155456542969],
                [0.0, 0.0, 1.0]])
zed_intrinsics = [zed_K[0,0],zed_K[1,1],zed_K[0,2],zed_K[1,2]]
zed_w = 1280
zed_h = 720

def execute_fast_global_registration(source_down, target_down, source_fpfh,
                                     target_fpfh, voxel_size):
    distance_threshold = voxel_size * 0.5
    #print(":: Apply fast global registration with distance threshold %.3f" \
    #        % distance_threshold)
    result = o3d.pipelines.registration.registration_fgr_based_on_feature_matching(
        source_down, target_down, source_fpfh, target_fpfh,
        o3d.pipelines.registration.FastGlobalRegistrationOption(
            maximum_correspondence_distance=distance_threshold))
    return result

def preprocess_point_cloud(pcd, voxel_size):
    #print(":: Downsample with a voxel size %.3f." % voxel_size)
    pcd_down = pcd.voxel_down_sample(voxel_size)

    radius_normal = voxel_size * 2
    #print(":: Estimate normal with search radius %.3f." % radius_normal)
    pcd_down.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))

    radius_feature = voxel_size * 5
    #print(":: Compute FPFH feature with search radius %.3f." % radius_feature)
    pcd_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        pcd_down,
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))    
    return pcd_down, pcd_fpfh

def open3d_icp(pc_0, pc_1, init=None):
    pc_0.estimate_normals(
                search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
    pc_1.estimate_normals(
                search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
    source_down, source_fpfh = preprocess_point_cloud(pc_0, 0.5)
    target_down, target_fpfh = preprocess_point_cloud(pc_1, 0.5)

    if(init is None):
        result_fast = execute_fast_global_registration(source_down, target_down,
                                               source_fpfh, target_fpfh,
                                               0.5)
    
        trans_init = result_fast.transformation
    else:
        trans_init = init


    reg_p2l = o3d.pipelines.registration.registration_icp(pc_0, pc_1, 0.02, trans_init, o3d.pipelines.registration.TransformationEstimationPointToPoint())
    final_transform = reg_p2l.transformation

    return final_transform

def main(folder):
    lidar_xform = se3.identity()
    lidar_pattern = os.path.join(folder,"lidar{}.npz")
    data = {}
    def load_and_show_scan(idx):
        arr_compressed = np.load(lidar_pattern.format(idx))
        arr = arr_compressed['arr_0']
        arr_compressed.close()

        data['lidar_points'] = arr

    data['index'] = 1
    def increment_index():
        data['index'] += 1
        try:
            load_and_show_scan(data['index'])
        except Exception:
            data['index'] -= 1
            return
    def decrement_index():
        data['index'] -= 1
        try:
            load_and_show_scan(data['index'])
        except Exception:
            data['index'] += 1
            return

    increment_index()


    lidar_pts_orig = data['lidar_points']
    
    lidar_pts = np.copy(lidar_pts_orig)

    x_min = 1.0
    x_max = 4.0

    y_min = -1.0
    y_max = 2.0


    z_min = -3.0
    z_max = 3.0
    

    lidar_pts = lidar_pts[lidar_pts[:,1] > y_min]
    lidar_pts = lidar_pts[lidar_pts[:,1] < y_max]

    lidar_pts = lidar_pts[lidar_pts[:,0] > x_min]
    lidar_pts = lidar_pts[lidar_pts[:,0] < x_max]

    lidar_pts = lidar_pts[lidar_pts[:,2] > z_min]
    lidar_pts = lidar_pts[lidar_pts[:,2] < z_max]
    
    #print('tes')

    lidar_pc = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(lidar_pts))
    lidar_pc_orig = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(lidar_pts_orig))

    lidar_pc_orig.paint_uniform_color([1,0,0])
    lidar_pc.paint_uniform_color([0,1,0])

    #o3d.visualization.draw_geometries([lidar_pc, lidar_pc_orig])


    points_mean = lidar_pts.mean(0)
    points_central = lidar_pts - points_mean
    cov_matrix = np.cov(points_central, rowvar=False)
    eigenvalues, eigenvectors = np.linalg.eigh(
        cov_matrix
    )

    plane_normal = eigenvectors[:,0]
    plane_normal = plane_normal[[0, 2]]

    elevation_angle_down = math.acos((plane_normal @ np.array([1,0]))/(np.linalg.norm(plane_normal)))
    elevation_angle_down_deg = elevation_angle_down / (np.pi / 180)


    lidar_rotation = np.array([
        [np.cos(elevation_angle_down), 0, np.sin(elevation_angle_down), 0],
        [0, 1, 0, 0],
        [-np.sin(elevation_angle_down), 0, np.cos(elevation_angle_down), 0],
        [0, 0, 0, 1]
    ])

    #lidar_rotation = np.linalg.inv(lidar_rotation)

    # Z axis: (74.5 + 3.5 - 10.6) = 67.4 inches = 1.71196 meters
    # X axis: (30.25 inches) = 0.76835 meters

    lidar_zx = np.array([
        [1, 0, 0, 0.76835],
        [0, 1, 0, 0],
        [0, 0, 1, 1.71196],
        [0, 0, 0, 1]
    ])

    lidar_x = np.array([
        [1, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])

    final = lidar_zx @ lidar_rotation

    adjusted_lidar = np.ones((lidar_pts_orig.shape[0], 4))
    adjusted_lidar[:,:3] = lidar_pts_orig
    
    adjusted_lidar = (final @ adjusted_lidar.T).T

    coordinate = o3d.geometry.TriangleMesh.create_coordinate_frame()
    adjusted_lidar_pc = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(adjusted_lidar[:,:3]))
    adjusted_lidar_pc.paint_uniform_color([1,0,0])

    o3d.visualization.draw_geometries([coordinate, adjusted_lidar_pc])


    print('done')



if __name__ == '__main__':
    import sys
    folder = 'data/flatwall/'
    if len(sys.argv) >= 2:
        folder = sys.argv[1]
    main(folder)
