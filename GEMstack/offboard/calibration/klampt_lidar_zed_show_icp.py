from klampt import vis
from klampt.math import so3,se3
from klampt.vis.colorize import colorize
from klampt import PointCloud,Geometry3D
from klampt.io import numpy_convert
from klampt.model.sensing import image_to_points
import GEMstack.offboard.calibration.extract_lidar_points as o3d
import cv2
import os
import numpy as np
import math
import time

#uncalibrated values -- TODO: load these from a calibration file
zed_K = np.array([[527.1734008789062, 0.0, 616.2451782226562],
                [0.0, 527.1734008789062, 359.2149353027344],
                [0.0, 0.0, 1.0]])
zed_intrinsics = [zed_K[0,0],zed_K[1,1],zed_K[0,2],zed_K[1,2]]
zed_w = 1280
zed_h = 720

def execute_fast_global_registration(source_down, target_down, source_fpfh, target_fpfh, voxel_size):
    distance_threshold = voxel_size * 0.5
    result = o3d.pipelines.registration.registration_fgr_based_on_feature_matching(
        source_down, target_down, source_fpfh, target_fpfh,
        o3d.pipelines.registration.FastGlobalRegistrationOption(
            maximum_correspondence_distance=distance_threshold))
    return result

def preprocess_pcd(pcd, voxel_size):
    # voxel down sample
    pcd_sample = pcd.voxel_down_sample(voxel_size)

    # normal estimation
    radius_normal = voxel_size * 2
    pcd_sample.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))

    # compute FPFH feature
    radius_feature = voxel_size * 5
    pcd_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        pcd_sample,
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))    
    return pcd_sample, pcd_fpfh

def icp(pc_0, pc_1, init=None):
    pc_0.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
    pc_1.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
    source_down, source_fpfh = preprocess_pcd(pc_0, 0.5)
    target_down, target_fpfh = preprocess_pcd(pc_1, 0.5)

    if init is None:
        result_fast = execute_fast_global_registration(source_down, target_down, source_fpfh, target_fpfh, 0.5)
        trans_init = result_fast.transformation
    else:
        trans_init = init

    # icp registration
    reg_p2l = o3d.pipelines.registration.registration_icp(pc_0, pc_1, 0.02, trans_init, o3d.pipelines.registration.TransformationEstimationPointToPoint())
    transform_matrix = reg_p2l.transformation

    return transform_matrix

def main(folder):
    lidar_xform = se3.identity()
    zed_xform = (so3.from_ndarray(np.array([[0,0,1],[-1,0,0],[0,-1,0]])),[0,0,0])
    zed_xform_numpy = np.array([[0, 0, 1, 0],[-1, 0, 0, 0],[0, -1, 0, 0], [0, 0, 0, 1]])
    lidar_pattern = os.path.join(folder,"lidar{}.npz")
    color_pattern = os.path.join(folder,"color{}.png")
    depth_pattern = os.path.join(folder,"depth{}.tif")
    points_pattern1 = os.path.join(folder,"lidar{}.npy")
    points_pattern2 = os.path.join(folder,"zed{}.npy")
    data = {}
    def load_and_show_scan(idx):
        arr_compressed = np.load(lidar_pattern.format(idx))
        arr = arr_compressed['arr_0']
        arr_compressed.close()
        # mask = arr[:,1] > 0
        # arr = arr[mask]

        pc = numpy_convert.from_numpy(arr,'PointCloud')
        pc = colorize(pc,'z','plasma')
        data['lidar'] = Geometry3D(pc)

        color = cv2.imread(color_pattern.format(idx))
        depth = cv2.imread(depth_pattern.format(idx),cv2.IMREAD_UNCHANGED)
        depth = depth.astype(np.float32)
        print("depth range",np.min(depth),np.max(depth))
        zed_xfov = 2*np.arctan(zed_w/(2*zed_intrinsics[0]))
        zed_yfov = 2*np.arctan(zed_h/(2*zed_intrinsics[1]))
        print("estimated zed horizontal FOV",math.degrees(zed_xfov),"deg")
        pc = image_to_points(depth,color,zed_xfov,zed_yfov,depth_scale=4000.0/0xffff, points_format='PointCloud')

        data['zed'] = Geometry3D(pc)
        data['lidar'].setCurrentTransform(*lidar_xform)
        data['zed'].setCurrentTransform(*zed_xform)
        
        # source_pts = image_to_points(depth,None,zed_xfov,zed_yfov,depth_scale=4000.0/0xffff, points_format='numpy')
        # target_pts = data['lidar'].getPointCloud().getPoints() # (25281, 3)
        # # data['zed'] = Geometry3D(numpy_convert.from_numpy(source_pts,'PointCloud'))
        # # data['zed'].setCurrentTransform(*zed_xform)

        # # save the point cloud
        # np.save(points_pattern1.format(idx), target_pts)
        # np.save(points_pattern2.format(idx), source_pts)

        data['lidar_points'] = arr
        data['zed_points'] = image_to_points(depth,None,zed_xfov,zed_yfov,depth_scale=4000.0/0xffff, points_format='numpy')

        vis.add('lidar',data['lidar'])
        vis.add('zed',data['zed'])

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
    def print_xforms():
        print("lidar:")
        print("rotation:",so3.ndarray(lidar_xform[0]))
        print("position:",lidar_xform[1])
        print("zed:")
        print("rotation:",so3.ndarray(zed_xform[0]))
        print("position:",zed_xform[1])

    zed_pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(data['zed_points']))
    lidar_pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(data['lidar_points']))

    zed_pcd.paint_uniform_color([1,0,0])
    lidar_pcd.paint_uniform_color([0,1,0])

    transformation = icp(zed_pcd, lidar_pcd, zed_xform_numpy)
    zed_pcd.transform(transformation)
    o3d.visualization.draw_geometries([zed_pcd, lidar_pcd])

    print('lidar to camera transform:', transformation)
    np.savetxt("/GEMstack/data/lidar_zed_calibration.txt", transformation)

    vis.addAction(increment_index,"Increment index",'=')
    vis.addAction(decrement_index,"Decrement index",'-')
    vis.addAction(print_xforms,'Print transforms','p')
    load_and_show_scan(1)
    vis.add('zed_xform',zed_xform)
    vis.add('lidar_xform',lidar_xform)
    vis.edit('zed_xform')
    vis.edit('lidar_xform')
    vis.show()
    while vis.shown():
        lidar_xform = vis.getItemConfig('lidar_xform')
        lidar_xform = lidar_xform[:9],lidar_xform[9:]
        zed_xform = vis.getItemConfig('zed_xform')
        zed_xform = zed_xform[:9],zed_xform[9:]
        data['lidar'].setCurrentTransform(*lidar_xform)
        data['zed'].setCurrentTransform(*zed_xform)
        time.sleep(0.02)
    vis.kill()

if __name__ == '__main__':
    import sys
    folder = 'data'
    if len(sys.argv) >= 2:
        folder = sys.argv[1]
    if len(sys.argv) >= 3:
        calib = sys.argv[2]
        import yaml
        with open(calib,'r') as f:
            config = yaml.load(f,yaml.SafeLoader)
            zed_K = np.array(config['K']).reshape((3,3))
            zed_intrinsics = [zed_K[0,0],zed_K[1,1],zed_K[0,2],zed_K[1,2]]
            zed_w = config['width']
            zed_height = config['height']
    main(folder)