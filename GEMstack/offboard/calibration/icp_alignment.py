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

#uncalibrated values -- TODO: load these from a calibration file
# intrinsic = [684.8333129882812, 0.0, 573.37109375, 0.0, 684.6096801757812, 363.700927734375, 0.0, 0.0, 1.0]
# intrinsic = [527.5779418945312, 0.0, 616.2459716796875, 0.0, 527.5779418945312, 359.2155456542969, 0.0, 0.0, 1.0]
# zed_K = np.array(intrinsic).reshape((3, 3))

zed_K = np.array([[684.8333129882812, 0.0, 573.37109375],
                [0.0, 684.6096801757812, 363.700927734375],
                [0.0, 0.0, 1.0]])
zed_intrinsics = [zed_K[0,0],zed_K[1,1],zed_K[0,2],zed_K[1,2]]
zed_w = 1152
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
    zed_xform_numpy = np.array([[0,0,1],[-1,0,0],[0,-1,0]])
    zed_xform_numpy4 = np.array([[0,0,1, 0],[-1,0,0, 0],[0,-1,0, 0], [0, 0, 0, 1]])
    zed_xform = (so3.from_ndarray(zed_xform_numpy),[0,0,0])
    lidar_pattern = os.path.join(folder,"lidar{}.npz")
    color_pattern = os.path.join(folder,"color{}.png")
    depth_pattern = os.path.join(folder,"depth{}.tif")
    data = {}
    def load_and_show_scan(idx):
        arr_compressed = np.load(lidar_pattern.format(idx))
        arr = arr_compressed['arr_0']
        arr_compressed.close()

        data['lidar_points'] = arr

        #pc = numpy_convert.from_numpy(arr,'PointCloud')
        #pc = colorize(pc,'z','plasma')
        #data['lidar_points'] = pc.getPoints()

        color = cv2.imread(color_pattern.format(idx))
        depth = cv2.imread(depth_pattern.format(idx),cv2.IMREAD_UNCHANGED)
        
        depth = depth.astype(np.float32)
        print("depth range",np.min(depth),np.max(depth))
        zed_xfov = 2*np.arctan(zed_w/(2*zed_intrinsics[0]))
        zed_yfov = 2*np.arctan(zed_h/(2*zed_intrinsics[1]))
        print("estimated zed horizontal FOV",math.degrees(zed_xfov),"deg")
        #pc = image_to_points(depth,color,zed_xfov,zed_yfov,depth_scale=4000.0/0xffff, points_format='PointCloud')
        data['zed_points'] = image_to_points(depth,None,zed_xfov,zed_yfov,depth_scale=4000.0/0xffff, points_format='numpy')

        #data['zed'] = Geometry3D(pc)
        #data['lidar'].setCurrentTransform(*lidar_xform)
        #data['zed'].setCurrentTransform(*zed_xform)
        #vis.add('lidar',data['lidar'])
        #vis.add('zed',data['zed'])
        #pc.transform(*zed_xform)
        #data['zed_points'] = pc.getPoints()


        #MAYBE ADD
        #data['zed_points'] = (zed_xform_numpy @ data['zed_points'].T).T

    data['index'] = 1
    def increment_index():
        data['index'] = 1
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

    increment_index()

    print ('data.keys():', list(data.keys()))
    zed_pc = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(data['zed_points']))
    lidar_pc = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(data['lidar_points']))

    zed_pc.paint_uniform_color([1,0,0])
    lidar_pc.paint_uniform_color([0,1,0])

    #zed_pc.transform(zed_xform_numpy4)
    #o3d.visualization.draw_geometries([zed_pc, lidar_pc])

    transform = open3d_icp(zed_pc, lidar_pc, zed_xform_numpy4)
    zed_pc.transform(transform)
    # o3d.visualization.draw_geometries([zed_pc, lidar_pc])

    np.savetxt("GEMstack/knowledge/calibration/zed2lidar.txt", transform)

    print('done')



if __name__ == '__main__':
    import sys
    folder = 'data/step1_rgb_image_stereo_depth/'
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


