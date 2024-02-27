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
zed_K = np.array([[527.5779418945312, 0.0, 616.2459716796875],
                [0.0, 527.5779418945312, 359.2155456542969],
                [0.0, 0.0, 1.0]])
zed_intrinsics = [zed_K[0,0],zed_K[1,1],zed_K[0,2],zed_K[1,2]]
zed_w = 1280
zed_h = 720


# def draw_registration_result(source, target, transformation):
#     source_temp = np.copy.deepcopy(source)
#     target_temp = np.copy.deepcopy(target)
#     source_temp.paint_uniform_color([1, 0.706, 0])
#     target_temp.paint_uniform_color([0, 0.651, 0.929])
#     source_temp.transform(transformation)
#     o3d.visualization.draw_geometries([source_temp, target_temp],
#                                       zoom=0.4459,
#                                       front=[0.9288, -0.2951, -0.2242],
#                                       lookat=[1.6784, 2.0612, 1.4451],
#                                       up=[-0.3402, -0.9189, -0.1996])
    

def draw_registration_result(source, target, transformation):
    source_temp = o3d.geometry.PointCloud(source)
    target_temp = o3d.geometry.PointCloud(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target_temp],
                                      zoom=0.4459,
                                      front=[0.9288, -0.2951, -0.2242],
                                      lookat=[1.6784, 2.0612, 1.4451],
                                      up=[-0.3402, -0.9189, -0.1996])
    
def preprocess_point_cloud(pcd, voxel_size):
    print(":: Downsample with a voxel size %.3f." % voxel_size)
    pcd_down = pcd.voxel_down_sample(voxel_size)

    radius_normal = voxel_size * 2
    print(":: Estimate normal with search radius %.3f." % radius_normal)
    pcd_down.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))

    radius_feature = voxel_size * 5
    print(":: Compute FPFH feature with search radius %.3f." % radius_feature)
    pcd_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        pcd_down,
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))
    return pcd_down, pcd_fpfh

# def prepare_dataset(voxel_size):
#     print(":: Load two point clouds and disturb initial pose.")

#     demo_icp_pcds = o3d.data.DemoICPPointClouds()
#     source = o3d.io.read_point_cloud(demo_icp_pcds.paths[0])
#     target = o3d.io.read_point_cloud(demo_icp_pcds.paths[1])
#     trans_init = np.asarray([[0.0, 0.0, 1.0, 0.0], [1.0, 0.0, 0.0, 0.0],
#                              [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]])
#     source.transform(trans_init)
#     draw_registration_result(source, target, np.identity(4))

#     source_down, source_fpfh = preprocess_point_cloud(source, voxel_size)
#     target_down, target_fpfh = preprocess_point_cloud(target, voxel_size)
#     return source, target, source_down, target_down, source_fpfh, target_fpfh

# voxel_size = 0.05  # means 5cm for this dataset
# source, target, source_down, target_down, source_fpfh, target_fpfh = prepare_dataset(
#     voxel_size)

def execute_global_registration(source_down, target_down, source_fpfh,
                                target_fpfh, voxel_size):
    distance_threshold = voxel_size * 1.5
    print(":: RANSAC registration on downsampled point clouds.")
    print("   Since the downsampling voxel size is %.3f," % voxel_size)
    print("   we use a liberal distance threshold %.3f." % distance_threshold)
    result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
        source_down, target_down, source_fpfh, target_fpfh, True,
        distance_threshold,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
        3, [
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(
                0.9),
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(
                distance_threshold)
        ], o3d.pipelines.registration.RANSACConvergenceCriteria(100000, 0.999))
    return result







def main(folder):
    lidar_xform = se3.identity()


    print("lidar_xform =", lidar_xform)
    zed_xform = (so3.from_ndarray(np.array([[0,0,1],[-1,0,0],[0,-1,0]])),[0,0,0])
    lidar_pattern = os.path.join(folder,"lidar{}.npz")
    color_pattern = os.path.join(folder,"color{}.png")
    depth_pattern = os.path.join(folder,"depth{}.tif")
    data = {}
    def load_and_show_scan(idx):
        arr_compressed = np.load(lidar_pattern.format(idx))
        arr = arr_compressed['arr_0']
        arr_compressed.close()
        pc_lidar = numpy_convert.from_numpy(arr,'PointCloud')
        pc_lidar = colorize(pc_lidar,'z','plasma')
        data['lidar'] = Geometry3D(pc_lidar)

        color = cv2.imread(color_pattern.format(idx))
        depth = cv2.imread(depth_pattern.format(idx),cv2.IMREAD_UNCHANGED)
        depth = depth.astype(np.float32)
        print("depth range",np.min(depth),np.max(depth))
        zed_xfov = 2*np.arctan(zed_w/(2*zed_intrinsics[0]))
        zed_yfov = 2*np.arctan(zed_h/(2*zed_intrinsics[1]))
        print("estimated zed horizontal FOV",math.degrees(zed_xfov),"deg")
        
        pc_zed_data = image_to_points(depth, None, zed_xfov,zed_yfov,depth_scale=4000.0/0xffff, points_format='numpy')

        #test
        pc_vel_data = arr
        pc_vel = o3d.geometry.PointCloud()
        pc_vel.points = o3d.utility.Vector3dVector(pc_vel_data)
        o3d.io.write_point_cloud("vel.pcd", pc_vel)
        pc_zed = o3d.geometry.PointCloud()
        pc_zed.points = o3d.utility.Vector3dVector(pc_zed_data)
        o3d.io.write_point_cloud("zed.pcd", pc_zed)
       
        
        voxel_size = 0.05  # means 5cm for this dataset
        source = pc_vel
        target = pc_zed
        trans_init_vel = np.asarray([[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0],
                                [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]])
        source.transform(trans_init_vel)
        trans_init_zed = np.asarray([[0.0, 0.0, 1.0, 0.0], [-1.0, 0.0, 0.0, 0.0],
                                [0.0, -1.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]])
        target.transform(trans_init_zed)
        draw_registration_result(source, target, np.identity(4))

        source_down, source_fpfh = preprocess_point_cloud(source, voxel_size)
        target_down, target_fpfh = preprocess_point_cloud(target, voxel_size)

        result_ransac = execute_global_registration(source_down, target_down,
                                            source_fpfh, target_fpfh,
                                            voxel_size)
        print(result_ransac.transformation)
        draw_registration_result(source_down, target_down, result_ransac.transformation)


        def refine_registration(source, target, source_fpfh, target_fpfh, voxel_size):
            distance_threshold = voxel_size * 0.4
            print(":: Point-to-point ICP registration is applied on original point")
            print("   clouds to refine the alignment. This time we use a strict")
            print("   distance threshold %.3f." % distance_threshold)
            result = o3d.pipelines.registration.registration_icp(
                source, target, distance_threshold, result_ransac.transformation,
                o3d.pipelines.registration.TransformationEstimationPointToPoint())
            return result

        result_icp = refine_registration(source, target, source_fpfh, target_fpfh,
                                 voxel_size)
        print(result_icp.transformation)
        draw_registration_result(source, target, result_icp.transformation)
        

        
        


        pc_stereo = image_to_points(depth,color,zed_xfov,zed_yfov,None, points_format='PointCloud')
        data['zed'] = Geometry3D(pc_stereo)
        data['lidar'].setCurrentTransform(*lidar_xform)
        data['zed'].setCurrentTransform(*zed_xform)
        vis.add('lidar',data['lidar'])
        vis.add('zed',data['zed'])

    
    load_and_show_scan(1)
   

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






