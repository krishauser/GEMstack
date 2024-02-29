"""To compute the relative transform that converts points from the velodyne to the vehicle frame

References:
- Plane segmentation: https://www.open3d.org/docs/latest/tutorial/Basic/pointcloud.html#Plane-segmentation
- GEM e2 sensors: https://publish.illinois.edu/robotics-autonomy-resources/gem-e2/hardware/ 
"""

from ...utils import settings
from .lidar_zed import Lidar, select_points_from_pcd

import open3d as o3d

import numpy as np
import yaml

def compute_best_fitting_plane(pts):
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pts)
    plane, _ = pcd.segment_plane(0.05, 3, 1000) # args: threshold, initial inliers, iterations
    print('Best fit plane:', plane)
    return plane

def compute_rotation(lidar_pts):
    # compute best fit plane and normal
    best_fit_plane = compute_best_fitting_plane(lidar_pts)

    unit_vec = lambda v : v / np.linalg.norm(v)
    unit_normal = unit_vec(best_fit_plane[:3])

    # compute rotation matrix
    theta = np.arccos(np.dot(unit_normal, [1,0,0]))
    R = [[np.cos(theta), -np.sin(theta), 0], 
         [np.sin(theta), np.cos(theta), 0], 
         [0,0,1]]
    
    return R

def compute_translation():
    # TODO: measure and record translation in metres
    d_ground_to_lidar = 0
    d_ground_to_rear_axle = 0
    d_rear_axle_to_lidar = 0 # along vehicle X axis

    tx = d_rear_axle_to_lidar
    ty = 0 # lidar is on the vehicle's y = 0 plane
    tz = d_ground_to_lidar - d_ground_to_rear_axle

    t = [tx, ty, tz]
    return t

def run(folder, idx):
    lidar = Lidar()
    lidar.print()
    lidar_data = lidar.get_lidar_data(folder, idx)

    # select points of interest
    lidar_pts = select_points_from_pcd(lidar_data)    
    
    R = compute_rotation(lidar_pts)
    print('R:\n', R)

    t = compute_translation()
    print('t:\n', t)

    # TODO: write R and t to GEMstack/GEMstack/knowledge/calibration/gem_e2_velodyne.yaml

    d = {
        'lidar2vehicle': {
            'R': np.array(R).flatten().tolist(),
            't': t
        }
    }
    with open(settings.get('calibration.transforms'), 'a') as yaml_file:
        yaml.dump(d, yaml_file)

def main():
    calib_data_folder = settings.get('calibration.data2')
    run(calib_data_folder, 1)
