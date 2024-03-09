"""To compute the relative transform that converts points from the velodyne to the vehicle frame

References:
- Plane segmentation: https://www.open3d.org/docs/latest/tutorial/Basic/pointcloud.html#Plane-segmentation
- Roll, pitch, yaw: https://msl.cs.uiuc.edu/planning/node102.html
- GEM e2 sensors: https://publish.illinois.edu/robotics-autonomy-resources/gem-e2/hardware/ 
"""

from ...utils import settings
from .lidar_zed import get_lidar_data, select_points_from_pcd

import open3d as o3d

import numpy as np
import yaml

unit_vec = lambda v : v / np.linalg.norm(v)

def compute_best_fitting_plane(pts):
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pts)
    plane, _ = pcd.segment_plane(0.05, 3, 1000) # args: threshold, initial inliers, iterations
    print('Best fit plane:', plane)
    return plane

def compute_rotation_roll(lidar_pts):
    # compute best fit plane and normal
    best_fit_plane = compute_best_fitting_plane(lidar_pts)
    unit_normal = unit_vec(best_fit_plane[:3]) # along vehicle Y axis
    
    gamma = np.arccos(np.dot(unit_normal, [0,1,0])) # angle between Y axes of vehicle and lidar
    R = [[1, 0, 0],
         [0, np.cos(gamma), -np.sin(gamma)],
         [0, np.sin(gamma), np.cos(gamma)]]
    return R

def compute_rotation_pitch(lidar_pts):
    best_fit_plane = compute_best_fitting_plane(lidar_pts)
    unit_normal = unit_vec(best_fit_plane[:3]) # along vehicle X axis
    
    beta = np.arccos(np.dot(unit_normal, [1,0,0])) # angle between X axes of vehicle and lidar
    R = [[np.cos(beta), 0, np.sin(beta)],
         [0, 1, 0],
         [-np.sin(beta), 0, np.cos(beta)]]
    return R

def compute_rotation_yaw(lidar_pts):
    pts_z0 = [p for p in lidar_pts if abs(p[2]) <= 0.04]
    avg = [sum(p) / len(p) for p in zip(*pts_z0)]
    pt = min(pts_z0, key=lambda p: np.linalg.norm(p - avg)) 

    alpha = -np.arctan(pt[1] / pt[0])
    R = [[np.cos(alpha), -np.sin(alpha), 0], 
         [np.sin(alpha), np.cos(alpha), 0], 
         [0, 0, 1]]
    return R

def compute_rotation(folder):
    print('--- for measuring roll ---')
    lidar_pts = select_points_from_pcd(get_lidar_data(folder, 5))   
    R_roll = compute_rotation_roll(lidar_pts)

    print('--- for measuring pitch ---')
    lidar_pts = select_points_from_pcd(get_lidar_data(folder, 6)) 
    R_pitch = compute_rotation_pitch(lidar_pts)

    print('--- for measuring yaw ---')
    lidar_pts = select_points_from_pcd(get_lidar_data(folder, 8)) 
    R_yaw = compute_rotation_yaw(lidar_pts)
    
    print(np.array(R_roll))
    print(np.array(R_pitch))
    print(np.array(R_yaw))

    return np.matmul(R_yaw, np.matmul(R_pitch, R_roll))

def compute_translation():
    # translation in metres
    d_ground_to_lidar = 1.985
    d_ground_to_rear_axle = 0.28
    d_rear_axle_to_lidar = 0.87 # measured along vehicle X axis

    t_x = d_rear_axle_to_lidar
    t_y = 0 # lidar is on the vehicle's y = 0 plane
    t_z = d_ground_to_lidar - d_ground_to_rear_axle

    t = [t_x, t_y, t_z]
    return t

def run(folder):
    R = compute_rotation(folder)
    print('R:\n', R)

    t = compute_translation()
    print('t:\n', t)

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
    run(calib_data_folder)
