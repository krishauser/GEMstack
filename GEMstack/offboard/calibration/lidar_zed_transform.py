"""To compute the relative transform that converts points from the velodyne to the zed frame

References:
- https://github.com/krishauser/GEMstack/blob/s2024/GEMstack/offboard/calibration/klampt_lidar_zed_show.py
- Klampt Python APIs: https://motion.cs.illinois.edu/software/klampt/latest/pyklampt_docs/index.html
- Open3d - Working with NumPy: https://www.open3d.org/docs/0.9.0/tutorial/Basic/working_with_numpy.html
- Point-to-point ICP: https://www.open3d.org/docs/release/tutorial/pipelines/icp_registration.html
"""

from ...utils import settings
from .lidar_zed import Lidar, Zed

import open3d as o3d

import numpy as np
import yaml

def register_points(source, target, T):
    d_max = 0.05
    T_init = np.identity(4)
    T_init[:3,:3] = np.array(T[0]).reshape((3,3)).transpose()
    T_init[:3,3] = [0.2,-0.2,0.2]
    
    reg = o3d.pipelines.registration.registration_icp(source, target, d_max, T_init, \
                                                      o3d.pipelines.registration.TransformationEstimationPointToPoint())
    print(reg)
    
    return reg.transformation

def run(folder, idx):
    zed = Zed()
    zed.print()
    zed_pcd = zed.get_point_cloud(folder, idx)
    print(zed_pcd)

    lidar = Lidar()
    lidar.print()
    lidar_pcd = lidar.get_point_cloud(folder, idx)
    print(lidar_pcd)

    T_lidar_zed = register_points(lidar_pcd, zed_pcd, zed.T)
    print('T_velodyne_zed:\n', T_lidar_zed)

    d = {
        'lidar2zed': T_lidar_zed.flatten().tolist()
    }
    with open(settings.get('calibration.transforms'), 'a') as yaml_file:
        yaml.dump(d, yaml_file)

def main():
    calib_data_folder = settings.get('calibration.data1')
    run(calib_data_folder, 9)
