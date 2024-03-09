# To compute the relative transform that converts points from the zed to the vehicle frame

from ...utils import settings

import numpy as np
import yaml

def get_lidar_zed_transform(file):
    with open(file, 'r') as f:
        config = yaml.load(f, yaml.SafeLoader)
        return np.array(config['lidar2zed']).reshape((4,4))   

def get_lidar_vehicle_transform(file):
    T = np.identity(4)

    with open(file, 'r') as f:
        config = yaml.load(f, yaml.SafeLoader)
        T[:3,:3] = np.array(config['lidar2vehicle']['R']).reshape((3,3))
        T[:3,3] = np.array(config['lidar2vehicle']['t'])
    
    return T

def run(file):
    T_lidar_zed = get_lidar_zed_transform(file)
    T_lidar_vehicle = get_lidar_vehicle_transform(file)

    T_zed_vehicle = np.matmul(T_lidar_vehicle, np.linalg.inv(T_lidar_zed))
    print(T_zed_vehicle)

    d = {
        'zed2vehicle': {
            'R': T_zed_vehicle[:3,:3].flatten().tolist(),
            't': T_zed_vehicle[:3,3].tolist()
        }
    }
    with open(file, 'a') as f:
        yaml.dump(d, f)

def main():
    transforms_file = settings.get('calibration.transforms')
    run(transforms_file)