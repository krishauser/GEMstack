"""To run the calibration steps and visualize the results
References:
- https://www.open3d.org/html/tutorial/Advanced/interactive_visualization.html
- https://towardsdatascience.com/what-are-intrinsic-and-extrinsic-camera-parameters-in-computer-vision-7071b72fb8ec
"""

#needed to import GEMstack from top level directory
import sys
import os
sys.path.append(os.getcwd())

from GEMstack.utils import settings
from GEMstack.offboard.calibration.lidar_zed import select_points_from_pcd

import open3d as o3d
import numpy as np
import yaml
import cv2

calib1_folder = settings.get('calibration.data1')
zed_K = None
T_lidar_zed = None

def read_zed_intrinsics():
    global zed_K
    with open(settings.get('calibration.zed_intrinsics'), 'r') as file:
        config = yaml.load(file, yaml.SafeLoader)
    zed_K = np.array(config['K']).reshape(3,3)

def read_lidar_zed_transform():
    global T_lidar_zed
    with open(settings.get('calibration.transforms'), 'r') as file:
        config = yaml.load(file, yaml.SafeLoader)
    T_lidar_zed = np.array(config['lidar2zed']).reshape(4,4)

def visualize_lidar_zed_transform(idx):
    lidar_data_path = os.path.join(calib1_folder, 'lidar{}.npz').format(idx)
    lidar_data = np.load(lidar_data_path)['arr_0']

    # select points of interest
    lidar_pts = select_points_from_pcd(lidar_data)

    # convert lidar points to 4 by |pts| format
    lidar_pts4 = np.ones((4,len(lidar_pts)))
    lidar_pts4[:3,:] = lidar_pts.transpose()

    zed_K4 = np.zeros((3,4))
    zed_K4[:3,:3] = zed_K
  
    # project lidar points --> zed coordinates --> image pixels
    proj_img_pts = np.matmul(zed_K4, np.matmul(T_lidar_zed, lidar_pts4))

    img_path = os.path.join(calib1_folder, 'color{}.png').format(idx)
    img = cv2.imread(img_path)

    for i in range(len(proj_img_pts[0])):
        u = int(proj_img_pts[0,i] / proj_img_pts[2,i])
        v = int(proj_img_pts[1,i] / proj_img_pts[2,i])
        img[v,u] = [0,255,255]

    cv2.imshow('proj_img_pts', img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

def run_zed_intrinsics():
    import GEMstack.offboard.calibration.zed_intrinsics as zed_intrinsics
    zed_intrinsics.main()

def run_lidar_zed_transform():
    import GEMstack.offboard.calibration.lidar_zed_transform as lidar_zed_transform
    lidar_zed_transform.main()

    # visualization
    read_zed_intrinsics()
    read_lidar_zed_transform()
    visualize_lidar_zed_transform(9)

def run_lidar_vehicle_transform():
    import GEMstack.offboard.calibration.lidar_vehicle_transform as lidar_vehicle_transform
    lidar_vehicle_transform.main()

def run_zed_vehicle_transform():
    import GEMstack.offboard.calibration.zed_vehicle_transform as zed_vehicle_transform
    zed_vehicle_transform.main()

if __name__ == '__main__':
    print('Options:')
    print('1. Obtain ZED2 intrinsics')
    print('2. Compute Velodyne to ZED2 transform')
    print('3. Compute Velodyne to vehicle transform')
    print('4. Compute ZED2 to vehicle transform')
    choice = input('Enter choice - ')
    print()

    if choice == '1':
        run_zed_intrinsics()
    elif choice == '2':
        run_lidar_zed_transform()
    elif choice == '3':
        run_lidar_vehicle_transform()
    elif choice == '4':
        run_zed_vehicle_transform()