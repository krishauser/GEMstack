from ...utils import settings

from klampt.math import se3
from klampt.io import numpy_convert
from klampt.model.sensing import image_to_points

import open3d as o3d

import cv2
import os
import numpy as np
import math
import yaml

class Zed:
    def __init__(self):
        with open(settings.get('calibration.zed_intrinsics'), 'r') as file:
            config = yaml.load(file, yaml.SafeLoader)
            self.K = np.array(config['K']).reshape(3,3)
            self.fx, self.fy = self.K[0,0], self.K[1,1]
            self.cx, self.cy = self.K[0,2], self.K[1,2]
            self.h = config['height']
            self.w = config['width']

        self.xfov = 2 * np.arctan(self.w/ (2 * self.fx))
        self.yfov = 2 * np.arctan(self.h/ (2 * self.fy))

        flatten = lambda l : np.array(l).flatten().tolist()
        R = flatten([[0,0,1], [-1,0,0], [0,-1,0]]) # from gem_e2_zed.yaml
        self.T = (R, [0,0,0])

    def print(self):
        print('--- Zed params ---')
        print('f:', self.fx, self.fy)
        print('c:', self.cx, self.cy)
        print('height:', self.h)
        print('width:', self.w)
        print('horizontal FOV:', math.degrees(self.xfov), "deg")
        print('vertical FOV:', math.degrees(self.yfov), "deg")
        print('T:', self.T)

    def get_point_cloud(self, folder, idx):
        depth_data_path = os.path.join(folder, 'depth{}.tif').format(idx)
        depth_data = cv2.imread(depth_data_path, cv2.IMREAD_UNCHANGED).astype(np.float32)

        # img_path = os.path.join(folder, 'color{}.png').format(idx)
        # img = cv2.imread(img_path)
        
        # for klampt.robotsim.PointCloud
        # pcd = image_to_points(depth_data, img, self.xfov, self.yfov, \
        #                       depth_scale=4000.0/0xffff, points_format='PointCloud')
        # return pcd

        # for open3d.geometry.PointCloud
        points = image_to_points(depth_data, None, self.xfov, self.yfov, depth_scale=4000.0/0xffff)
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        return pcd

class Lidar:
    def __init__(self):
        self.T = se3.identity()

    def print(self):
        print('--- Lidar params ---')
        print('T:', self.T)

    def get_point_cloud(self, folder, idx):
        lidar_data_path = os.path.join(folder, 'lidar{}.npz').format(idx)
        lidar_data = np.load(lidar_data_path)['arr_0']
        
        # for klampt.robotsim.PointCloud
        # pcd = numpy_convert.from_numpy(lidar_data, 'PointCloud')
        # return pcd

        # for open3d.geometry.PointCloud
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(lidar_data)
        return pcd