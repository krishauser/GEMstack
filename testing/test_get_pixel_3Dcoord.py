#needed to import GEMstack from top level directory
import sys
import os
import cv2
import argparse
import random
import numpy as np
from typing import Dict,Tuple, List
import pathlib

parser = argparse.ArgumentParser()

parser.add_argument('--output_dir', '-o', type=str, default='save')
parser.add_argument('--src_dir', '-s', type=str, default='./data/gt')
parser.add_argument('--data_idx', '-i', type=int, default=1)

args = parser.parse_args()

OUTPUT_DIR = args.output_dir

from sklearn.cluster import DBSCAN

colors = []
colors += [(i, 0, 255) for i in range(100, 256, 25)]
colors += [(i, 255, 0) for i in range(100, 256, 25)]
colors += [(0, i, 255) for i in range(100, 256, 25)]
colors += [(255, i, 0) for i in range(100, 256, 25)]
colors += [(255, 0, i) for i in range(100, 256, 25)]
colors += [(0, 255, i) for i in range(100, 256, 25)]
seed = 19
random.seed(seed)
random.shuffle(colors)

def filter_lidar_by_range(point_cloud, xrange: Tuple[float, float], yrange: Tuple[float, float]):
    xmin, xmax = xrange
    ymin, ymax = yrange
    idxs = np.where((point_cloud[:, 0] > xmin) & (point_cloud[:, 0] < xmax) &
                    (point_cloud[:, 1] > ymin) & (point_cloud[:, 1] < ymax) )
    return point_cloud[idxs]

def lidar_to_image(point_cloud_lidar: np.ndarray, extrinsic : np.ndarray, intrinsic : np.ndarray):
    
    homo_point_cloud_lidar = np.hstack((point_cloud_lidar, np.ones((point_cloud_lidar.shape[0], 1)))) # (N, 4)
    pointcloud_pixel = (intrinsic @ extrinsic @ (homo_point_cloud_lidar).T) # (3, N)
    pointcloud_pixel = pointcloud_pixel.T # (N, 3)

    # normalize
    pointcloud_pixel[:, 0] /= pointcloud_pixel[:, 2] # normalize
    pointcloud_pixel[:, 1] /= pointcloud_pixel[:, 2] # normalize
    point_cloud_image =  pointcloud_pixel[:,:2] # (N, 2)
    return point_cloud_image

class TestHelper:
    def __init__(self, point_cloud, zed_image, depth):
        self.point_cloud = point_cloud
        self.zed_image = zed_image
        self.depth = depth
        
        self.extrinsic = np.loadtxt("GEMstack/knowledge/calibration/gem_e4_lidar2oak.txt")    
        intrinsic = np.loadtxt("GEMstack/knowledge/calibration/gem_e4_intrinsic.txt")
        self.intrinsic = np.concatenate([intrinsic, np.zeros((3, 1))], axis=1)

        T_lidar2_Gem = np.loadtxt("GEMstack/knowledge/calibration/gem_e4_lidar2vehicle.txt")
        self.T_lidar2_Gem = np.asarray(T_lidar2_Gem)

        # Hardcode the roi area for agents
        self.xrange = (0, 100)
        self.yrange = (-100, 100)

    def run(self):
        print ('\nStart testing...')
        filtered_point_cloud = filter_lidar_by_range(self.point_cloud, 
                                                     self.xrange,
                                                     self.yrange)
        
        point_cloud_image = lidar_to_image(filtered_point_cloud, 
                                           self.extrinsic, 
                                           self.intrinsic)
        vis = self.zed_image.copy()
        for proj_pt in point_cloud_image:
            color = (0, 255, 0) # green
            radius = 1
            center = int(proj_pt[0]), int(proj_pt[1])
            vis = cv2.circle(vis, center, radius, color, cv2.FILLED)
        
        pathlib.Path(OUTPUT_DIR).mkdir(parents=True, exist_ok=True)
        output_path = os.path.join(OUTPUT_DIR, f'lidar_to_image_{args.data_idx}.png')
        print ('Output lidar_to_image result:', output_path)
        cv2.imwrite(output_path, vis)


if __name__=='__main__':
    # load data
    lidar_fn = os.path.join(args.src_dir, f'lidar{args.data_idx}.npz')
    image_fn = os.path.join(args.src_dir, f'color{args.data_idx}.png')
    depth_fn = os.path.join(args.src_dir, f'depth{args.data_idx}.tif')
    
    point_cloud = np.load(lidar_fn)['arr_0']
    image = cv2.imread(image_fn)
    depth = cv2.imread(depth_fn)
    
    test_helper = TestHelper(point_cloud, image, depth)
    
    test_helper.run()
    print ('\nDone!')
    