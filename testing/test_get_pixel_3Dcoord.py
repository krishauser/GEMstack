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
parser.add_argument('--data_idx', '-i', type=int, default=3)

args = parser.parse_args()
OUTPUT_DIR = args.output_dir

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

def lidar_to_vehicle(point_cloud_lidar: np.ndarray, T_lidar2_Gem: np.ndarray):
    ones = np.ones((point_cloud_lidar.shape[0], 1))
    pcd_homogeneous = np.hstack((point_cloud_lidar, ones)) # (N, 4)
    pointcloud_trans = np.dot(T_lidar2_Gem, pcd_homogeneous.T) # (4, N)
    pointcloud_trans = pointcloud_trans.T # (N, 4)
    point_cloud_image_world = pointcloud_trans[:, :3] # (N, 3)
    return point_cloud_image_world


coord_3d_map = None
def mouse_callback(event, x, y, flags, param):
    global coord_3d_map
    if event == cv2.EVENT_MOUSEMOVE:
        # Display pixel coordinates on the console
        print(f"Pixel coordinates: x={x}, y={y}, 3D coord: {coord_3d_map[y][x]}")


extrinsic = np.loadtxt("GEMstack/knowledge/calibration/gem_e4_lidar2oak.txt")    
intrinsic = np.loadtxt("GEMstack/knowledge/calibration/gem_e4_intrinsic.txt")
intrinsic = np.concatenate([intrinsic, np.zeros((3, 1))], axis=1)

T_lidar2_Gem = np.loadtxt("GEMstack/knowledge/calibration/gem_e4_lidar2vehicle.txt")

# Hardcode the roi area for agents
xrange = (0, 10)
yrange = (-10, 10)

def get3DCoordByPixels(image, point_cloud):
    global coord_3d_map
    print ('\nStart testing...')
    filtered_point_cloud = filter_lidar_by_range(point_cloud, xrange,yrange)
    
    point_cloud_3D = lidar_to_vehicle(filtered_point_cloud, T_lidar2_Gem)
    
    point_cloud_image = lidar_to_image(filtered_point_cloud, 
                                        extrinsic, 
                                        intrinsic)
    
    # Keep only the lidar points whose projected coordinates lie  within the boundary of images
    height, width = image.shape[:2]
    idxs = np.where((point_cloud_image[:, 0] > 0) & (point_cloud_image[:, 0] < width) &
                    (point_cloud_image[:, 1] > 0) & (point_cloud_image[:, 1] < height) )[0]

    print ('# of point_cloud_image:', len(point_cloud_image)) # 131072
    print ('# of filtered_point_cloud:', len(idxs))
    
    # initialize coord_3d_map
    coord_3d_map = np.zeros(shape=(height, width, 3), dtype=point_cloud_3D.dtype)
    for idx in idxs:
        u, v = int(point_cloud_image[idx][0]), int(point_cloud_image[idx][1])
        
        coord_3d_map[v][u][:] = point_cloud_3D[idx]
    
    return coord_3d_map

if __name__=='__main__':
    # load data
    lidar_fn = os.path.join(args.src_dir, f'lidar{args.data_idx}.npz')
    image_fn = os.path.join(args.src_dir, f'color{args.data_idx}.png')
    # depth_fn = os.path.join(args.src_dir, f'depth{args.data_idx}.tif')
    
    point_cloud = np.load(lidar_fn)['arr_0']
    image = cv2.imread(image_fn)
    coord_3d_map = get3DCoordByPixels(image, point_cloud)
    
    # check pixel 3D coords
    cv2.namedWindow('Image')
    cv2.setMouseCallback('Image', mouse_callback)
    while True:
        cv2.imshow('Image', image)
        
        # Break the loop if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    cv2.destroyAllWindows()
    print ('\nDone!')
    