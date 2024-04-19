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

parser.add_argument('--src_dir', '-s', type=str, default='./data/gt')
parser.add_argument('--test_target', '-t', type=str, default='interpolation', 
                    choices=['interpolation', 'test_conv'])
parser.add_argument('--data_idx', '-i', type=int, default=3)
parser.add_argument('--kernel_size', '-k', type=int, default=5)

args = parser.parse_args()

def test_conv_2D():
    # Define a sample image and kernel
    image = np.array([
        [10, 0, 0],
        [0, 20, 0],
        [0, 0, 30]
    ], dtype=np.float32)

    # Do convolution
    kernel = np.array([
                 [1, 1],
                 [1, 1]
             ], dtype=np.float32)
    kernel = cv2.flip(kernel, flipCode=-1)
    result = cv2.filter2D(image, -1, kernel, borderType=cv2.BORDER_CONSTANT)

    # Derive num_elements matrix
    has_value = image > 0
    has_value = has_value.astype(np.uint8)
    num_elements_mat = cv2.filter2D(has_value, -1, kernel, borderType=cv2.BORDER_CONSTANT)
    
    # divide by num of elements correspond to that particular pixel
    num_elements_mat[num_elements_mat == 0] = 1 # avoid divide by zero
    interpolation_result = result / num_elements_mat

    print("Image:")
    print(image)
    print("\nKernel:")
    print(kernel)
    print("\nResult:")
    print(result)
    print("\nnum_elements_mat:")
    print(num_elements_mat)
    print("\n Interpolation Result:")
    print(interpolation_result)

def interpolate(image):
    def interpolate_single_channel(image, kernel):
        kernel = cv2.flip(kernel, flipCode=-1)
        result = cv2.filter2D(image, -1, kernel, borderType=cv2.BORDER_CONSTANT)

        # Derive num_elements matrix
        has_value = image > 0
        has_value = has_value.astype(np.uint8)
        num_elements_mat = cv2.filter2D(has_value, -1, kernel, borderType=cv2.BORDER_CONSTANT)
    
        # divide by num of elements correspond to that particular pixel
        num_elements_mat[num_elements_mat == 0] = 1 # avoid divide by zero
        final_result = result / num_elements_mat # divide element-wise
        return final_result
    
    # dilate and interpolation
    channels = []
    num_channels = coord_3d_map.shape[-1]
    for i in range(num_channels):
        channel = coord_3d_map[:, :, i]
        kernel = np.ones((args.kernel_size, args.kernel_size), dtype=np.float32)
        result = interpolate_single_channel(channel, kernel)
        channels.append(result)
        print ('result.shape:', result.shape)

    interpolate_3D_map = cv2.merge(channels)
    return interpolate_3D_map
        
        
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
        x_3d, y_3d, z_3d = coord_3d_map[y][x]
        print(f"Pixel coordinates: x={x}, y={y} | 3D xyz coord: x={x_3d:.2f}, y={y_3d:.2f}, z={z_3d:.2f} (in meters)")


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
    
    # interpolate
    interpolate_3D_map = interpolate(coord_3d_map)
    print ('interpolate_3D_map shape:', interpolate_3D_map.shape)
    
    return interpolate_3D_map

if __name__=='__main__':
    # load data
    lidar_fn = os.path.join(args.src_dir, f'lidar{args.data_idx}.npz')
    image_fn = os.path.join(args.src_dir, f'color{args.data_idx}.png')
    
    point_cloud = np.load(lidar_fn)['arr_0']
    image = cv2.imread(image_fn)
    
    if args.test_target == 'test_conv':
        ### verify the correctness of opencv filter2D ###
        test_conv_2D()
        
    elif args.test_target == 'interpolation':
        coord_3d_map = get3DCoordByPixels(image, point_cloud)
        
        # check pixel 3D coords in interactive mode
        cv2.namedWindow('Image')
        cv2.setMouseCallback('Image', mouse_callback)
        while True:
            cv2.imshow('Image', image)
            
            # Break the loop if 'q' is pressed
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        
        cv2.destroyAllWindows()
        
    print ('\nDone!')
        