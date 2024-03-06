# examples/Python/Basic/icp_registration.py

import open3d as o3d
import numpy as np
import copy

import imageio.v3 as iio
from PIL import Image
import cv2

def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp.to_legacy(), target_temp.to_legacy()])

from sys import exit
if __name__ == "__main__":
    # TODO put an object on the center line of the vehicle and load the image and lidar cloud
    source = np.load(r"C:\Users\Li\Desktop\HW3\pc\pc\zed2.npy")[::64]
    target = np.load(r"C:\Users\Li\Desktop\HW3\pc\pc\lidar2.npy")
    
    # mannually select the points that correspond to the object
    target = np.array([p for p in target if p[0]>0.1 and p[2]<1 and p[0]<10 and p[2]>-1 and p[1]>-1 and p[1]<1])
    print(target.mean(axis=0)) #[ 8.07718906  0.15936596 -0.42052382]
    
    
    source = o3d.t.geometry.PointCloud(source)
    target = o3d.t.geometry.PointCloud(target)
    
    # outliner removal
    cl, ind = target.remove_radius_outliers(nb_points=8,
                                                         search_radius=0.5)
    target = target.select_by_mask(ind)
    
    plane_model, inliers = target.segment_plane(distance_threshold=0.1,
                                                ransac_n=3,
                                                num_iterations=10,
                                                probability=0.99)
    print(plane_model)
    # from the plane model, we can derive the yaw
    trans_init = np.identity(4)
    draw_registration_result(source, target, trans_init)