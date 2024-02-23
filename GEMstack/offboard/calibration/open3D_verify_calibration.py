""" 
    This script is for verifying extrinsic matrix we found.
"""
import numpy as np
import open3d as o3d
import cv2

def imshow(window_name, image):
    cv2.imshow(window_name, image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

lidar_data = np.load('../../../hw3_data/lidar4.npz')
points = lidar_data['arr_0']  # Assuming 'points' is the key for your point cloud data
print (points.shape)

print (points)

# stopsign_idxs can be obtained using "open3D_select_object_by_box.py"
stopsign_idxs = [22649, 22691, 22693, 22704, 22706, 22718, 22720, 22735, 22749, 22753, 22764, 22768, 22782, 22784, 22793, 22797, 22799, 22809, 22811, 22815, 22827, 22831, 22841, 22843, 22845, 22847, 22857, 22861, 22863, 22872, 22878, 22889, 22893, 22904, 22908, 22917, 22919, 22921, 22923, 22936, 22938, 22946, 22950, 22952, 22962, 22964, 22966, 22968, 22972, 22974, 22976, 22980, 22984, 22988, 22992, 22996, 22998, 23000, 23004, 23006, 23008, 23010, 23014, 23016, 23025, 23029, 23031, 23040, 23042, 23046, 23057, 23061, 23075, 23077, 23087, 23089, 23091, 23093, 23103, 23109, 23123, 23125, 23137, 23139, 23141, 23152, 23154, 23165, 23169, 23171, 23180, 23186, 23196, 23198, 23200, 23210, 23212, 23216, 23226, 23230, 23232, 23245, 23247, 23256, 23258, 23262, 23287, 23289, 23322, 23338, 23352, 23354, 23366, 23367]
points_target = points[stopsign_idxs]

# intrinsic matrix is given by GEM car
intrinsic = [527.5779418945312, 0.0, 616.2459716796875, 0.0, 527.5779418945312, 359.2155456542969, 0.0, 0.0, 1.0]
intrinsic = np.array(intrinsic).reshape((3, 3))
intrinsic = np.concatenate([intrinsic, np.zeros((3, 1))], axis=1) # homogeneous transformation

# extrinsic matrix can be obtained by scipy.optmize
extrinsic = [[0.37446817, -0.59755021, 0.70901857, 6.127661841],
             [0.45366811, 0.78495263, 0.42194148, -4.36739205],
             [-0.80867721, 0.16365546, 0.56502925, -17.71628366],
             [0., 0., 0., 1.]] # homogeneous transformation

print (extrinsic)
extrinsic = np.asarray(extrinsic)

img = cv2.imread('../../../hw3_data/color4.png')
vis = img.copy()
print ('# of points_target:', len(points_target))
points_target = np.concatenate([points_target, np.zeros((len(points_target), 1))], axis=1) # homogeneous coord
for lidar_pt in points_target:
    img_pt = intrinsic @ extrinsic @ lidar_pt.reshape(-1, 1) # homogeneous coord
    print ("img_pt:", img_pt)
    x, y = int(img_pt[0] / img_pt[2]), int(img_pt[1] / img_pt[2]) # proj point
    vis = cv2.circle(vis, (x, y), 1, (0, 255, 0), cv2.FILLED)
    
imshow('vis', vis)