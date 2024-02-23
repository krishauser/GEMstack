""" 
    This script is for verifying extrinsic matrix we found.
"""
import numpy as np
import cv2

def imshow(window_name, image):
    cv2.imshow(window_name, image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

lidar_data = np.load('../../../hw3_data/lidar4.npz')
lidar_pts = lidar_data['arr_0']
print (lidar_pts.shape)

# stopsign_idxs can be obtained using "open3D_select_object_by_box.py"
stopsign_idxs = [22649, 22691, 22693, 22704, 22706, 22718, 22720, 22735, 22749, 22753, 22764, 22768, 22782, 22784, 22793, 22797, 22799, 22809, 22811, 22815, 22827, 22831, 22841, 22843, 22845, 22847, 22857, 22861, 22863, 22872, 22878, 22889, 22893, 22904, 22908, 22917, 22919, 22921, 22923, 22936, 22938, 22946, 22950, 22952, 22962, 22964, 22966, 22968, 22972, 22974, 22976, 22980, 22984, 22988, 22992, 22996, 22998, 23000, 23004, 23006, 23008, 23010, 23014, 23016, 23025, 23029, 23031, 23040, 23042, 23046, 23057, 23061, 23075, 23077, 23087, 23089, 23091, 23093, 23103, 23109, 23123, 23125, 23137, 23139, 23141, 23152, 23154, 23165, 23169, 23171, 23180, 23186, 23196, 23198, 23200, 23210, 23212, 23216, 23226, 23230, 23232, 23245, 23247, 23256, 23258, 23262, 23287, 23289, 23322, 23338, 23352, 23354, 23366, 23367]
lidar_pts = lidar_pts[stopsign_idxs]

# intrinsic matrix is given by GEM car
intrinsic = [527.5779418945312, 0.0, 616.2459716796875, 0.0, 527.5779418945312, 359.2155456542969, 0.0, 0.0, 1.0]
intrinsic = np.array(intrinsic).reshape((3, 3))
intrinsic = np.concatenate([intrinsic, np.zeros((3, 1))], axis=1) # homogeneous transformation

# extrinsic matrix can be obtained by scipy.optmize
# this one is most close to the answer
extrinsic = [[ 0.35282628 , -0.9356864 ,  0.00213977, -1.42526548],
             [-0.04834961 , -0.02051524, -0.99861977, -0.02062586],
             [ 0.93443883 ,  0.35223584, -0.05247839, -0.15902421],
             [ 0.         ,  0.        ,  0.        ,  1.        ]]

# extrinsic = [[ 0.58570256,  0.80857743,  0.05616982, -2.97855635],
#  [-0.17858055,  0.06113821,  0.98202398,  1.08055973],
#  [ 0.79060831, -0.5852048 ,  0.18020501, -7.918656  ],
#  [ 0.        ,  0.        ,  0.        ,  1.        ]]


extrinsic = np.asarray(extrinsic)

img = cv2.imread('../../../hw3_data/color4.png')
vis = img.copy()
print ('# of points_target:', len(lidar_pts))
lidar_pts = np.concatenate([lidar_pts, np.ones((len(lidar_pts), 1))], axis=1) # homogeneous coord
camera_pts = extrinsic @ lidar_pts.T # (4, N)
proj_pts = intrinsic @ camera_pts # (3, N)  homogeneous coord
proj_pts /= proj_pts[2, :] # normalize 

for idx, proj_pt in enumerate(proj_pts.T):
    print ('proj_pt:', proj_pt)
    color = (0, 255, 0) # green
    radius = 1
    center = int(proj_pt[0]), int(proj_pt[1])
    vis = cv2.circle(vis, center, 1, color, cv2.FILLED)
    
imshow('vis', vis)