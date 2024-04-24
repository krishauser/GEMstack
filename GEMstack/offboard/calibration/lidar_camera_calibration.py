import numpy as np
from scipy.optimize import least_squares,minimize,differential_evolution
from scipy.spatial.transform import Rotation as R
from scipy.linalg import svd
import pandas as pd
import cv2
import pathlib
import os

data_idx = 6
lidar_csv_fn = os.path.join(f'./save/pointcld{data_idx}.csv')
image_csv_fn = os.path.join(f'./save/image{data_idx}.csv')
output_fn = 'GEMstack/knowledge/calibration/lidar2zed.txt'

def sse_reprojection_error(params, object_points, image_points, camera_matrix):
    
    r_vec = params[:3]
    t_vec = params[3:]
    
    # Convert rotation vector to rotation matrix
    R, _ = cv2.Rodrigues(r_vec)
    T = np.array(t_vec).reshape(3, 1)
    
    extrinsic_matrix = np.hstack((R, T)) # (3, 4)

    sse_error = 0
    for point_3d, point_2d in zip(object_points, image_points):
        # Transform 3D points to 2D cords
        point_cam = np.dot(extrinsic_matrix, np.append(point_3d, 1))
        
        # Transform 2D cords to pixel cords
        point_img = np.dot(camera_matrix, point_cam[:3])
        point_img /= point_img[2]  # normalization
        
        sse_error += np.sum((point_img[:2] - point_2d)**2)
    print("SSE error:", sse_error)
    return sse_error


# Camera intrinsic matrix
# K: [684.8333129882812, 0.0, 573.37109375, 0.0, 684.6096801757812, 363.700927734375, 0.0, 0.0, 1.0]
focal_length_x = 684.8333129882812
focal_length_y = 684.6096801757812
principal_point_x = 573.37109375
principal_point_y = 363.700927734375
camera_matrix = np.array([[focal_length_x, 0, principal_point_x],
                          [0, focal_length_y, principal_point_y],
                          [0, 0, 1]])

# load files
image_points_df = pd.read_csv(image_csv_fn)
image_points = image_points_df[['X', 'Y']].to_numpy()

object_points_df = pd.read_csv(lidar_csv_fn)
object_points = object_points_df[['x', 'y', 'z']].to_numpy()

# initial guess
# initial_camera_params = np.ones(6)
np.random.seed(10)
initial_camera_params = np.random.randn(6)

# optimizer
# res = least_squares(reprojection_error, initial_camera_params, args=(object_points, image_points, camera_matrix)
# res = minimize(sse_reprojection_error, initial_camera_params, args=(object_points, image_points, camera_matrix),
#                method='BFGS', options={'maxiter': 5000, 'gtol': 1e-6})

tol = 1e-6 

# Try different optimizer
# methods = ['Nelder-Mead', 'TNC', 'L-BFGS-B']
methods = ['L-BFGS-B']
for method in methods:
    res = minimize(sse_reprojection_error, initial_camera_params, args=(object_points, image_points, camera_matrix),
                      method=method, options={'maxiter': 50000})
    print(f"Optimization Result using {method}:", res)

optimized_rvec, optimized_tvec = res.x[:3], res.x[3:]
optimized_rotation_matrix = R.from_rotvec(optimized_rvec).as_matrix()
optimized_transform_matrix = np.eye(4)
optimized_transform_matrix[:3, :3] = optimized_rotation_matrix
optimized_transform_matrix[:3, 3] = optimized_tvec

print("Optimized transform matrix:\n", optimized_transform_matrix)

np.savetxt(output_fn, optimized_transform_matrix)
print("Save to:", output_fn)