import os
import numpy as np
import cv2
from scipy.spatial.transform import Rotation as R

# Define OAK camera intrinsic parameters
focal_length_x = 684.83  # Focal length in pixels
focal_length_y = 684.86
principal_point_x = 573.37  # Principal point in pixels
principal_point_y = 363.7

# Define OAK camera distortion coefficients (if available)

def euler_from_matrix(matrix):
    # Function to extract Euler angles from rotation matrix
    euler = R.from_matrix(matrix).as_euler('xyz', degrees=True)
    return euler

def calibrate(points2D=None, points3D=None):
    # Load corresponding points
    camera_points = np.genfromtxt('image6.csv', delimiter=',', skip_header=1)
    lidar_points = np.genfromtxt('pointcld6.csv', delimiter=',', skip_header=1)

    # Extract 2D and 3D points
    points2D = camera_points[:, 0:2]
    points3D = lidar_points[:, 0:3]
    # Check points shape
    assert(points2D.shape[0] == points3D.shape[0])
    if not (points2D.shape[0] >= 5):
        print('PnP RANSAC Requires minimum 5 points')
        return

    # Obtain camera matrix and distortion coefficients
    camera_matrix = np.array([[focal_length_x, 0, principal_point_x],
                              [0, focal_length_y, principal_point_y],
                              [0, 0, 1]])
    dist_coeffs = np.array([0.908803403377533, 0.10777730494737625, -0.00011555181117728353, 0.00016320882423315197, 0.021386317908763885, 0.8666334748268127, 0.18107722699642181, 0.012254921719431877])


    # Estimate extrinsics
    success, rotation_vector, translation_vector, inliers = cv2.solvePnPRansac(points3D, 
        points2D, camera_matrix, dist_coeffs, flags=cv2.SOLVEPNP_ITERATIVE)
    inliers = inliers.squeeze()
    # Compute re-projection error.
    points2D_reproj = cv2.projectPoints(points3D, rotation_vector,
        translation_vector, camera_matrix, dist_coeffs)[0].squeeze(1)
    print("points2D_reproj shape:", points2D_reproj.shape)
    print("points2D shape:", points2D.shape)
    print("inliers shape:", inliers.shape)  # Ensure inliers is a boolean mask
    assert(points2D_reproj.shape == points2D.shape)
    error = (points2D_reproj - points2D)[inliers]  # Compute error only over inliers.
    print("errors shape:", error.shape)

    rmse = np.sqrt(np.mean(error[:, 0] ** 2 + error[:, 1] ** 2))
    print('Re-projection error before LM refinement (RMSE) in px: ' + str(rmse))

    # Refine estimate using LM
    if not success:
        print('Initial estimation unsuccessful, skipping refinement')
    elif not hasattr(cv2, 'solvePnPRefineLM'):
        print('solvePnPRefineLM requires OpenCV >= 4.1.1, skipping refinement')
    else:
        assert len(inliers) >= 3, 'LM refinement requires at least 3 inlier points'
        rotation_vector, translation_vector = cv2.solvePnPRefineLM(points3D[inliers],
            points2D[inliers], camera_matrix, dist_coeffs, rotation_vector, translation_vector)

        # Compute re-projection error.
        points2D_reproj = cv2.projectPoints(points3D, rotation_vector,
            translation_vector, camera_matrix, dist_coeffs)[0].squeeze(1)
        assert(points2D_reproj.shape == points2D.shape)
        error = (points2D_reproj - points2D)[inliers]  # Compute error only over inliers.
        rmse = np.sqrt(np.mean(error[:, 0] ** 2 + error[:, 1] ** 2))
        print('Re-projection error after LM refinement (RMSE) in px: ' + str(rmse))

    # Convert rotation vector
    rotation_matrix = cv2.Rodrigues(rotation_vector)[0]
    euler = euler_from_matrix(rotation_matrix)
    
    # Save extrinsics
    np.savez(os.path.join('extrinsics.npz'),
        euler=euler, R=rotation_matrix, T=translation_vector.T)

    # Display results
    print('Euler angles (RPY):', euler)
    print('Rotation Matrix:', rotation_matrix)
    print('Translation Offsets:', translation_vector.T)

    extrinsics = {'euler': euler.tolist(), 'R': rotation_matrix.tolist(), 'T': translation_vector.T.tolist()}
    np.savez('extrinsics.npz', **extrinsics)
    
    return extrinsics

# Call the calibrate function with provided or default arguments
extrinsic_matrix = calibrate()
print('Extrinsic Matrix:', extrinsic_matrix)