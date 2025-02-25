#!/usr/bin/env python3
import numpy as np
import cv2
import matplotlib.pyplot as plt
import json
import argparse

def pick_image_points(image):
    """
    Display the image using matplotlib and let the user pick 
    calibration points interactively.
    """
    plt.imshow(image)
    plt.title("Click to select corresponding calibration points.\nPress Enter when done.")
    pts = plt.ginput(n=-1, timeout=0)
    plt.close()
    image_points = np.array(pts, dtype=np.float32)
    return image_points

def compute_reprojection_error(object_points, image_points, rvec, tvec, camera_matrix, dist_coeffs):
    projected_points, _ = cv2.projectPoints(object_points, rvec, tvec, camera_matrix, dist_coeffs)
    projected_points = projected_points.squeeze()
    errors = np.linalg.norm(image_points - projected_points, axis=1)
    return np.mean(errors)

def main(args):
    # Load LiDAR calibration points (3D points). These should be in the same coordinate ordering as your image points.
    lidar_points = np.load(args.lidar_points)  # e.g., lidar_cal_points.npy
    print("Loaded LiDAR calibration points:")
    print(lidar_points)
    
    # Load the calibration image.
    image = cv2.imread(args.image)
    if image is None:
        print("Error: could not load image", args.image)
        return
    image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    
    # Pick corresponding 2D image points.
    print("Select corresponding feature points in the image, in the same order as the LiDAR points.")
    image_points = pick_image_points(image_rgb)
    
    if image_points.shape[0] != lidar_points.shape[0]:
        print("Error: number of image points and LiDAR points do not match.")
        return

    # Load camera intrinsics.
    with open(args.intrinsics, "r") as f:
        intrinsics = json.load(f)
    fx = intrinsics["fx"]
    fy = intrinsics["fy"]
    cx = intrinsics["cx"]
    cy = intrinsics["cy"]
    camera_matrix = np.array([[fx, 0, cx],
                              [0, fy, cy],
                              [0,  0, 1]], dtype=np.float32)
    # Assume no lens distortion (or load them if available).
    dist_coeffs = np.zeros((4,1), dtype=np.float32)

    # Use cv2.solvePnPRansac for a robust initial estimate.
    print("Computing initial solution using solvePnPRansac...")
    retval, rvec, tvec, inliers = cv2.solvePnPRansac(
        objectPoints=lidar_points,
        imagePoints=image_points,
        cameraMatrix=camera_matrix,
        distCoeffs=dist_coeffs,
        flags=cv2.SOLVEPNP_EPNP
    )
    if not retval:
        print("solvePnPRansac failed to find a solution.")
        return
    print("RANSAC output:")
    print("Rotation vector (rvec):", rvec)
    print("Translation vector (tvec):", tvec)
    if inliers is not None:
        print("Number of inliers:", len(inliers))
    
    # Refine the solution with iterative optimization.
    print("Refining solution using solvePnP (SOLVEPNP_ITERATIVE)...")
    retval, rvec_refined, tvec_refined = cv2.solvePnP(
        objectPoints=lidar_points,
        imagePoints=image_points,
        cameraMatrix=camera_matrix,
        distCoeffs=dist_coeffs,
        rvec=rvec,
        tvec=tvec,
        useExtrinsicGuess=True,
        flags=cv2.SOLVEPNP_ITERATIVE
    )
    if not retval:
        print("Refinement with solvePnP failed.")
        return
    
    # Compute reprojection error using the refined solution.
    mean_error = compute_reprojection_error(lidar_points, image_points, rvec_refined, tvec_refined, camera_matrix, dist_coeffs)
    print("Mean reprojection error (refined):", mean_error)
    
    # Convert the refined rotation vector to a rotation matrix.
    R_refined, _ = cv2.Rodrigues(rvec_refined)
    print("Estimated refined Rotation Matrix:\n", R_refined)
    print("Estimated refined Translation Vector:\n", tvec_refined)
    
    # Visualize reprojection: project the LiDAR points onto the image using the refined extrinsics.
    projected_img_points, _ = cv2.projectPoints(lidar_points, rvec_refined, tvec_refined, camera_matrix, dist_coeffs)
    projected_img_points = projected_img_points.squeeze()
    
    for pt in projected_img_points:
        cv2.circle(image, (int(pt[0]), int(pt[1])), 5, (0, 0, 255), -1)
    
    cv2.imshow("Reprojected LiDAR Points", image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    
    # Save the refined extrinsics to disk.
    extrinsics = {"R": R_refined.tolist(), "t": tvec_refined.tolist(), "rvec": rvec_refined.tolist()}
    np.savez(args.output, **extrinsics)
    print("Extrinsics saved to:", args.output)

if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description="Robust Extrinsic Calibration: Compute LiDAR-to-Camera transformation using RANSAC and iterative refinement."
    )
    parser.add_argument("--lidar_points", type=str, required=True,
                        help="Path to saved LiDAR calibration points file (e.g., lidar_cal_points.npy)")
    parser.add_argument("--image", type=str, required=True,
                        help="Path to a calibration image (e.g., image_0.png)")
    parser.add_argument("--intrinsics", type=str, required=True,
                        help="Path to camera intrinsics JSON file (e.g., camera_intrinsics.json)")
    parser.add_argument("--output", type=str, default="extrinsics.npz",
                        help="Output file name for saving computed extrinsics")
    args = parser.parse_args()
    main(args)
