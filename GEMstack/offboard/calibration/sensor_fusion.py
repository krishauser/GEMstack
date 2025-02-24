#!/usr/bin/env python3
import cv2
import numpy as np
import json
import argparse

def load_extrinsics(extrinsics_file):
    """
    Load calibrated extrinsics from a .npz file.
    Assumes the file contains keys 'R' and 't'.
    """
    data = np.load(extrinsics_file)
    R = data['R']
    t = data['t']
    return R, t

def load_intrinsics(intrinsics_file):
    """
    Load camera intrinsics from a JSON file.
    Expects keys: 'fx', 'fy', 'cx', and 'cy'.
    """
    with open(intrinsics_file, 'r') as f:
        intrinsics = json.load(f)
    fx = intrinsics["fx"]
    fy = intrinsics["fy"]
    cx = intrinsics["cx"]
    cy = intrinsics["cy"]
    K = np.array([[fx, 0, cx],
                  [0, fy, cy],
                  [0,  0,  1]], dtype=np.float32)
    return K

def load_lidar_scan(lidar_file):
    """
    Load a LiDAR scan from a file.
    This function handles both npy (returns a NumPy array) and npz files.
    """
    data = np.load(lidar_file, allow_pickle=True)
    if isinstance(data, np.ndarray):
        return data.astype(np.float32)
    else:
        key = list(data.keys())[0]
        return data[key].astype(np.float32)

def transform_lidar_points(lidar_points, R, t):
    """
    Transform LiDAR points from the LiDAR frame into the camera frame.
    p_cam = R * p_lidar + t.
    """
    P_cam = (R @ lidar_points.T + t.reshape(3,1)).T
    return P_cam

def project_points(points_3d, K):
    """
    Project 3D points (in the camera frame) into 2D image coordinates using the camera matrix K.
    Only projects points with z > 0.
    """
    proj_points = []
    for pt in points_3d:
        if pt[2] > 0:  # only project points in front of the camera
            u = K[0, 0] * (pt[0] / pt[2]) + K[0, 2]
            v = K[1, 1] * (pt[1] / pt[2]) + K[1, 2]
            proj_points.append((int(u), int(v)))
    return proj_points

def main(args):
    # Load extrinsics (R and t)
    R, t = load_extrinsics(args.extrinsics)
    print("Loaded extrinsics")
    
    # Load camera intrinsics.
    K = load_intrinsics(args.intrinsics)
    print("Loaded intrinsics")
    
    # Load the LiDAR scan (e.g., calibration points or full scan)
    lidar_points = load_lidar_scan(args.lidar_scan)
    print("Loaded LiDAR scan with {} points".format(lidar_points.shape[0]))
    
    # Transform LiDAR points into the camera coordinate frame.
    lidar_in_camera = transform_lidar_points(lidar_points, R, t)
    
    # Project the transformed points into the image plane.
    projected_pts = project_points(lidar_in_camera, K)
    
    # Load the camera image.
    image = cv2.imread(args.image)
    if image is None:
        print("Error: could not load image", args.image)
        return
    
    # Draw projected LiDAR points on the image.
    for pt in projected_pts:
        cv2.circle(image, pt, 2, (0, 0, 255), -1)
    
    # Display the resulting image.
    cv2.imshow("LiDAR Points Projected onto Image", image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Sensor Fusion: Project LiDAR points into the camera image using extrinsic calibration.")
    parser.add_argument("--extrinsics", type=str, required=True,
                        help="Path to extrinsics file (e.g., extrinsics.npz)")
    parser.add_argument("--intrinsics", type=str, required=True,
                        help="Path to camera intrinsics JSON file (e.g., camera_intrinsics.json)")
    parser.add_argument("--lidar_scan", type=str, required=True,
                        help="Path to the LiDAR scan file (e.g., lidar_cal_points.npy)")
    parser.add_argument("--image", type=str, required=True,
                        help="Path to the camera image file (e.g., image_0.png)")
    args = parser.parse_args()
    main(args)
