#!/usr/bin/env python3
"""
Absolute Calibration of Vehicle Geometry and Sensor Mounts

This script computes:
    • T_toplidar^vehicle: the homogeneous transformation from the top LiDAR frame to the vehicle frame,
      where the vehicle frame is defined with its origin at the rear axle center, x pointing forward,
      y pointing left, and z pointing up.
      
    The calibration uses:
      - A LiDAR scan of the flat ground to determine pitch and roll (by plane fitting) and the height above ground.
      - A measured yaw offset (from distinctive calibration objects placed along the vehicle centerline) to correct horizontal alignment.
      - A manual measurement of rear axle height above the ground.

    Once T_toplidar^vehicle is determined, T_frontcamera^vehicle is computed via:
        T_frontcamera^vehicle = T_toplidar^vehicle * inv(T_toplidar^frontcamera)
        
    The transform T_toplidar^frontcamera is assumed to be computed in an earlier calibration step (e.g., using calibrate_extrinsics.py).

The resulting calibrations are saved into a YAML file (e.g., gem_e4.yaml) with metadata.
"""

import numpy as np
import cv2
import argparse
import datetime
import yaml

# ---------------------------
# Utility Functions
# ---------------------------

def load_transform(npz_file):
    """
    Load a 4x4 homogeneous transform from an npz file.
    Assumes the file contains keys 'R' and 't'.
    """
    data = np.load(npz_file)
    R = data['R']
    t = data['t']
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t.flatten()
    return T

def fit_ground_plane(points):
    """
    Fit a plane to ground LiDAR points using SVD.
    Returns the normalized plane normal and the centroid.
    Here, points is an (N,3) NumPy array.
    """
    centroid = np.mean(points, axis=0, dtype=np.float32)

    # SVD on the centered data
    _, _, Vt = np.linalg.svd((points - centroid).astype(np.float32))

    normal = Vt[-1, :]   # The smallest singular value gives the normal direction
    # Ensure the normal points upward (z>0 in vehicle frame)
    if normal[2] < 0:
        normal = -normal
    normal /= np.linalg.norm(normal)
    return normal, centroid

def compute_R_ground(normal):
    """
    Compute the rotation matrix that aligns the measured ground normal to the vehicle's 
    desired ground normal [0, 0, 1] (i.e., z upward).
    This correction removes pitch and roll errors.
    """
    desired = np.array([0, 0, 1], dtype=np.float32)
    # Compute axis of rotation and angle between the normals
    axis = np.cross(normal, desired)
    axis_norm = np.linalg.norm(axis)
    if axis_norm < 1e-6:
        # Already aligned; return identity
        return np.eye(3, dtype=np.float32)
    axis = axis / axis_norm
    angle = np.arccos(np.clip(np.dot(normal, desired), -1.0, 1.0))
    # Rodrigues formula to convert axis-angle to rotation matrix
    K = np.array([[0, -axis[2], axis[1]],
                  [axis[2], 0, -axis[0]],
                  [-axis[1], axis[0], 0]], dtype=np.float32)
    R = np.eye(3, dtype=np.float32) + np.sin(angle) * K + (1 - np.cos(angle)) * (K @ K)
    return R

def create_T_toplidar_vehicle(ground_points, yaw_offset_deg, rear_axle_height):
    """
    Using ground plane fit and measured yaw offset and rear axle height, compute the 
    transform T_toplidar^vehicle.
    
    Parameters:
    - ground_points: (N,3) LiDAR scan points that belong to the ground (use a cropped region).
    - yaw_offset_deg: Measured yaw offset (in degrees) between the LiDAR and vehicle centerline.
    - rear_axle_height: Measured height of the rear axle center above the ground (in meters).
    
    Returns:
    - A 4x4 homogeneous matrix T_toplidar_vehicle.
    """
    # Fit the ground plane to obtain the measured ground normal.
    normal, centroid = fit_ground_plane(ground_points)
    # Compute rotation (pitch & roll correction) to align measured ground to [0,0,1].
    R_ground = compute_R_ground(normal)
    # Yaw correction: rotation about the z-axis (vehicle's vertical axis)
    yaw_offset_rad = np.radians(yaw_offset_deg)
    R_yaw = np.array([[np.cos(yaw_offset_rad), -np.sin(yaw_offset_rad), 0],
                      [np.sin(yaw_offset_rad),  np.cos(yaw_offset_rad), 0],
                      [0, 0, 1]], dtype=np.float32)
    # Combined sensor-to-vehicle rotation: first align pitch/roll, then adjust yaw
    R_vehicle = R_yaw @ R_ground
    # For translation: we determine the desired z offset as the measured rear axle height.
    # Assume x and y offsets between the sensor and the vehicle frame are negligible,
    # or have been measured separately (set to 0 here).
    t_vehicle = np.array([0, 0, rear_axle_height], dtype=np.float32)
    # Build homogeneous transform
    T = np.eye(4, dtype=np.float32)
    T[:3, :3] = R_vehicle
    T[:3, 3] = t_vehicle
    return T

def combine_transforms(T_toplidar_vehicle, T_toplidar_frontcamera):
    """
    Given T_toplidar^vehicle and T_toplidar^frontcamera (from previous front-camera calibration),
    compute T_frontcamera^vehicle:
         T_frontcamera^vehicle = T_toplidar^vehicle * inv(T_toplidar^frontcamera)
    """
    T_frontcamera_vehicle = T_toplidar_vehicle @ np.linalg.inv(T_toplidar_frontcamera)
    return T_frontcamera_vehicle

def save_calibration(filename, T_toplidar_vehicle, T_frontcamera_vehicle, vehicle, calibration_type):
    """
    Save the calibration transforms and metadata to a YAML file.
    """
    calibration_data = {
        "vehicle": vehicle,
        "calibration_type": calibration_type,
        "date": str(datetime.datetime.now()),
        "T_toplidar_vehicle": T_toplidar_vehicle.tolist(),
        "T_frontcamera_vehicle": T_frontcamera_vehicle.tolist()
    }
    with open(filename, 'w') as f:
        yaml.dump(calibration_data, f)
    print("Calibration saved to", filename)

# ---------------------------
# Main routine
# ---------------------------
def main(args):
    # For ground calibration, load a LiDAR scan file that covers a flat floor.
    # It is assumed that the file contains points that belong to the ground.
    ground_points = np.load(args.ground_lidar_points).astype(np.float32)

    print("Loaded ground LiDAR points with shape:", ground_points.shape)
    
    # Compute T_toplidar^vehicle from ground plane and measurements.
    T_toplidar_vehicle = create_T_toplidar_vehicle(ground_points, args.yaw_offset, args.rear_axle_height)
    print("Computed T_toplidar^vehicle:")
    print(T_toplidar_vehicle)
    
    # Load the previously computed sensor calibration T_toplidar^frontcamera.
    T_toplidar_frontcamera = load_transform(args.toplidar_frontcamera)
    print("Loaded T_toplidar^frontcamera:")
    print(T_toplidar_frontcamera)
    
    # Compute T_frontcamera^vehicle.
    T_frontcamera_vehicle = combine_transforms(T_toplidar_vehicle, T_toplidar_frontcamera)
    print("Computed T_frontcamera^vehicle:")
    print(T_frontcamera_vehicle)
    
    # Save the calibration to a YAML file with metadata.
    save_calibration(args.output_yaml, T_toplidar_vehicle, T_frontcamera_vehicle, args.vehicle, "absolute_vehicle_calibration")

if __name__ == '__main__':
    # Import datetime and yaml here
    import datetime, yaml
    parser = argparse.ArgumentParser(
        description="Absolute Calibration of Vehicle Frame: Compute transforms T_toplidar^vehicle and T_frontcamera^vehicle."
    )
    parser.add_argument("--ground_lidar_points", default = "./data/ground_points.npy", type=str,
                        help="Path to LiDAR scan file with ground points (e.g., ground_points.npy)")
    parser.add_argument("--toplidar_frontcamera", default = "./data/extrinsics.npz", type=str,
                        help="Path to sensor calibration file for T_toplidar^frontcamera (e.g., extrinsics.npz)")
    parser.add_argument("--rear_axle_height", default = 0.33, type=float,
                        help="Measured rear axle height above the ground (in meters)")
    parser.add_argument("--yaw_offset",default = 0.12, type=float,
                        help="Measured yaw offset (in degrees) from calibration objects on the vehicle centerline")
    parser.add_argument("--vehicle", default = "gem_e4", type=str,
                        help="Vehicle identifier (e.g., gem_e4)")
    parser.add_argument("--output_yaml", default = "gem_e4.yaml",type=str,
                        help="Output YAML file path (e.g., gem_e4.yaml)")
    args = parser.parse_args()
    main(args)
