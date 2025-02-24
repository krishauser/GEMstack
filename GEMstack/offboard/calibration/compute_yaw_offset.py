#!/usr/bin/env python3
import numpy as np
import open3d as o3d
import argparse

def load_lidar_scan(lidar_file):
    """Load a LiDAR scan from an npz or npy file."""
    data = np.load(lidar_file, allow_pickle=True)
    if isinstance(data, np.ndarray):
        return data.astype(np.float32)
    else:
        key = list(data.keys())[0]
        return data[key].astype(np.float32)

def manual_crop(pcd):
    """Interactively select a polygon region to crop the LiDAR point cloud."""
    print("\nCrop the LiDAR scan:")
    print("1. Press 'K' to lock the screen and enter selection mode.")
    print("2. Use Ctrl + Left Click to draw a polygon around the region of interest.")
    print("3. Press 'C' to crop the selection, then 'F' to exit.")
    vis = o3d.visualization.VisualizerWithEditing()
    vis.create_window(window_name="Crop Point Cloud", width=800, height=600)
    vis.add_geometry(pcd)
    vis.run()  # User draws polygon and crops
    vis.destroy_window()
    # Extract cropped points
    cropped_pcd = vis.get_cropped_geometry()
    return cropped_pcd

def manual_select_objects(pcd):
    """Interactively select calibration objects in the cropped point cloud."""
    print("\nSelect calibration objects:")
    print("Use SHIFT+LEFT CLICK to pick points on each calibration object.")
    print("Press 'Q' when done.")
    vis = o3d.visualization.VisualizerWithEditing()
    vis.create_window(window_name="Select Calibration Objects", width=800, height=600)
    vis.add_geometry(pcd)
    vis.run()  # User selects points
    vis.destroy_window()
    picked_indices = vis.get_picked_points()
    if not picked_indices:
        print("No points selected. Exiting.")
        exit(1)
    selected = np.asarray(pcd.points)[picked_indices, :]
    return selected

def get_vehicle_reference_points(num_points):
    """Prompt for vehicle frame coordinates of each selected object."""
    vehicle_points = []
    for i in range(num_points):
        inp = input(f"Enter vehicle (x,y) for object {i+1} (e.g., 5.5,0): ")
        try:
            x, y = map(float, inp.split(','))
            vehicle_points.append([x, y])
        except:
            print("Invalid input. Use comma-separated values (e.g., 5.5,0).")
            exit(1)
    return np.array(vehicle_points)

def compute_yaw_offset(lidar_xy, vehicle_xy):
    """Compute yaw offset (degrees) between LiDAR and vehicle frames."""
    A = lidar_xy - np.mean(lidar_xy, axis=0)
    B = vehicle_xy - np.mean(vehicle_xy, axis=0)
    H = A.T @ B
    U, _, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    yaw_rad = np.arctan2(R[1, 0], R[0, 0])
    return np.degrees(yaw_rad)

def main(args):
    # Step 1: Load LiDAR scan
    points = load_lidar_scan(args.lidar_scan)
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    print(f"Loaded LiDAR scan with {len(points)} points.")

    # Step 2: Crop the point cloud interactively
    cropped_pcd = manual_crop(pcd)
    print(f"Cropped to {len(cropped_pcd.points)} points.")

    # Step 3: Select calibration objects in the cropped cloud
    selected_points = manual_select_objects(cropped_pcd)
    print(f"Selected {len(selected_points)} calibration objects.")

    # Step 4: Get vehicle frame coordinates for these objects
    vehicle_xy = get_vehicle_reference_points(len(selected_points))
    
    # Step 5: Compute yaw offset
    lidar_xy = selected_points[:, :2]
    yaw_offset_deg = compute_yaw_offset(lidar_xy, vehicle_xy)
    print(f"Yaw offset (LiDAR -> Vehicle): {yaw_offset_deg:.2f}°")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Crop LiDAR scan and compute yaw offset.")
    parser.add_argument("--lidar_scan", type=str, required=True,
                        help="Path to LiDAR scan file (e.g., lidar_0.npz)")
    args = parser.parse_args()
    main(args)
