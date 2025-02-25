#!/usr/bin/env python3
import open3d as o3d
import numpy as np
import argparse
import os
import cv2

def load_lidar_scan(npz_path):
    """
    Load a LiDAR point cloud from a file.
    Assumes the file stores one array of shape (N, 3), either in
    an npz container or as a direct npy file.
    """
    if not os.path.isfile(npz_path):
        raise FileNotFoundError(f"Could not find LiDAR file: {npz_path}")
    data = np.load(npz_path, allow_pickle=True)
    # If the file is saved as an npy array directly
    if isinstance(data, np.ndarray):
        return data.astype(np.float32)
    else:
        # Else, assume it's an npz file and get the first stored array.
        key = list(data.keys())[0]
        return data[key].astype(np.float32)

def crop_ground_points(pcd):
    """
    Use Open3D's interactive VisualizerWithEditing to let you select
    vertices that define a polygon bounding the flat, level ground.
    This routine then selects all points that fall within the
    defined polygon (based on the x-y coordinates).
    """
    print("Crop ground region:")
    print("  Use shift+click to select vertices that enclose the flat ground area (e.g., a parking lot).")
    print("  Press 'q' (or close the window) when finished.")
    
    vis = o3d.visualization.VisualizerWithEditing()
    vis.create_window(window_name="Crop Ground", width=800, height=600)
    vis.add_geometry(pcd)
    vis.run()  # Wait for user interaction.
    vis.destroy_window()
    
    picked_indices = vis.get_picked_points()
    print("DEBUG: Picked indices for cropping:", picked_indices)
    if not picked_indices:
        print("No cropping vertices selected; returning the original point cloud.")
        return pcd

    # Get the (x,y) coordinates of the picked vertices.
    picked_points = np.asarray(pcd.points)[picked_indices, :]
    polygon = picked_points[:, :2]

    # Prepare the polygon for cv2.pointPolygonTest:
    contour = polygon.reshape((-1, 1, 2)).astype(np.float32)

    all_points = np.asarray(pcd.points)
    indices_inside = []
    for i, pt in enumerate(all_points):
        # Test using the (x,y) coordinates.
        if cv2.pointPolygonTest(contour, (pt[0], pt[1]), False) >= 0:
            indices_inside.append(i)
    
    print("DEBUG: Number of points inside polygon:", len(indices_inside))
    cropped_pcd = pcd.select_by_index(indices_inside)
    return cropped_pcd

def main(args):
    # Load the LiDAR scan (ideally a full scan that includes the ground)
    lidar_points = load_lidar_scan(args.lidar_scan)
    print("Loaded LiDAR scan with {} points".format(lidar_points.shape[0]))
    
    # Create an Open3D point cloud from the data.
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(lidar_points)
    
    # Ask user if they want to crop to isolate the ground.
    crop_response = input("Do you want to crop the point cloud to the ground region? (y/n): ").strip().lower()
    if crop_response == 'y':
        pcd = crop_ground_points(pcd)
        print("After cropping, {} points remain.".format(np.asarray(pcd.points).shape[0]))
        o3d.visualization.draw_geometries([pcd], window_name="Cropped Ground")
    
    # Save the cropped (ground only) points to a .npy file.
    np.save(args.output, np.asarray(pcd.points))
    print("Ground points saved to", args.output)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Extract ground (flat) LiDAR points for calibration.")
    parser.add_argument("--lidar_scan", type=str, required=True,
                        help="Path to the LiDAR scan file (e.g., a full scan in .npz or .npy format)")
    parser.add_argument("--output", type=str, required=True,
                        help="Output file name for the ground points (e.g., ground_points.npy)")
    args = parser.parse_args()
    main(args)
