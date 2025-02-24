#!/usr/bin/env python3
import open3d as o3d
import numpy as np
import argparse
import os
import cv2

def load_lidar_scan(npz_path):
    """
    Load the LiDAR point cloud from the npz file.
    Assumes the file stores one array with shape (N, 3).
    """
    if not os.path.isfile(npz_path):
        raise FileNotFoundError(f"Could not find LiDAR file: {npz_path}")
    data = np.load(npz_path)
    # Extract the first stored array (assumed to be the whole point cloud).
    lidar_points = data[list(data.keys())[0]]
    return lidar_points.astype(np.float32)

def crop_point_cloud(pcd):
    """
    Use Open3D's VisualizerWithEditing to let the user pick polygon vertices.
    Then, using these vertices (projected in the x-y plane), select all points
    from the point cloud that lie within the polygon.
    
    This method enables you to crop a dense region rather than only returning the selected vertices.
    """
    print("Cropping step:")
    print("  Use shift+click to select vertices that define the boundary of the region of interest (e.g., the face of the box).")
    print("  After clicking the desired vertices (e.g., 4 points), press 'q' or close the window.")
    
    vis = o3d.visualization.VisualizerWithEditing()
    vis.create_window(window_name="Crop ROI", width=800, height=600)
    vis.add_geometry(pcd)
    vis.run()  # User interacts and then closes.
    vis.destroy_window()
    
    picked_indices = vis.get_picked_points()
    print("DEBUG: Cropping picked indices:", picked_indices)
    if not picked_indices:
        print("No cropping indices selected. Returning original point cloud.")
        return pcd
    
    # Get the points you clicked.
    picked_points = np.asarray(pcd.points)[picked_indices, :]
    # Use the first two coordinates (x,y) to define the polygon.
    poly_2d = picked_points[:, :2]
    
    # Prepare the polygon contour for cv2.pointPolygonTest.
    # It expects an array of shape (n, 1, 2) of type float32.
    contour = poly_2d.reshape((-1, 1, 2)).astype(np.float32)
    
    # Now, iterate over all points in the cloud and test if they fall inside the polygon.
    all_points = np.asarray(pcd.points)
    indices_inside = []
    for i, pt in enumerate(all_points):
        # Test using (x, y) of the current point.
        if cv2.pointPolygonTest(contour, (pt[0], pt[1]), False) >= 0:
            indices_inside.append(i)
    
    print("DEBUG: Number of points inside cropping polygon:", len(indices_inside))
    cropped_pcd = pcd.select_by_index(indices_inside)
    return cropped_pcd

def pick_calibration_points(pcd):
    """
    Use Open3D's VisualizerWithEditing to interactively pick calibration points
    from the (possibly cropped) point cloud.
    
    The user should select the key feature points (e.g., corners of the calibration target).
    """
    print("Calibration point selection:")
    print("  Use shift+click to pick the feature points (e.g., corners) in the point cloud.")
    print("  Then press 'q' or close the window.")
    
    vis = o3d.visualization.VisualizerWithEditing()
    vis.create_window(window_name="Pick Calibration Points", width=800, height=600)
    vis.add_geometry(pcd)
    vis.run()
    vis.destroy_window()
    
    picked_indices = vis.get_picked_points()
    print("DEBUG: Calibration picked indices:", picked_indices)
    if not picked_indices:
        print("No calibration points were selected. Exiting.")
        exit(1)
    selected_points = np.asarray(pcd.points)[picked_indices, :]
    return selected_points

def main(args):
    # Load LiDAR scan from file.
    lidar_points = load_lidar_scan(args.lidar_npz)
    print(f"Loaded LiDAR scan with {lidar_points.shape[0]} points from {args.lidar_npz}")
    
    # Create an Open3D point cloud.
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(lidar_points)
    
    # Optional cropping: if the full scan is large, crop to the region containing your target.
    crop_response = input("Do you want to crop the point cloud? (y/n): ").strip().lower()
    if crop_response == 'y':
        pcd = crop_point_cloud(pcd)
        print(f"After cropping, point cloud has {np.asarray(pcd.points).shape[0]} points.")
        o3d.visualization.draw_geometries([pcd], window_name="Cropped Point Cloud")
    
    # Pick calibration points from the (possibly cropped) point cloud.
    cal_points = pick_calibration_points(pcd)
    if cal_points.shape[0] < 3:
        print("Error: You must select at least 3 points for calibration.")
        exit(1)
    print("Selected LiDAR calibration points:")
    print(cal_points)
    
    # Save selected calibration points to file.
    output_path = args.output
    np.save(output_path, cal_points)
    print(f"Calibration points saved to {output_path}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Interactively pick LiDAR calibration points (with optional cropping) from a captured LiDAR scan."
    )
    parser.add_argument("--lidar_npz", type=str, required=True,
                        help="Path to the LiDAR npz file (from the paired capture)")
    parser.add_argument("--output", type=str, default="lidar_cal_points.npy",
                        help="Output filename for saving the picked calibration points")
    args = parser.parse_args()
    main(args)
