#!/usr/bin/env python3

import os
import glob
import numpy as np
import cv2
import open3d as o3d

# --- Adjust these to match your intrinsics and data details ---
INTRINSICS = np.array([
    [684.83331299,   0.        , 573.37109375],
    [  0.        , 684.60968018, 363.70092773],
    [  0.        ,   0.        ,   1.        ]
], dtype=np.float32)

# Depth scale: how to go from raw .tif values to actual "Z" in meters.
# In your capture code, you used: dimage = (dimage/4000*0xffff)
# Possibly each pixel is stored with range 0..65535. Adjust as needed.
DEPTH_SCALE = 4000.0  # Example factor if your depth was in mm or a certain scale.

def depth_to_points(depth_img: np.ndarray, intrinsics: np.ndarray):
    """
    Convert a single-channel depth image into an Nx3 array of 3D points
    in the camera coordinate system.
    - depth_img: 2D array of type uint16 or float with depth values
    - intrinsics: 3x3 camera matrix
    """
    fx = intrinsics[0,0]
    fy = intrinsics[1,1]
    cx = intrinsics[0,2]
    cy = intrinsics[1,2]

    # Indices of each pixel
    h, w = depth_img.shape
    i_range = np.arange(h)
    j_range = np.arange(w)
    jj, ii = np.meshgrid(j_range, i_range)  # shape (h,w)

    # Flatten
    ii = ii.flatten().astype(np.float32)
    jj = jj.flatten().astype(np.float32)
    depth = depth_img.flatten().astype(np.float32)

    # Filter out zero / invalid depths
    valid_mask = (depth > 0)
    ii = ii[valid_mask]
    jj = jj[valid_mask]
    depth = depth[valid_mask]

    # Reproject to 3D (camera frame)
    # X = (x - cx) * Z / fx,  Y = (y - cy) * Z / fy,  Z = depth
    # Note: Z must be in meters or consistent units
    z = depth / DEPTH_SCALE  # Convert from your .tif scale to real meters
    x = (jj - cx) * z / fx
    y = (ii - cy) * z / fy
    points3d = np.stack((x, y, z), axis=-1)  # shape (N,3)

    return points3d

def load_lidar_points(npz_file: str):
    """
    Load the Nx3 LiDAR points from a .npz file created by 'np.savez'.
    """
    data = np.load(npz_file)
    # The capture code used: np.savez(lidar_fn, pc)
    # So 'data' might have the default key 'arr_0' or 'pc' if named
    # Inspect data.files to see. Let's assume 'arr_0' or single key:
    arr_key = data.files[0]
    points = data[arr_key]
    # shape check, must be Nx3 or Nx4, ...
    return points

def make_open3d_pcd(points: np.ndarray):
    """
    Convert Nx3 numpy array into an Open3D point cloud object.
    """
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    return pcd

def perform_icp(camera_pcd: o3d.geometry.PointCloud,
                lidar_pcd: o3d.geometry.PointCloud):
    """
    Perform local ICP alignment of camera_pcd (source) to lidar_pcd (target).
    Returns a transformation 4x4 that maps camera -> lidar (or vice versa).
    """
    # 1) Estimate normals if you want point-to-plane
    lidar_pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(
        radius=1.0, max_nn=30))
    camera_pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(
        radius=1.0, max_nn=30))

    # 2) Initial guess: identity or something better if you have one
    init_guess = np.eye(4)

    # 3) Choose a threshold for inlier distance
    threshold = 0.5  # adjust based on your environment scale

    # 4) Run ICP (point-to-plane or point-to-point)
    print("[ICP] Running ICP alignment ...")
    result = o3d.pipelines.registration.registration_icp(
        camera_pcd,  # source
        lidar_pcd,   # target
        threshold,
        init_guess,
        estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPlane()
    )

    print("[ICP] Done. Fitness = %.4f, RMSE = %.4f" % (result.fitness, result.inlier_rmse))
    print("[ICP] Transformation:\n", result.transformation)
    return result.transformation

def main(folder='data'):
    color_files = sorted(glob.glob(os.path.join(folder, "color*.png")))
    icp_results = []  # to store (index, transform, fitness, rmse)

    for color_path in color_files:
        # Extract index from filename, e.g. color10.png -> 10
        basename = os.path.basename(color_path)
        idx_str = basename.replace("color","").replace(".png","")

        if int(idx_str) in range(77):
            depth_path = os.path.join(folder, f"depth{idx_str}.tif")
            lidar_path = os.path.join(folder, f"lidar{idx_str}.npz")

            if not (os.path.exists(depth_path) and os.path.exists(lidar_path)):
                continue

            # Load depth / convert to 3D
            depth_img = cv2.imread(depth_path, cv2.IMREAD_UNCHANGED)
            if depth_img is None:
                continue
            camera_points = depth_to_points(depth_img, INTRINSICS)
            camera_pcd = make_open3d_pcd(camera_points)

            # Load LiDAR
            lidar_points = load_lidar_points(lidar_path)
            lidar_pcd = make_open3d_pcd(lidar_points)

            lidar_pcd.estimate_normals(
                search_param=o3d.geometry.KDTreeSearchParamHybrid(
                    radius=1.0,   # Adjust based on your scene scale
                    max_nn=30
                )
            )

            # Also estimate normals on the Camera (source) cloud (recommended)
            camera_pcd.estimate_normals(
                search_param=o3d.geometry.KDTreeSearchParamHybrid(
                    radius=1.0,
                    max_nn=30
                )
            )

            # Now you can run Point-to-Plane ICP
            init_guess = np.eye(4)
            threshold = 0.5  # or some appropriate distance
            result_icp = o3d.pipelines.registration.registration_icp(
                camera_pcd,
                lidar_pcd,
                threshold,
                init_guess,
                o3d.pipelines.registration.TransformationEstimationPointToPlane()
            )

            print("ICP result:", result_icp.transformation)
            print("Fitness:", result_icp.fitness, " RMSE:", result_icp.inlier_rmse)

    # After processing all frames, we can analyze or average
    if len(icp_results) > 0:
        # Example: pick the best frame by highest fitness
        best_frame = max(icp_results, key=lambda x: x[2])  # x[2] is fitness
        print("\nBest frame by fitness was index=", best_frame[0],
            " with fitness=", best_frame[2], " rmse=", best_frame[3])
    else:
        print("No frames processed properly.")

if __name__ == "__main__":
    main()