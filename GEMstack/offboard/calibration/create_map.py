import os
import glob
import numpy as np
import cv2
import open3d as o3d
import pyvista as pv

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

def make_open3d_pc(points: np.ndarray):
    """
    Convert Nx3 numpy array into an Open3D point cloud object.
    """
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    return pcd

def perform_icp(source_pc: o3d.geometry.PointCloud,
                target_pc: o3d.geometry.PointCloud):
    """
    Perform local ICP alignment of camera_pcd (source) to lidar_pcd (target).
    Returns a transformation 4x4 that maps camera -> lidar (or vice versa).
    """
    # Estimate normals for point-to-plane
    source_pc.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(
        radius=1.0, max_nn=30))
    target_pc.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(
        radius=1.0, max_nn=30))

    # Initial guess: identity or something better if you have one
    init_guess = np.eye(4)

    # Choose a threshold for inlier distance
    threshold = 0.5  # adjust based on your environment scale

    # Run ICP (point-to-plane or point-to-point)
    print("[ICP] Running ICP alignment ...")
    result = o3d.pipelines.registration.registration_icp(
        source_pc,
        target_pc,
        threshold,
        init_guess,
        estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPlane()
    )

    print("[ICP] Done. Fitness = %.4f, RMSE = %.4f" % (result.fitness, result.inlier_rmse))
    print("[ICP] Transformation:\n", result.transformation)
    return result

def main(source='ouster1', target='ouster2', folder='data'):
    source_path = os.path.join(folder, f"{source}.npz")
    target_path = os.path.join(folder, f"{target}.npz")

    # Check for existence of files
    if not os.path.exists(source_path):
        print("Invalid source path", source_path)
        exit(1)
    if not os.path.exists(target_path):
        print("Invalid target path", target_path)
        exit(1)

    # Load LiDAR
    source_points = load_lidar_points(source_path)
    target_points = load_lidar_points(target_path)
    source_points = source_points[~np.all(source_points == 0, axis=1)] # remove (0,0,0)'s
    target_points = target_points[~np.all(target_points == 0, axis=1)] # remove (0,0,0)'s
    source_pc = make_open3d_pc(source_points)
    target_pc = make_open3d_pc(target_points)

    # Run ICP
    result = perform_icp(source_pc, target_pc)
    # point_cloud = np.append(target_points, source_points @ result.transformation)

    # View result
    # cloud = pv.PolyData(point_cloud)
    target = pv.PolyData(target_points)
    source = pv.PolyData(source_points @ result.transformation)
    plotter = pv.Plotter(notebook=False)
    plotter.camera.position = (-20,0,20)
    plotter.camera.focal_point = (0,0,0)
    # plotter.add_mesh(cloud, color='lightblue', point_size=5, render_points_as_spheres=True)
    plotter.add_mesh(source, color='orange', point_size=5, render_points_as_spheres=True)
    plotter.add_mesh(target, color='navy', point_size=5, render_points_as_spheres=True)
    plotter.show()


if __name__ == "__main__":
    import sys
    if len(sys.argv) == 1:
        main()
    elif len(sys.argv) == 3:
        source = sys.argv[1]
        target = sys.argv[2]
        main(source, target)
    elif len(sys.argv) == 4:
        source = sys.argv[1]
        target = sys.argv[2]
        folder = sys.argv[3]
        main(source, target, folder)
    else:
        print("Usage: python3 createmap.py [source target] [folder]")
    