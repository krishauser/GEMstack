#!/usr/bin/env python3
import numpy as np
import open3d as o3d
import copy
import time
import argparse
import os
import glob
from scipy.spatial.transform import Rotation as R

def load_map(map_file):
    """Load a .ply map file."""
    print(f"Loading map from {map_file}...")
    try:
        map_pcd = o3d.io.read_point_cloud(map_file)
        points = np.asarray(map_pcd.points)
        print(f"Map stats:")
        print(f"  Number of points: {len(points)}")
        print(f"  Min: {np.min(points, axis=0)}")
        print(f"  Max: {np.max(points, axis=0)}")
        
        # Calculate map center for later use
        map_center = np.mean(points, axis=0)
        print(f"  Center: {map_center}")
        
        return map_pcd
    except Exception as e:
        print(f"Error loading map: {e}")
        return None, None

def load_lidar_scan(scan_file):
    """Load a .npz lidar scan file."""
    print(f"Loading lidar scan from {scan_file}...")
    try:
        data = np.load(scan_file, allow_pickle=True)
        
        # Use 'arr_0' as the key for your .npz files
        if 'arr_0' in data:
            points = data['arr_0']
        elif 'points' in data:
            points = data['points']
        else:
            points = data[list(data.keys())[0]]
        
        # Create point cloud from numpy array
        scan_pcd = o3d.geometry.PointCloud()
        scan_pcd.points = o3d.utility.Vector3dVector(points[:, :3])
        
        # Add intensity as colors if available (4th column)
        if points.shape[1] >= 4:
            intensities = points[:, 3]
            normalized_intensity = (intensities - np.min(intensities)) / (np.max(intensities) - np.min(intensities) + 1e-10)
            colors = np.zeros((points.shape[0], 3))
            colors[:, 0] = normalized_intensity  # Map intensity to red channel
            colors[:, 1] = normalized_intensity  # Map intensity to green channel
            colors[:, 2] = normalized_intensity  # Map intensity to blue channel
            scan_pcd.colors = o3d.utility.Vector3dVector(colors)
        
        print(f"Scan loaded with {len(np.asarray(scan_pcd.points))} points")
        
        # Print scan stats
        points = np.asarray(scan_pcd.points)
        print(f"Scan stats:")
        print(f"  Number of points: {len(points)}")
        print(f"  Min: {np.min(points, axis=0)}")
        print(f"  Max: {np.max(points, axis=0)}")
        
        return scan_pcd
    except Exception as e:
        print(f"Error loading scan: {e}")
        return None

def load_all_scans_from_folder(folder_path):
    """Load all .npz files from a folder as lidar scans."""
    print(f"Loading all lidar scans from {folder_path}...")
    
    # First validate that the path exists and is a directory
    if not os.path.isdir(folder_path):
        print(f"Error: {folder_path} is not a valid directory")
        return []
    
    # List all files in the directory (for debugging)
    all_files = os.listdir(folder_path)
    print(f"All files in directory: {len(all_files)}")
    
    # Get all .npz files using glob
    scan_files = glob.glob(os.path.join(folder_path, "*.npz"))
    
    if not scan_files:
        print(f"No .npz files found in {folder_path}")
        print(f"Files in directory: {all_files}")
        return []
    
    print(f"Found {len(scan_files)} scan files")
    scan_list = []
    
    for scan_file in scan_files:
        print(f"Loading {os.path.basename(scan_file)}...")
        scan = load_lidar_scan(scan_file)
        if scan is not None:
            scan_list.append(scan)
    
    print(f"Successfully loaded {len(scan_list)} scans")
    return scan_list

def remove_floor_ceiling(pcd, z_min=-0.5, z_max=2.5):
    """Remove floor and ceiling points to focus on walls and structural features."""
    points = np.asarray(pcd.points)
    mask = np.logical_and(points[:, 2] > z_min, points[:, 2] < z_max)
    
    filtered_pcd = o3d.geometry.PointCloud()
    filtered_pcd.points = o3d.utility.Vector3dVector(points[mask])
    if pcd.has_colors():
        filtered_pcd.colors = o3d.utility.Vector3dVector(np.asarray(pcd.colors)[mask])
    
    return filtered_pcd

def extract_structural_features(pcd, voxel_size):
    """Extract structural features like walls from point cloud."""
    # Downsample first
    pcd_down = pcd.voxel_down_sample(voxel_size)
    
    # Estimate normals
    pcd_down.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size*2, max_nn=30))
    
    # Find planar segments (walls, floors, etc.)
    planes = []
    rest = copy.deepcopy(pcd_down)
    for i in range(6):  # Extract top 6 planes
        if len(np.asarray(rest.points)) < 100:
            break
        plane_model, inliers = rest.segment_plane(distance_threshold=voxel_size*2,
                                                  ransac_n=3,
                                                  num_iterations=1000)
        if len(inliers) < 50:
            break
            
        plane = rest.select_by_index(inliers)
        planes.append(plane)
        rest = rest.select_by_index(inliers, invert=True)
    
    # Combine planes into a single point cloud
    structural_pcd = o3d.geometry.PointCloud()
    for plane in planes:
        structural_pcd += plane
    
    # If no planes were found, return the original downsampled point cloud
    if len(planes) == 0:
        return pcd_down
        
    return structural_pcd

def fuse_scans(scan_list, voxel_size=0.1):
    """Fuse multiple LiDAR scans into a single point cloud."""
    if not scan_list:
        return None
    
    # Start with the first scan
    fused_pcd = copy.deepcopy(scan_list[0])
    
    # Add points from subsequent scans
    for scan in scan_list[1:]:
        fused_pcd += scan
    
    # Downsample to maintain uniform density
    fused_pcd = fused_pcd.voxel_down_sample(voxel_size)
    
    # Print stats for fused scan
    fused_points = np.asarray(fused_pcd.points)
    print(f"Fused scan stats:")
    print(f"  Number of points: {len(fused_points)}")
    print(f"  Min: {np.min(fused_points, axis=0)}")
    print(f"  Max: {np.max(fused_points, axis=0)}")
    
    return fused_pcd

def prepare_scan_for_global_registration(scan_pcd, map_pcd, scale_ratio=None):
    """Improved scaling/translation using actual map bounds"""
    # Get map dimensions
    map_points = np.asarray(map_pcd.points)
    map_min = np.min(map_points, axis=0)
    map_max = np.max(map_points, axis=0)
    map_center = (map_min + map_max) / 2
    
    # Get scan dimensions
    scan_points = np.asarray(scan_pcd.points)
    scan_min = np.min(scan_points, axis=0)
    scan_max = np.max(scan_points, axis=0)
    
    # Calculate dynamic scale ratio
    if not scale_ratio:
        map_range = map_max - map_min
        scan_range = scan_max - scan_min
        scale_ratio = np.min(map_range / scan_range) * 0.8  # Use 80% of map size
    
    # Apply scaling and center alignment
    scaled_points = (scan_points - scan_min) * scale_ratio + map_min
    
    aligned_scan = o3d.geometry.PointCloud()
    aligned_scan.points = o3d.utility.Vector3dVector(scaled_points)
    
    print(f"Dynamic scaling ratio: {scale_ratio}")
    return aligned_scan




def preprocess_point_cloud(pcd, voxel_size, radius_normal=None, radius_feature=None):
    """Modified feature parameters for better matching"""
    pcd_down = pcd.voxel_down_sample(voxel_size)
    
    # Larger radii to capture building-scale features
    radius_normal = radius_normal or voxel_size * 5.0  # Increased from 2.0
    radius_feature = radius_feature or voxel_size * 10.0  # Increased from 5.0
    
    pcd_down.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=50))
    
    pcd_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        pcd_down,
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))
    
    return pcd_down, pcd_fpfh



def execute_global_registration(source_down, target_down, source_fpfh, target_fpfh, 
                               voxel_size, max_iterations=1000000):
    """Improved RANSAC registration with configurable iterations."""
    distance_threshold = voxel_size * 15  # Increased for initial alignment
    
    result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
        source_down, target_down, source_fpfh, target_fpfh, True,
        distance_threshold,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
        4,
        [o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
         o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(distance_threshold),
         o3d.pipelines.registration.CorrespondenceCheckerBasedOnNormal(0.5)],
        o3d.pipelines.registration.RANSACConvergenceCriteria(max_iterations, 500))
    
    return result



def execute_fast_global_registration(source_down, target_down, source_fpfh, target_fpfh, voxel_size):
    """Perform Fast Global Registration."""
    distance_threshold = voxel_size * 0.5
    print(f":: Apply fast global registration with distance threshold {distance_threshold:.3f}")
    
    try:
        result = o3d.pipelines.registration.registration_fgr_based_on_feature_matching(
            source_down, target_down, source_fpfh, target_fpfh,
            o3d.pipelines.registration.FastGlobalRegistrationOption(
                maximum_correspondence_distance=distance_threshold))
        return result
    except RuntimeError as e:
        print(f"Error in FGR: {e}")
        # Return dummy result with identity transformation in case of failure
        dummy_result = o3d.pipelines.registration.RegistrationResult()
        dummy_result.transformation = np.identity(4)
        dummy_result.fitness = 0.0
        dummy_result.inlier_rmse = 0.0
        dummy_result.correspondence_set = []
        return dummy_result

def multi_scale_icp(source, target, voxel_sizes=[2.0, 1.0, 0.5], max_iterations=[50, 30, 14], 
                   initial_transform=np.eye(4)):
    """Perform multi-scale ICP for robust alignment."""
    print("Running multi-scale ICP...")
    current_transform = initial_transform
    
    for i, (voxel_size, max_iter) in enumerate(zip(voxel_sizes, max_iterations)):
        print(f"ICP Scale {i+1}/{len(voxel_sizes)}: voxel_size={voxel_size}, max_iterations={max_iter}")
        
        # Downsample based on current voxel size
        source_down = source.voxel_down_sample(voxel_size)
        target_down = target.voxel_down_sample(voxel_size)
        
        # Estimate normals if not already computed
        source_down.estimate_normals(
            o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size*2, max_nn=30))
        target_down.estimate_normals(
            o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size*2, max_nn=30))
        
        # Use appropriate distance threshold based on scale
        distance_threshold = max(0.5, voxel_size * 2)
        
        # Run ICP
        result = o3d.pipelines.registration.registration_icp(
            source_down, target_down, distance_threshold, current_transform,
            o3d.pipelines.registration.TransformationEstimationPointToPoint(),
            o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=max_iter))
        
        current_transform = result.transformation
        print(f"  Scale {i+1} result - Fitness: {result.fitness:.4f}, RMSE: {result.inlier_rmse:.4f}")
    
    return current_transform

def transform_to_pose(transformation_matrix):
    """Convert transformation matrix to position and orientation (RPY)."""
    # Extract translation
    x, y, z = transformation_matrix[:3, 3]
    
    # Extract rotation matrix and convert to Euler angles
    # Make a writable copy to avoid read-only array issues
    rotation_matrix = np.array(transformation_matrix[:3, :3], copy=True)
    r = R.from_matrix(rotation_matrix)
    roll, pitch, yaw = r.as_euler('xyz', degrees=True)
    
    return x, y, z, roll, pitch, yaw

def draw_registration_result(source, target, transformation):
    """Visualize the source and target point clouds after alignment."""
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    
    source_temp.paint_uniform_color([1, 0.706, 0])  # Orange for source
    target_temp.paint_uniform_color([0, 0.651, 0.929])  # Blue for target
    
    source_temp.transform(transformation)
    
    # Downsample for visualization if point clouds are too large
    if len(np.asarray(source_temp.points)) > 500000:
        source_temp = source_temp.voxel_down_sample(0.1)
    if len(np.asarray(target_temp.points)) > 500000:
        target_temp = target_temp.voxel_down_sample(0.1)
    vis = o3d.visualization.Visualizer()
    vis.create_window()
    vis.add_geometry(source_temp)
    vis.add_geometry(target_temp)
    vis.get_render_option().point_size = 2
    vis.run()
    # o3d.visualization.draw_geometries([source_temp, target_temp])

def main():
    parser = argparse.ArgumentParser(description='Global registration for LiDAR localization')
    parser.add_argument('--map', required=True, help='Path to .ply map file')
    parser.add_argument('--scan-folder', help='Folder containing .npz lidar scan files')
    parser.add_argument('--scans', nargs='+', help='Paths to .npz lidar scan files')
    parser.add_argument('--output', default='global_registration_result.txt', 
                        help='Output file for transformation matrix')
    parser.add_argument('--method', choices=['ransac', 'fgr', 'both'], default='both',
                        help='Global registration method to use')
    parser.add_argument('--voxel-size', type=float, default=1.0,
                        help='Voxel size for downsampling (for global map)')
    parser.add_argument('--scan-voxel-size', type=float, default=0.5,
                        help='Voxel size for scan downsampling')
    parser.add_argument('--visualize', action='store_true',
                        help='Visualize the alignment result')
    parser.add_argument('--map-scale-ratio', type=float, default=1.0,
                        help='Scale ratio to apply to scan to match map scale')
    parser.add_argument('--icp-refine', action='store_true', default=True,
                        help='Refine alignment with ICP after global registration')
    parser.add_argument('--structural-features', action='store_true', default=True,
                        help='Extract structural features for better alignment')
    args = parser.parse_args()
    
    # Validate inputs
    if not args.scan_folder and not args.scans:
        print("Error: You must provide either --scan-folder or --scans")
        return
    
    # Load map
    map_pcd = load_map(args.map)
    print(map_pcd)
    if map_pcd is None:
        print("Failed to load map. Exiting.")
        return
    
    # Load scans
    scan_list = []
    if args.scan_folder:
        scan_list = load_all_scans_from_folder(args.scan_folder)
    elif args.scans:
        for scan_file in args.scans:
            scan = load_lidar_scan(scan_file)
            if scan is not None:
                scan_list.append(scan)
    
    if not scan_list:
        print("No valid scans loaded. Exiting.")
        return
    
    # Fuse scans if multiple
    print(f"Loaded {len(scan_list)} LiDAR scans.")
    if len(scan_list) > 1:
        print("Fusing scans...")
        scan_pcd = fuse_scans(scan_list, voxel_size=args.scan_voxel_size)
    else:
        scan_pcd = scan_list[0]
    
    # Scale and translate the scan to match map scale and center
    print(f"Preparing scan for global registration with scale ratio {args.map_scale_ratio}...")
    scaled_scan_pcd = prepare_scan_for_global_registration(
        scan_pcd, 
        map_pcd, 
        args.map_scale_ratio
    )
    

    normal_radius_factor = 2.0
    feature_radius_factor = 5.0

    print("Preprocessing map...")
    map_down, map_fpfh = preprocess_point_cloud(
        map_pcd, args.voxel_size, normal_radius_factor, feature_radius_factor)

    print("Preprocessing scan...")
    scan_down, scan_fpfh = preprocess_point_cloud(
        scaled_scan_pcd, args.scan_voxel_size, normal_radius_factor, feature_radius_factor)

    
    # Global registration
    transformation = np.identity(4)
    
    if args.method in ['ransac', 'both']:
        start_time = time.time()
        ransac_result = execute_global_registration(
            scan_down, map_down, scan_fpfh, map_fpfh, args.voxel_size)
        print(f"RANSAC registration took {time.time() - start_time:.3f} seconds")
        print(f"RANSAC result:")
        print(f"  Fitness: {ransac_result.fitness:.6f}")
        print(f"  Inlier RMSE: {ransac_result.inlier_rmse:.6f}")
        print(f"  Correspondence set size: {len(ransac_result.correspondence_set)}")
        
        if args.visualize:
            print("Visualizing RANSAC result...")
            draw_registration_result(scan_down, map_down, ransac_result.transformation)
        
        transformation = ransac_result.transformation
    
    if args.method in ['fgr', 'both']:
        start_time = time.time()
        fgr_result = execute_fast_global_registration(
            scan_down, map_down, scan_fpfh, map_fpfh, args.voxel_size)
        print(f"Fast Global Registration took {time.time() - start_time:.3f} seconds")
        print(f"FGR result:")
        print(f"  Fitness: {fgr_result.fitness:.6f}")
        print(f"  Inlier RMSE: {fgr_result.inlier_rmse:.6f}")
        print(f"  Correspondence set size: {len(fgr_result.correspondence_set)}")
        
        if args.visualize:
            print("Visualizing FGR result...")
            draw_registration_result(scan_down, map_down, fgr_result.transformation)
        
        if args.method == 'both':
            # Use the better result based on fitness
            if fgr_result.fitness > ransac_result.fitness:
                print("Using FGR result (better fitness).")
                transformation = fgr_result.transformation
            else:
                print("Using RANSAC result (better fitness).")
        else:
            transformation = fgr_result.transformation
    
    # Refine with ICP if requested
    if args.icp_refine:
        print("Refining with multi-scale ICP...")
        icp_transformation = multi_scale_icp(
            scan_down, map_down, 
            voxel_sizes=[2.0, 1.0, 0.5], 
            max_iterations=[100, 50, 30],
            initial_transform=transformation)
        
        final_transformation = icp_transformation
    else:
        final_transformation = transformation
    
    # # Extract position and orientation
    x, y, z, roll, pitch, yaw = transform_to_pose(final_transformation)
    
    # # We need to scale back the translation since we scaled the scan
    # # The rotation part doesn't need scaling
    # if args.map_scale_ratio != 1.0:
    #     # Need to account for the initial translation and scaling
    #     # The final transformation includes the initial translation
    #     # So we need to extract just the additional transformation from global registration
    #     x /= args.map_scale_ratio
    #     y /= args.map_scale_ratio
    #     z /= args.map_scale_ratio
        
    #     # Modify the final transformation matrix to undo the scaling effect on translation
    #     tmp = final_transformation
    #     final_transformation = np.zeros_like(tmp)
    #     final_transformation[:] = tmp
    #     final_transformation[:3, 3] /= args.map_scale_ratio
    
    # Print final results
    print(f"Estimated vehicle position: [{x:.2f}, {y:.2f}, {z:.2f}]")
    print(f"Estimated vehicle orientation (RPY, degrees): [{roll:.2f}, {pitch:.2f}, {yaw:.2f}]")
    
    # Save results
    with open(args.output, 'w') as f:
        f.write(f"# Global Registration Result\n")
        f.write(f"# Method: {args.method}\n")
        if args.method == 'both':
            f.write(f"# RANSAC Fitness: {ransac_result.fitness:.6f}\n")
            f.write(f"# FGR Fitness: {fgr_result.fitness:.6f}\n")
        f.write(f"# Final Fitness: {max(ransac_result.fitness, fgr_result.fitness):.6f}\n")
        # f.write(f"# Position: {x:.6f}, {y:.6f}, {z:.6f}\n")
        # f.write(f"# Orientation (roll, pitch, yaw degrees): {roll:.6f}, {pitch:.6f}, {yaw:.6f}\n")
        f.write("# Transformation matrix (4x4):\n")
        np.savetxt(f, final_transformation, fmt='%.6f')
    
    print(f"Results saved to {args.output}")
    
    # Final visualization
    if args.visualize:
        print("Visualizing final result...")
        # # First create an unscaled copy of the scan for visualization with unscaled map
        # unscaled_scan = copy.deepcopy(scan_pcd)
        # # Calculate the unscaled final transformation
        # unscaled_final_transform = np.copy(final_transformation)
        # unscaled_final_transform[:3, 3] /= args.map_scale_ratio
        # draw_registration_result(unscaled_scan, map_pcd, unscaled_final_transform)
        draw_registration_result(scan_down, map_down, final_transformation)

if __name__ == "__main__":
    main()
