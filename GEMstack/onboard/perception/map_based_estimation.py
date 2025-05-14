from dataclasses import replace
import math
from typing import List
from ...utils import settings
from ...mathutils import transforms
from ...state.vehicle import VehicleState,VehicleGearEnum
from ...state.physical_object import ObjectFrameEnum,ObjectPose,convert_xyhead
from ...knowledge.vehicle.geometry import front2steer,steer2front
from ...mathutils.signal import OnlineLowPassFilter
from ..interface.gem import GEMInterface
from ..component import Component
from ..interface.gem import GNSSReading

import numpy as np
import open3d as o3d
import copy
import utm
import time
import argparse
import os
import glob
from scipy.spatial.transform import Rotation as R

def load_map(map_file):
    """Load a .ply map file."""
    try:
        map_pcd = o3d.io.read_point_cloud(map_file)
        points = np.asarray(map_pcd.points)
        
        # Calculate map center for later use
        map_center = np.mean(points, axis=0)
        
        return map_pcd
    except Exception as e:
        print(f"Error loading map: {e}")
        return None, None

def load_lidar_scan(points):
    """Load a .npz lidar scan file."""
    try:
        # Create point cloud from numpy array
        scan_pcd = o3d.geometry.PointCloud()
        points = np.ascontiguousarray(points[:, :3], dtype=np.float64)
        scan_pcd.points = o3d.utility.Vector3dVector(points)

        
        # # Add intensity as colors if available (4th column)
        # if points.shape[1] >= 4:
        #     intensities = points[:, 3]
        #     normalized_intensity = (intensities - np.min(intensities)) / (np.max(intensities) - np.min(intensities) + 1e-10)
        #     colors = np.zeros((points.shape[0], 3))
        #     colors[:, 0] = normalized_intensity  # Map intensity to red channel
        #     colors[:, 1] = normalized_intensity  # Map intensity to green channel
        #     colors[:, 2] = normalized_intensity  # Map intensity to blue channel
        #     scan_pcd.colors = o3d.utility.Vector3dVector(colors)
        
        return scan_pcd
    except Exception as e:
        print(f"Error loading scan: {e}")
        return None

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
    
    try:
        result = o3d.pipelines.registration.registration_fast_based_on_feature_matching(
            source_down, target_down, source_fpfh, target_fpfh,
            o3d.pipelines.registration.FastGlobalRegistrationOption(
                maximum_correspondence_distance=distance_threshold))
        return result
    except RuntimeError as e:
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
    current_transform = initial_transform
    
    for i, (voxel_size, max_iter) in enumerate(zip(voxel_sizes, max_iterations)):
        
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



def transform_to_pose(transformation_matrix,origin):
    """Convert transformation matrix to position and orientation (RPY)."""
    # Extract translation
    x, y, z = transformation_matrix[:3, 3]
    
    # Extract rotation matrix and convert to Euler angles
    # Make a writable copy to avoid read-only array issues
    rotation_matrix = np.array(transformation_matrix[:3, :3], copy=True)
    r = R.from_matrix(rotation_matrix)
    roll, pitch, yaw = r.as_euler('xyz', degrees=True)
     
    lat_deg,lon_deg = utm.to_latlon(x, y, 16, "N")
    
    return lon_deg,lat_deg,z,roll,pitch,yaw

class MapBasedStateEstimator(Component):
    """Just looks at the GNSS reading to estimate the vehicle state"""
    def __init__(self, map_fn : str, vehicle_interface : GEMInterface):
        self.vehicle_interface = vehicle_interface
        if 'top_lidar' not in vehicle_interface.sensors():
            raise RuntimeError("GNSS sensor not available")
        vehicle_interface.subscribe_sensor('top_lidar',self.lidar_callback,np.ndarray)
        self.map_based_pose = None
        self.map_based_speed = None
        self.points = None
        self.map = load_map(map_fn)

        # TODO: Change these to be some form of variable
        self.map_scale_ratio = 1.0
        self.voxel_size = 0.5
        self.scan_voxel_size = 0.5
        self.normal_radius_factor = 2.0
        self.feature_radius_factor = 5.0

        self.map_down, self.map_fpfh = preprocess_point_cloud(
            self.map, self.voxel_size, self.normal_radius_factor, self.feature_radius_factor)

        if 'gnss' not in vehicle_interface.sensors():
            raise RuntimeError("GNSS sensor not available")
        vehicle_interface.subscribe_sensor('gnss',self.gnss_callback,GNSSReading)
        self.gnss_pose = None
        self.location = settings.get('vehicle.calibration.gnss_location')[:2]
        self.yaw_offset = settings.get('vehicle.calibration.gnss_yaw')
        self.speed_filter  = OnlineLowPassFilter(1.2, 30, 4)
        self.status = None
        self.transformation = None

    # Get GNSS information
    def gnss_callback(self, reading : GNSSReading):
        self.gnss_pose = reading.pose
        self.gnss_speed = reading.speed
        self.status = reading.status

    # Get lidar information
    def lidar_callback(self, reading : np.ndarray):
        self.points = reading
    
    def rate(self):
        return 1
    
    def state_outputs(self) -> List[str]:
        return ['vehicle']

    def healthy(self):
        return self.map_based_pose is not None

    def update(self) -> VehicleState:
        if self.points is None:
            return
        
        scan_time = self.vehicle_interface.time()

        # Load scans
        scan_pcd = load_lidar_scan(self.points)

        
        # # Scale and translate the scan to match map scale and center
        # scaled_scan_pcd = prepare_scan_for_global_registration(
        #     scan_pcd, 
        #     self.map, 
        #     self.map_scale_ratio
        # )

        # print("Prepare scan", self.vehicle_interface.time() - scan_time)
        
        scan_down, scan_fpfh = preprocess_point_cloud(
            scan_pcd, self.scan_voxel_size, self.normal_radius_factor, self.feature_radius_factor)

        
        # Global registration
        transformation = np.identity(4)
        
        # Fast Global Registration
        fgr_result = execute_fast_global_registration(
            scan_down, self.map_down, scan_fpfh, self.map_fpfh, self.voxel_size)
        
        # Use the better result based on fitness
        transformation = fgr_result.transformation
        
        # Refine with ICP
        icp_transformation = multi_scale_icp(
            scan_down, self.map_down, 
            voxel_sizes=[2.0, 1.0, 0.5], 
            max_iterations=[100, 50, 25],
            initial_transform=transformation)
        
        final_transformation = icp_transformation
        
        # Extract position and orientation
        lat,lon, *_ = utm.from_latlon(0,-90,16,'N')
        origin90w0n = [lat,lon,0]
        x, y, z, roll, pitch, yaw = transform_to_pose(final_transformation,origin90w0n)
        # TODO: Estimate speed
        if self.map_based_pose != None:
            translation = np.array([x - self.map_based_pose.x, y - self.map_based_pose.y, z - self.map_based_pose.z])
            self.map_based_speed = np.linalg.norm(translation) / (scan_time - self.map_based_pose.t)
        self.map_based_pose = ObjectPose(ObjectFrameEnum.GLOBAL, scan_time, x, y, z, yaw, pitch, roll)

        # # vehicle gnss heading (yaw) in radians
        # # vehicle x, y position in fixed local frame, in meters
        # # reference point is located at the center of GNSS antennas
        # localxy = transforms.rotate2d(self.location,-self.yaw_offset)
        # gnss_xyhead_inv = (-localxy[0],-localxy[1],-self.yaw_offset)
        # center_xyhead = self.gnss_pose.apply_xyhead(gnss_xyhead_inv)
        # vehicle_pose_global = replace(self.gnss_pose,
        #                               t=self.vehicle_interface.time(),
        #                               x=center_xyhead[0],
        #                               y=center_xyhead[1],
        #                               yaw=center_xyhead[2])

        readings = self.vehicle_interface.get_reading()
        raw = readings.to_state(self.map_based_pose)

        print("Time:", self.vehicle_interface.time() - scan_time)
        print(x, y, z, yaw)

        #filtering speed
        if self.map_based_speed != None:
            raw.v = self.map_based_speed
        else:
            raw.v = 0.0

        if self.gnss_pose is None:
            return
        #TODO: figure out what this status means
        #print("INS status",self.status)

        # vehicle gnss heading (yaw) in radians
        # vehicle x, y position in fixed local frame, in meters
        # reference point is located at the center of GNSS antennas
        localxy = transforms.rotate2d(self.location,-self.yaw_offset)
        gnss_xyhead_inv = (-localxy[0],-localxy[1],-self.yaw_offset)
        center_xyhead = self.gnss_pose.apply_xyhead(gnss_xyhead_inv)
        vehicle_pose_global = replace(self.gnss_pose,
                                      t=self.vehicle_interface.time(),
                                      x=center_xyhead[0],
                                      y=center_xyhead[1],
                                      yaw=center_xyhead[2])

        print(self.gnss_pose.x, self.gnss_pose.y, self.gnss_pose.z, self.gnss_pose.yaw)
        
        return raw