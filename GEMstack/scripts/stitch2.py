import rosbag
import numpy as np
import open3d as o3d
import math
import copy
from scipy.spatial.transform import Rotation
from sensor_msgs import point_cloud2
import pyproj

# Define the coordinate transformations
wgs84 = pyproj.CRS('EPSG:4326')  # WGS84 latitude/longitude
utm16N = pyproj.CRS('EPSG:32616')  # UTM zone 16N
transformer = pyproj.Transformer.from_crs(wgs84, utm16N, always_xy=True)

def interpolate_position(gnss_times, gnss_data, target_time):
    """Interpolate position using GNSS data"""
    idx = np.searchsorted(gnss_times, target_time)
    if idx == 0:
        return gnss_data[0]
    if idx == len(gnss_times):
        return gnss_data[-1]
    
    t0, t1 = gnss_times[idx-1], gnss_times[idx]
    p0 = gnss_data[idx-1]
    p1 = gnss_data[idx]
    
    alpha = (target_time - t0) / (t1 - t0)
    
    class InterpolatedGNSS:
        pass
    
    msg = InterpolatedGNSS()
    msg.latitude = p0.latitude + alpha * (p1.latitude - p0.latitude)
    msg.longitude = p0.longitude + alpha * (p1.longitude - p0.longitude)
    msg.height = p0.height + alpha * (p1.height - p0.height)
    msg.roll = p0.roll + alpha * (p1.roll - p0.roll)
    msg.pitch = p0.pitch + alpha * (p1.pitch - p0.pitch)
    msg.heading = p0.heading + alpha * (p1.heading - p0.heading)
    
    return msg


def create_transformation_matrix(gnss_msg):
    """Create 4x4 transformation matrix from GNSS data using UTM coordinates"""
    # Convert lat/lon from radians to degrees
    lon_deg = math.degrees(gnss_msg.longitude)
    lat_deg = math.degrees(gnss_msg.latitude)
    
    try:
        easting, northing = transformer.transform(lon_deg, lat_deg)
    except Exception as e:
        print(f"Error in UTM transformation: {e}")
        return np.eye(4)
    
    # Store first position as origin
    if not hasattr(create_transformation_matrix, "origin"):
        create_transformation_matrix.origin = (easting, northing, gnss_msg.height)
    
    # Calculate position relative to origin in meters
    dx = easting - create_transformation_matrix.origin[0]
    dy = northing - create_transformation_matrix.origin[1]
    dz = gnss_msg.height - create_transformation_matrix.origin[2]

    # Base transformation to align coordinate systems
    base_rotation = np.array([
        [0, -1, 0],   # x forward
        [1, 0, 0],  # y right
        [0, 0, 1]    # z up
    ])
    
    # Convert heading to radians and create rotation matrix
    # Only using heading, setting pitch and roll to 0
    heading_rad = math.radians(gnss_msg.heading)
    r = Rotation.from_euler('z', -heading_rad, degrees=False)
    rotation_matrix = r.as_matrix()

    # Combine the rotations
    final_rotation = rotation_matrix @ base_rotation

    # Create 4x4 transformation matrix
    transform = np.eye(4)
    transform[:3, :3] = final_rotation
    transform[:3, 3] = [dx, dy, dz]
    
    return transform

def preprocess_point_cloud(pcd, voxel_size):
    """Preprocess point cloud for registration"""
    # Voxel downsampling
    pcd_down = pcd.voxel_down_sample(voxel_size=voxel_size)
    if pcd_down is None or len(pcd_down.points) == 0:
        return None
        
    # Remove outliers
    cl, ind = pcd_down.remove_radius_outlier(nb_points=16, radius=0.5)
    if cl is None or len(cl.points) == 0:
        return None
        
    # Estimate normals
    cl.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 2, max_nn=30))
    
    return cl

def register_point_clouds(source, target, initial_transform=np.eye(4), voxel_size=0.1):
    """Register two point clouds using ICP"""
    source_down = preprocess_point_cloud(source, voxel_size)
    target_down = preprocess_point_cloud(target, voxel_size)
    
    if source_down is None or target_down is None:
        return initial_transform
    
    result = o3d.pipelines.registration.registration_icp(
        source_down, target_down, voxel_size * 2, initial_transform,
        o3d.pipelines.registration.TransformationEstimationPointToPlane(),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=50))
    
    return result.transformation

print("Opening bag file...")
bag = rosbag.Bag('fr+gnss+lidar.bag')

# Collect GNSS messages
print("Collecting GNSS data...")
gnss_data = []
gnss_times = []

for topic, msg, t in bag.read_messages(topics=['/septentrio_gnss/insnavgeod']):
    gnss_times.append(t.to_sec())
    gnss_data.append(msg)

gnss_times = np.array(gnss_times)
print(f"Collected {len(gnss_data)} GNSS messages")

print("\nFirst few GNSS messages (converted):")
for i in range(min(5, len(gnss_data))):
    msg = gnss_data[i]
    print(f"\nGNSS Message {i}:")
    print(f"Latitude: {math.degrees(msg.latitude)} degrees (converted from {msg.latitude} rad)")
    print(f"Longitude: {math.degrees(msg.longitude)} degrees (converted from {msg.longitude} rad)")
    print(f"Height: {msg.height} meters")
    print(f"Roll: {msg.roll} rad")
    print(f"Pitch: {msg.pitch} rad")
    print(f"Heading: {msg.heading} degrees")


# First pass: Collect and preprocess all scans with initial UTM transformation
print("Processing LiDAR scans...")
scans = []
scan_count = 0

for topic, msg, t in bag.read_messages(topics=['/ouster/points']):
    # Get interpolated GNSS data
    image_time = t.to_sec()
    gnss_msg = interpolate_position(gnss_times, gnss_data, image_time)
    
    # Extract points
    pc_data = point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
    points = np.array(list(pc_data))
    
    if len(points) == 0:
        continue
    
    # Create point cloud
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    
    # Preprocess point cloud
    processed_pcd = preprocess_point_cloud(pcd, voxel_size=0.1)
    if processed_pcd is None:
        continue
    
    # Get initial transformation from GNSS in UTM coordinates
    utm_transform = create_transformation_matrix(gnss_msg)
    
    scans.append((image_time, processed_pcd, utm_transform))
    scan_count += 1
    if scan_count % 50 == 0:  # Changed from 10 to 50 to reduce output
        print(f"Processed {scan_count} scans")

print(f"Total scans processed: {scan_count}")

if len(scans) == 0:
    print("No valid scans collected!")
    bag.close()
    exit()

# Second pass: Progressive scan registration
print("Performing scan registration...")
registered_scans = []

# Start with the first scan
registered_scans.append((scans[0][1], scans[0][2]))
print(f"First scan points: {len(scans[0][1].points)}")

# Register subsequent scans
for i in range(1, len(scans)):
    print(f"Registering scan {i}/{len(scans)-1}...")
    
    _, current_scan, current_utm = scans[i]
    print(f"Current scan points: {len(current_scan.points)}")
    
    # Get initial alignment from UTM
    initial_transform = np.linalg.inv(scans[i-1][2]) @ current_utm
    
    # Perform ICP with previous scan
    refined_transform = register_point_clouds(
        current_scan, 
        registered_scans[-1][0],
        initial_transform=initial_transform,
        voxel_size=0.1
    )
    
    # Update global transform
    scan_global_transform = registered_scans[-1][1] @ refined_transform
    
    # Store registered scan
    registered_scans.append((current_scan, scan_global_transform))

    # ... (previous imports and functions remain the same) ...

# Combine registered scans
print("Combining registered scans...")
combined_pcd = o3d.geometry.PointCloud()

# Get the first GNSS message for absolute UTM coordinates
first_gnss = gnss_data[0]
lon_deg = math.degrees(first_gnss.longitude)
lat_deg = math.degrees(first_gnss.latitude)
print(f"First GNSS message (converted): lat={lat_deg}, lon={lon_deg}")

try:
    first_easting, first_northing = transformer.transform(lon_deg, lat_deg)
    print(f"First point UTM: E={first_easting}, N={first_northing}")
except Exception as e:
    print(f"Error converting first point to UTM: {e}")
    first_easting, first_northing = 0, 0

all_points = []
for i, (scan, transform) in enumerate(registered_scans):
    print(f"\nProcessing scan {i}...")
    
    # Transform scan to global coordinate frame
    scan_transformed = copy.deepcopy(scan)
    
    # Apply registration transform
    scan_transformed.transform(transform)
    
    # Convert points to numpy array for easier manipulation
    points = np.asarray(scan_transformed.points)
    
    # Check for invalid points
    valid_mask = ~np.any(np.isnan(points) | np.isinf(points), axis=1)
    if not np.all(valid_mask):
        print(f"Removing {np.sum(~valid_mask)} invalid points from scan {i}")
        points = points[valid_mask]
    
    if len(points) > 0:
        # Add UTM offset without using transform
        points = points + np.array([first_easting, first_northing, first_gnss.height])
        
        # Check for invalid points after offset
        valid_mask = ~np.any(np.isnan(points) | np.isinf(points), axis=1)
        points = points[valid_mask]
        
        if len(points) > 0:
            all_points.append(points)
            print(f"Added {len(points)} valid points from scan {i}")

# Combine all points
if all_points:
    combined_points = np.vstack(all_points)
    print(f"\nTotal combined points: {len(combined_points)}")
    
    # Create final point cloud
    final_pcd = o3d.geometry.PointCloud()
    final_pcd.points = o3d.utility.Vector3dVector(combined_points)
    
    # Optional: Downsample to reduce size
    final_pcd = final_pcd.voxel_down_sample(voxel_size=0.1)
    
    # Optional: Remove statistical outliers
    final_pcd, _ = final_pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
    
    print(f"Final point cloud size after processing: {len(final_pcd.points)}")
    
    # Save result
    print("\nSaving point cloud...")
    try:
        o3d.io.write_point_cloud("utm16N_registered_lidar.ply", final_pcd)
        np.savetxt("utm16N_registered_lidar.xyz", np.asarray(final_pcd.points))
        print("Successfully saved point clouds")
    except Exception as e:
        print(f"Error saving point cloud: {e}")
else:
    print("No valid points to combine!")
