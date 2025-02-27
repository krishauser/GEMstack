# Data Capture

### Data Capture Script (`capture_ouster_oak.py`)

Usage:

Run roscore in a terminal
Run catkin_make in gem stack 
Run source /demo_ws/devel/setup.bash
Run roslaunch basic_launch sensor_init.launch to get all sensors running

To run script:
python3 capture_ouster_oak.py
To specify directory to save data, use --folder "path to save location" (default save folder is data)
To specify frequency of data capture, use --frequency put_frequency_in_hz_here (default is 2 hz)
To specify the what index the data should start being saved as, use --start_index desired_index_here (default is 1)


# GEMstack Offboard Calibration 

This section explains tools for offline calibration of LiDAR and camera sensors to the vehicle coordinate system on the GEM E4 platform. The calibration pipeline consists of three stages:

1. **LiDAR-to-Vehicle**  
2. **Camera-to-Vehicle**  
3. **LiDAR-to-Camera**  

---


## Calibration Pipeline

### 1. LiDAR-to-Vehicle Calibration (`lidar_to_vehicle.py`)
**Method**:  
- **Ground Plane Detection**:  
  1. Crop LiDAR points near ground (Z ∈ [-3, -2])  
  2. Use RANSAC to fit a plane to ground points  
  3. Compute vehicle height (`tz`) using plane equation and axel height offset (0.2794m)  
  4. Derive pitch (`rx`) and roll (`ry`) from plane normal vector  

- **Frontal Object Alignment**:  
  1. Crop LiDAR data to frontal area (X ∈ [0,20], Y ∈ [-1,1])  
  2. Fit line to object points for yaw (`rz`) estimation  
  3. Compute translation (`tx`, `ty`) using vehicle dimensions  

**Usage**:  

Our script assumes data is formated as: colorx.png, lidarx.npz, depthx.tif where x is some index number. Choose x depending on what data sample you want to use for calibration. 

python3 lidar_to_vehicle.py --data_path "path to data folder" --index INDEX_NUM

Use --vis flag for visualizations throughout the computation process


### 2. CAMERA-to-Vehicle Calibration (`camera_to_vehicle_manual.py`)
**Method**:  
  1. Capture camera intrinsics using camera_info.py (ROS node)  
  2. Manually select 4 matching points in RGB image and LiDAR cloud
  3. Solve PnP problem to compute camera extrinsics  

**Usage**:
  1. Get camera intrinsics:
    rosrun offboard\calibration\camera_info.py  # Prints intrinsic matrix
  2. Update camera_in in script with intrinsics
  3. Our script assumes data is formated as: colorx.png, lidarx.npz, depthx.tif where x is some index number. Choose x depending on what data sample you want to use for calibration. The script also reads the lidar_to_vehicle matrix from the gem_e4_ouster.yaml file so ensure that is up to date.
  4. Run calibration:
    python3 camera_to_vehicle_manual.py --data_path "path to data folder" --index INDEX_NUM --config "path to gem_e4_ouster.yaml"


### 3. LIDAR-to-CAMERA Calibration (`lidar_to_camera.py`)
**Method**:  
  1. Invert Camera-to-Vehicle matrix  
  2. Multiply with LiDAR-to-Vehicle matrix

**Usage**:

python3 lidar_to_camera.py   # Ensure T_lidar_vehicle and T_camera_vehicle matrices are updated


### Visualization Tools

**3D Alignment Check**:
 1. Use vis() function in scripts to view calibrated LiDAR/camera clouds
 2. Toggle VIS = True in lidar_to_vehicle.py for ground plane/object visualization
 3. Use test_transforms.py to visualize lidar point cloud on top of png image. Helps verify accuracy of lidar->camera.

**Projection Validation**:
 1. RGB image overlaid with transformed LiDAR points (Z-buffered)
 2. Frontal view comparison of camera and LiDAR data






### Assumption and Notes

1. The sensor data should be time-aligned.
2. The flat surface should be detectable in the Lidar scan
3. Magic Numbers:
    Axel height (0.2794m) from physical measurements
    Frontal crop areas based on vehicle dimensions
4. Validation: Use pyvista visualizations to verify alignment






