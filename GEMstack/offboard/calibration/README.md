# GEMstack Offboard Calibration 

This repository contains tools for offline calibration of LiDAR and camera sensors to the vehicle coordinate system on the GEM E4 platform. The calibration pipeline consists of three stages:

1. **LiDAR-to-Vehicle**  
2. **Camera-to-Vehicle**  
3. **LiDAR-to-Camera**  

---


## Calibration Pipeline

### 1. LiDAR-to-Vehicle Calibration (`make_gem_e4_ouster_v2.py`)
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

python3 make_gem_e4_ouster_v2.py      # Edit LiDAR data paths in script


### 2. CAMERA-to-Vehicle Calibration (`camera_to_vehicle_manual.py`)
**Method**:  
  1. Capture camera intrinsics using camera_info.py (ROS node)  
  2. Manually select 4 matching points in RGB image and LiDAR cloud
  3. Solve PnP problem to compute camera extrinsics  

**Usage**:
  1. Get camera intrinsics:
    rosrun offboard\calibration\camera_info.py  # Prints intrinsic matrix
  2. Update camera_in in script with intrinsics
  3. Run calibration:
    python3 camera_to_vehicle_manual.py


### 3. LIDAR-to-CAMERA Calibration (`lidar_to_camera.py`)
**Method**:  
  1. Invert Camera-to-Vehicle matrix  
  2. Multiply with LiDAR-to-Vehicle matrix

**Usage**:

python3 lidar_to_camera.py   # Ensure T_lidar_vehicle and T_camera_vehicle matrices are updated


### Visualization Tools

**3D Alignment Check**:
 1. Use vis() function in scripts to view calibrated LiDAR/camera clouds
 2. Toggle VIS = True in make_gem_e4_ouster_v2.py for ground plane/object visualization

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






