# Data Capture

### Data Capture Script (`capture_ouster_oak.py`)

Set up on vehicle:

1. Run roscore in a terminal
2. Run catkin_make in gem stack
3. Run source /demo_ws/devel/setup.bash
4. Run roslaunch basic_launch sensor_init.launch to get all sensors running

Default script usage:

    python3 capture_ouster_oak.py

Additional Options:
1. To specify directory to save data, use --folder "path to save location" (default save folder is data)
2. To specify frequency of data capture, use --frequency put_frequency_in_hz_here (default is 2 hz)
3. To specify the what index the data should start being saved as, use --start_index desired_index_here (default is 1)


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

Our script assumes data is formated as: colorx.png, lidarx.npz, depthx.tif where x is some index number. x is chosen using the --index flag seen below. Set it based on what data sample you want to use for calibration. 

    python3 lidar_to_vehicle.py --data_path "path to data folder" --index INDEX_NUM

Optionally, use --vis flag for visualizations throughout the computation process


### 2. CAMERA-to-Vehicle Calibration (`camera_to_vehicle_manual.py`)
**Method**:  
  1. Capture camera intrinsics using camera_info.py (ROS node)  
  2. Manually select 8 matching points in RGB image and LiDAR cloud (can adjust variable to select more pairs)
  3. Solve PnP problem to compute camera extrinsics  

**Usage**:
  1. Get camera intrinsics:
    rosrun offboard\calibration\camera_info.py  # Prints intrinsic matrix
  2. Update camera_in in script with intrinsics
  3. Our script assumes data is formated as: colorx.png, lidarx.npz, depthx.tif where x is some index number. x is chosen using the --index flag seen below. Set it based on what data sample you want to use for calibration. 
  
  The script also reads the lidar_to_vehicle matrix from the gem_e4_ouster.yaml file so ensure that is up to date.
  
  4. Run calibration:
    
    python3 camera_to_vehicle_manual.py --data_path "path to data folder" --index INDEX_NUM --config "path to gem_e4_ouster.yaml"


### 3. LIDAR-to-CAMERA Calibration (`lidar_to_camera.py`)
**Method**:  
  1. Invert Camera-to-Vehicle matrix  
  2. Multiply with LiDAR-to-Vehicle matrix

**Usage**:
```
python3 lidar_to_camera.py   # Ensure T_lidar_vehicle and T_camera_vehicle matrices are updated
```

### Visualization Tools

**3D Alignment Check**:
 1. Use vis() function in scripts to view calibrated LiDAR/camera clouds
 2. Use --vis flag when running lidar_to_vehicle.py for ground plane/object visualization
 3. Use test_transforms.py to visualize the transformed lidar point cloud to camera frame on top of the corresponding png image. Helps verify accuracy of lidar->camera.

Usage of test_transforms.py:
```
python3 test_transforms.py --data_path "path to data folder" --index INDEX_NUM
```
Data path is the directory where lidar npz and color png files are located, index number is whichever lidar/png pair you want to evaluate. Ex. lidar1.npz color1.png where INDEX_NUM is 1 (--index 1)

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


### Results

Our resultant transformation matrices are the following:

T_camera_vehicle = np.array([[ 0.00349517, -0.03239524,  0.99946903, 1.75864913],
                             [-0.99996547,  0.00742285,  0.0037375, 0.01238124],
                             [-0.00753999, -0.99944757, -0.03236817, 1.54408419],
                             [0.000000000,  0.00000000,  0.00000000, 1.0]])

T_lidar_vehicle = np.array([[ 0.99941328,  0.02547416,  0.02289458, 1.1],
                            [-0.02530855,  0.99965159, -0.00749488, 0.03773044170906172],
                            [-0.02307753,  0.00691106,  0.99970979, 1.9525244316515322],
                            [0.000000000,  0.00000000,  0,00000000, 1.0]])

T_lidar_camera = np.array([
        [ 2.89748006e-02, -9.99580136e-01,  3.68439439e-05, -3.07300513e-02],
        [-9.49930618e-03, -3.12215512e-04, -9.99954834e-01, -3.86689354e-01],
        [ 9.99534999e-01,  2.89731321e-02, -9.50437214e-03, -6.71425124e-01],
        [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.00000000e+00]
    ])

We find that these matrices are very accurate and worked well with perceptions task of identifying pedestrains using camera and lidar. Perception team makes use of our lidar->camera matrix. Below is an image showcasing the effectiveness of our lidar->camera matrix. You can see the lidar pointcloud corresponds very well to the pixels in the image.

<img width="260" alt="Screenshot 2025-02-26 at 11 07 16 PM" src="https://github.com/user-attachments/assets/65322674-c715-47d4-bbef-880022ba1a5d" />
