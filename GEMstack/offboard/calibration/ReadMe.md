

The vehicle frame is defined as:
- **Origin:** Rear axle center  
- **X-axis:** Forward  
- **Y-axis:** Left  
- **Z-axis:** Up


## Assignment Overview

- **Capture Paired Data:**  
  Acquire synchronized LiDAR scans and camera images using your ROS-based capture scripts.

- **Extract Camera Intrinsics:**  
  Use the `/oak/rgb/camera_info` topic to extract intrinsic data (`fx`, `fy`, `cx`, `cy`) and save them into a JSON file (e.g., `camera_intrinsics.json`).

- **Extrinsic Calibration:**  
  Using a calibration target (e.g., a checkerboard, foam cubes, or a calibration box), select corresponding points in the LiDAR and image data in order to compute the transform $$ T_{\text{toplidar}}^{\text{frontcamera}} $$ with scripts like **calibrate_extrinsics.py**.

- **Vehicle Frame Alignment:**  
  Determine the yaw offset between the LiDAR frame and the vehicle frame:
  - Use **compute_yaw_offset.py** to manually select calibration objects (e.g., foam cubes) from the LiDAR scan and provide their corresponding vehicle frame positions (x forward, y left).  
  - The computed yaw offset is then incorporated with any other measurements in your full calibration pipeline.

- **Sensor Fusion Visualization:**  
  Finally, use the **sensor_fusion.py** script to project the LiDAR points—transformed using the computed extrinsics and vehicle alignment—onto a camera image, verifying that the sensors have been accurately calibrated.

---
## Dependencies and Setup

### Required Packages

Install the following Python packages:
```bash
pip install numpy open3d opencv-python matplotlib argparse pyyaml
```

### ROS Environment

Ensure you have a working ROS (e.g., Noetic) setup with the following topics:
- `/oak/rgb/camera_info`
- `/oak/rgb/image_raw`
- `/ouster/points`

---

## Scripts and Their Functions

### capture_ouster_oak.py
- **Purpose:** Captures paired LiDAR scans and camera images.  
- **Output:** Stores images as `.png` and LiDAR scans as `.npz`.

### pick_lidar_points.py
- **Purpose:** Allows manual selection of feature points from the LiDAR scan for extrinsic calibration.

### calibrate_extrinsics.py
- **Purpose:** Computes the sensor-to-sensor transform $$ T_{\text{toplidar}}^{\text{frontcamera}} $$ using corresponding points between LiDAR and camera images.

### compute_yaw_offset.py
- **Purpose:**  
  - First, lets you manually crop or select calibration objects (typically foam cubes along the vehicle centerline) in the LiDAR scan.
  - Then, prompts you to enter the corresponding vehicle frame (x, y) positions.
  - Finally, computes the yaw offset (in degrees) between the LiDAR frame and the vehicle frame.
  
### sensor_fusion.py
- **Purpose:**  
  - Loads the extrinsic calibration (from **calibrate_extrinsics.py**) and the camera intrinsics.
  - Transforms LiDAR points into the camera frame.
  - Projects these points onto a camera image.
  - Visualizes the result to verify sensor alignment.

---

## How to Run

### Step 1: Capture the Calibration Dataset
- Run **capture_ouster_oak.py**:
  ```bash
  python3 capture_ouster_oak.py --output_dir /path/to/calibration_01
  ```
  This will save paired LiDAR and image data.

### Step 2: Extract Camera Intrinsics
- Use your ROS system to extract the intrinsic parameters (`fx`, `fy`, `cx`, `cy`) from the `/oak/rgb/camera_info` topic and save them as `camera_intrinsics.json`.

### Step 3: Select LiDAR Calibration Points
- Run **pick_lidar_points.py** to manually select key points from your LiDAR scan:
  ```bash
  python3 pick_lidar_points.py --lidar_npz /path/to/lidar_0.npz --output /path/to/lidar_cal_points.npy
  ```

### Step 4: Compute LiDAR-to-Camera Extrinsics
- Run **calibrate_extrinsics.py**:
  ```bash
  python3 calibrate_extrinsics.py \
      --lidar_points /path/to/lidar_cal_points.npy \
      --image /path/to/image_0.png \
      --intrinsics /path/to/camera_intrinsics.json \
      --output /path/to/extrinsics.npz
  ```
  Follow prompts to select corresponding points in the camera frame.

### Step 5: Compute Yaw Offset (Manual Method)
- Run **compute_yaw_offset.py**:
  ```bash
  python3 compute_yaw_offset.py --lidar_scan /path/to/lidar_0.npz
  ```
  - A window will open for manual selection ("Manual Selection").  
  - Use SHIFT+LEFT CLICK to select the calibration objects (foam cubes) along the vehicle centerline.  
  - After quitting the window, you will be prompted to enter the corresponding vehicle frame (x, y) positions (e.g., `5.5, 0`).  
  - The script computes and prints the yaw offset.

### Step 6: Visualize Sensor Fusion
- Run **sensor_fusion.py** to project LiDAR points onto a camera image:
  ```bash
  python3 sensor_fusion.py \
      --extrinsics /path/to/extrinsics.npz \
      --intrinsics /path/to/camera_intrinsics.json \
      --lidar_scan /path/to/lidar_cal_points.npy \
      --image /path/to/image_0.png
  ```
  Verify that the projected points align well with the image features.

### Step 7: Vehicle Frame Calibration (Absolute Calibration)
- Once you have the yaw offset and other measurements (e.g., ground plane height, rear axle height), run your absolute calibration script (e.g., **absolute_calibration.py**) to compute:
  - $$ T_{\text{toplidar}}^{\text{vehicle}} $$
  - $$ T_{\text{frontcamera}}^{\text{vehicle}} $$
- Save these calibrations in a YAML file (e.g., `gem_e4.yaml` in `GEMstack/knowledge/calibration/`).

