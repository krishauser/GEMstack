# Point Cloud Colorizer from Multi-View Images

This script projects LiDAR point clouds into multi-view camera images and colorizes the 3D points using RGB values interpolated from the corresponding camera views. It supports multiple camera types and exports both raw and colorized point clouds in PLY format.

## Features

- Projects LiDAR point clouds into camera frames using calibrated extrinsic and intrinsic parameters
- Handles undistortion of input images using OpenCV
- Samples RGB values at subpixel precision from multi-view images
- Merges color information into a unified point cloud and saves as `.ply`
- Supports multiple camera types: front-right (`fr`), rear-right (`rr`), front-left (`fl`), rear-left (`rl`)

## Installation

Make sure you have the following dependencies installed:

```bash
pip install numpy opencv-python open3d
```
### Usage: 

```bash
python path/to/colorize_pointcloud.py \
    --folder_path /path/to/your/pointcloud_and_image_data \
    --output_path /path/to/save/output \
    --camera_types fr,rr
```
##### 1. Arguments
```bash
    --folder_path: Directory containing .b files (NumPy arrays) with point cloud data and corresponding images ({camera_type}_{index}.png)

    --output_path: Directory where the resulting .ply point clouds will be saved

    --camera_types: Comma-separated list of camera types to project and colorize from. Supported values: fr, rr, fl, rl
```

###### 2. Data Format Requirements
- Point cloud files: .b files that store N x 3 or N x 6 NumPy arrays of 3D points (with optional RGB). You can rename your saved pointcloud in NumPy arrays format to .b and then use this script.  
<br>
- Images: RGB .png images named like fr_001.png, rr_001.png, etc., located in the same directory as the .b files
<br>

###### 3. Calibration: 
Calibration parameters must be accessible via the settings module:

- Extrinsic:

    ```python
    settings.get("calibration.<camera_name>.extrinsics.rotation")
    settings.get("calibration.<camera_name>.extrinsics.position")
    ```
    ```python
    settings.get("calibration.top_lidar.rotation")
    settings.get("calibration.top_lidar.position")
    ```

- Intrinsic:
    ```python
    settings.get("calibration.<camera_name>.intrinsics.focal")
    settings.get("calibration.<camera_name>.intrinsics.center")
    settings.get("calibration.<camera_name>.intrinsics.distort")
    settings.get("calibration.<camera_name>.intrinsics.skew")
    ```
###### 4. Outputs
For each .b file:

    pure_pointcloud_<filename>.ply: raw point cloud without color

    colored_pointcloud_<filename>.ply: point cloud with interpolated RGB from visible camera views

##### Example
```python
python colorize_pointcloud.py \
    --folder_path data/session1 \
    --output_path data/colored_output \
    --camera_types fr,rr
```

## Notes
    Only the 3D points that are visible in at least one selected camera are assigned color.

    If no image exists for a particular view or projection fails, that camera is skipped.

    Requires accurate and synchronized calibration between LiDAR and camera systems.

    Author: Henry Yi
    License: MIT
