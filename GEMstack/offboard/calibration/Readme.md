# Calibration Usage

## For HW3
#### Folder Structure
```
data
└── hw3
    ├── camera_info
    ├── step1
    └── step2
```
- `camera_info`: Include camera intrinsic information.
- `step1`: All data related to step1 calibration.
- `step2`: All data related to step2 calibration.

#### HW3 Step 1 (Lidar -> Camera Calibration)
1. Manually extract corresponding feature points between 3D lidar point cloud and 2D image using `lidar_feature_extract.py` and `camera_feature_extract.py`. After extraction, scripts will output the .csv file including the coordinate of feature points. Make sure two .csv files of lidar and image including the same number of feature points. <br>
Note: You can select points by `shift + click` in Open3D: <br>
<img width="547" alt="image" src="https://github.com/krishauser/GEMstack/assets/22386566/fa69b4bf-9e9d-470e-a670-de65586d58c6">
2. Use `lidar_camera_calibration.py` to find optimized parameters of extrinsic matrix based on corresponding feature points extracted in previous step.
3. Use `open3D_verify_calibration.py` to verify the obtained extrinsic matrix. If it works, you can see point clouds well align with the image pixels on target object <br>
<img width="540" alt="image" src="https://github.com/krishauser/GEMstack/assets/22386566/23d48634-c9e2-4887-9ecf-d836619cf3c6">

#### HW3 Step 2 (Lidar -> Vehicle Calibration)
1. Use `lidar_vehicle_calibration.py` to select four corner points of the planar object manually, then press `esc` two times to get the transformation matrix. <br>
<img width="500" alt="image" src="https://github.com/krishauser/GEMstack/assets/22386566/bcdbfbac-ec13-4a44-966d-0931d26f2021">

#### HW3 Step 3
1. Use `check_target_lidar_range.py` to determine the lidar range we desire for agents to be detected.