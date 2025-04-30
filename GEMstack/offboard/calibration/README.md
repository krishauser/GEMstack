## Table of Contents
- [Pipeline](#Pipeline)
- [img2pc.py](#img2pcpy) - Camera-to-LiDAR extrinsic calibration
- [test_transforms.py](#test_transformspy) - Manual tuning of calibrations
- [capture_ouster_oak.py](#capture_ouster_oakpy) - Sensor data capture script
- [camera_info.py](#camera_infopy) - Intrinsic retrieval from hardware
- [get_intrinsic_by_chessboard.py](#get_intrinsic_by_chessboardpy) - Intrinsic calibration using chessboard
- [get_intrinsic_by_SfM.py](#get_intrinsic_by_sfmpy) - Intrinsic calibration using SfM
- [undistort_images.py](#undistort_imagespy) - Create rectified copies of distorted images
---

# Pipeline
**Data collection**

Use the `capture_ouster_oak.py` script to collect a series of scans combining both camera images and lidar pointclouds. The two sensors are not activated at the same time, so it is best to stay in place for a few seconds at each position to ensure you get aligned scans.

Some cameras may have calibrated hardware: use the `camera_info.py` script to extract their intrinsics if needed. An output containing all zeros means the hardware is not calibrated, and so you will need to calibrate yourself.

**Intrinsic Calibration**

There are two ways to calibrate the intrinsics, depending on what data you have.

To use the `get_intrinsic_by_chessboard.py` script, collect a series of images with a large chessboard using either the data collection scripts or a rosbag. Select images where the chessboard is at different points in the camera frame, different distances including filling the entire frame, and at different angles. The script detects internal corners where four squares meet, so the extreme edge of the chessboard does not need to be in frame.

To use the `get_intrinsic_by_SfM.py` script, prepare a set of images recorded from the same camera going through a continuous movement, and follow [this](#get_intrinsic_by_sfmpy)

The `undistort_images.py` script can then be used to rectify a set of images using the calibrated intrinsics to evaluate or use in other applications.

**Extrinsic Calibration**

The `img2pc.py` file contains the main part of the extrinsic calibration process. Select a synchronized camera image and lidar pointcloud to align, ideally containing features that are easy to detect in both, such as boards or signs with corners. Alignment can be done with 4 feature pairs(must be coplanar) or 6+ points. The first screen will ask you to select points on the image, and will close on its own once *n_features* points are selected. The second screen will ask you to select points in the point cloud, and will need to be closed manually once exactly *n_features* points are selected, or it will prompt you again. The extrinsic matrices will then be displayed, and if an *out_path* is provided they will also be saved.

The `test_transforms.py` file can then be used to manually fine-tune the calculated intrinsics. Use the sliders to change the translation and rotation to project the lidar points onto the image more accurately.

## img2pc.py
**Camera-to-LiDAR Extrinsic Calibration Tool**

Compute camera extrinsic parameters by manually selecting corresponding features in an image and point cloud.

**Note**: On the img prompt, click n points and the window closed itself. On the pc prompt, right click n points and close the window manually.

### Usage
```bash
python3 img2pc.py \
    --img_path IMG_PATH \
    --pc_path PC_PATH \
    --img_intrinsic_path IMG_INTRINSICS_PATH \
    [--pc_transform_path PC_TRANSFORM_PATH] \
    [--out_path OUT_PATH] \
    [--n_features N_FEATURES]
```

#### Parameters
| Parameter | Description | Format | Required | Default |
|-----------|-------------|--------|----------|---------|
| `--img_path` | Input image path | .png | Yes | - |
| `--pc_path` | Point cloud path | .npz | Yes | - |
| `--img_intrinsic_path` | Camera intrinsics file | .yaml | Yes | - |
| `--pc_transform_path` | LiDAR extrinsic transform | .yaml | No | Identity |
| `--n_features` | Manual feature points | int | No | 8 |
| `--out_path` | Output extrinsic path | .yaml | No | None |
| `--no_undistort` | to undistort | - | - | False |
| `--show` | visualize reprojection | - | - | False |

*`--no_undistort True` is rare because it's almost sure that extrinsic solving performs better after undistortion*

### Example
```bash
root='/mnt/GEMstack'
python3 img2pc.py \
    --img_path $root/data/calib1/img/fl/fl16.png \
    --pc_path $root/data/calib1/pc/ouster16.npz \
    --pc_transform_path $root/GEMstack/knowledge/calibration/gem_e4_ouster.yaml \
    --img_intrinsic_path $root/GEMstack/knowledge/calibration/gem_e4_oak_in.yaml \
    --n_features 4 \
    --out_path $root/GEMstack/knowledge/calibration/gem_e4_oak.yaml
```

## test_transforms.py
Script used for testing and fine-tuning extrinsics.

### Usage
```bash
python3 test_transforms.py \
    --img_path IMG_PATH \
    --lidar_path LIDAR_PATH \
    --lidar_transform_path LIDAR_TRANSFORM_PATH \
    --camera_transform_path CAMERA_TRANSFORM_PATH \
    --img_intrinsics_path IMG_INTRINSICS_PATH \
    [--out_path OUT_PATH] \
    [--undistort]
```

#### Parameters
| Parameter | Description | Format | Required | Default |
|-----------|-------------|--------|----------|---------|
| `--img_path` | Input image path | .png | Yes | - |
| `--lidar_path` | Point cloud path | .npz | Yes | - |
| `--lidar_transform_path` | LiDAR extrinsic transform | .yaml | Yes | - |
| `--camera_transform_path` | Camera extrinsic transform | .yaml | Yes | - |
| `--img_intrinsics_path` | Camera intrinsics file | .yaml | Yes | - |
| `--out_path` | Output extrinsic path | .yaml | No | None |
| `--undistort` | Flag for using distortion coefficients | - | - | - |

### Example
```bash
root='/mnt/GEMstack'
python3 test_transforms.py \
    --img_path $root/data/fl16.png \
    --lidar_path $root/data/ouster16.npz \
    --lidar_transform_path $root/GEMstack/knowledge/calibration/gem_e4_ouster.yaml \
    --camera_transform_path $root/GEMstack/knowledge/calibration/gem_e4_fl.yaml \
    --img_intrinsics_path $root/GEMstack/knowledge/calibration/gem_e4_fl_in.yaml \
    --out_path $root/GEMstack/knowledge/calibration/gem_e4_fl.yaml \
    --undistort
```

## capture_ouster_oak.py
Collect synchronized data from all initialized sensors on the e4 vehicle. Requires oak camera to be running.

### Usage
```bash
python3 capture_ouster_oak.py [FOLDER] [START_INDEX]
```

#### Parameters
| Parameter | Description | Format | Required | Default |
|-----------|-------------|--------|----------|---------|
| `FOLDER` | Output data folder path | directory | No | data |
| `START_INDEX` | Start index for scans | int | No | 1 |

## camera_info.py
Read camera info from ROS publishers to capture intrinsics. If the hardware is not calibrated, the subscriber will receive all zeros. Intrinsics will be saved in a file titled by camera.

### Usage
```bash
python3 camera_info.py [FOLDER]
```

#### Parameters
| Parameter | Description | Format | Required | Default |
|-----------|-------------|--------|----------|---------|
| `FOLDER` | Output data folder path | directory | No | data |

## get_intrinsic_by_chessboard.py
Chessboard-based Intrinsic Calibration

Compute camera intrinsic parameters using multiple images of a chessboard pattern.

### Usage
```bash
python3 get_intrinsic_by_chessboard.py \
    --img_folder_path IMG_FOLDER_PATH \
    [--camera_name CAMERA_NAME] \
    [--out_path OUT_PATH] \
    [--board_width BOARD_WIDTH] \
    [--board_height BOARD_HEIGHT]
```

#### Parameters
| Parameter | Description | Format | Required | Default |
|-----------|-------------|--------|----------|---------|
| `--img_folder_path` | Input image folder path | directory | Yes | - |
| `--camera_name` | Camera prefix used to identify images | string | No | empty string |
| `--out_path` | Output extrinsic path | .yaml | No | None |
| `--board_width` | Chessboard width (squares - 1) | int | No | 8 |
| `--board_height` | Chessboard height (squares - 1) | int | No | 6 |


## get_intrinsic_by_SfM.py 

Compute camera intrinsic parameters using Structure-from-Motion on a sequence of images.

### Usage
```bash
python3 intrinsic_calibration_chessboard.py \
    --input_imgs INPUT_IMGS [INPUT_IMGS ...] \
    --workspace WORKSPACE \
    [--out_file OUT_FILE]
```
### Parameters
| Parameter | Description | Required |
|-----------|-------------|----------|
| `--input_imgs` | Input images (glob pattern) | Yes |
| `--workspace` | Temporary directory path (default `/tmp/colmap_tmp`) | No |
| `--out_file` | Output .yaml path | No |

*note:`--workspace` allows you to save running time for continuing/redoing a previous job. you can clean it up after. check [colmap](https://colmap.github.io/) for more infomation*
### Example
```bash
root='/mnt/GEMstack'
python3 intrinsic_calibration_chessboard.py \
    --input_imgs data/fl/images/0000[0-8][147].png \
    --workspace /tmp/SfM_intrinsic_fl \
    --out_file $root/GEMstack/knowledge/calibration/camera_intrinsics2/gem_e4_fl_in.yaml
```

## undistort_images.py
Script to remove distortion from images.

### Usage
```bash
python3 undistort_images.py \
    --img_intrinsics_path IMG_INTRINSICS_PATH \
    --img_folder_path IMG_FOLDER_PATH \
    --camera_name CAMERA_NAME 
```

#### Parameters
| Parameter | Description | Format | Required | Default |
|-----------|-------------|--------|----------|---------|
| `--img_intrinsics_path` | Camera intrinsics file | .yaml | Yes | - |
| `--img_folder_path` | Input image folder path | directory | Yes | - |
| `--camera_name` | Camera prefix used to identify images | string | No | empty string |

### Example
```bash
root='/mnt/GEMstack'
python3 undistort_images.py \
    --img_intrinsics_path $root/GEMstack/knowledge/calibration/gem_e4_fr_in.yaml \
    --img_folder_path $root/data \
    --camera_name fr
```

# Credit
Michal Juscinski, [Renjie Sun](https://github.com/rjsun06), Dev Sakaria


