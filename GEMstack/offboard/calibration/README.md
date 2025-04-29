## Table of Contents
- [img2pc.py](#img2pcpy) - Camera-to-LiDAR extrinsic calibration
- [get_intrinsic_by_chessboard.py](#get_intrinsic_by_chessboardpy) - Intrinsic calibration using chessboard
- [get_intrinsic_by_SfM.py](#get_intrinsic_by_sfmpy) - Intrinsic calibration using SfM
---

## img2pc.py
**Camera-to-LiDAR Extrinsic Calibration Tool**

Compute camera extrinsic parameters by manually selecting corresponding features in an image and point cloud.

### Usage
```bash
python3 img2pc.py \
    --img_path IMG_PATH \
    --pc_path PC_PATH \
    --img_intrinsics_path IMG_INTRINSICS_PATH \
    [--pc_transform_path PC_TRANSFORM_PATH] \
    [--out_path OUT_PATH] \
    [--n_features N_FEATURES]
```
#### Parameters
| Parameter | Description | Format | Required | Default |
|-----------|-------------|--------|----------|---------|
| `--img_path` | Input image path | .png | Yes | - |
| `--pc_path` | Point cloud path | .npz | Yes | - |
| `--img_intrinsics_path` | Camera intrinsics file | .yaml | Yes | - |
| `--pc_transform_path` | LiDAR extrinsic transform | .yaml | No | Identity |
| `--out_path` | Output extrinsic path | .yaml | No | None |
| `--n_features` | Manual feature points | int | No | 8 |

### Example
```bash
root='/mnt/GEMstack'
python3 img2pc.py \
    --img_path $root/data/calib1/img/fl/fl16.png \
    --pc_path $root/data/calib1/pc/ouster16.npz \
    --pc_transform_path $root/GEMstack/knowledge/calibration/gem_e4_ouster.yaml \
    --img_intrinsics_path $root/GEMstack/knowledge/calibration/gem_e4_oak_in.yaml \
    --n_features 4 \
    --out_path $root/GEMstack/knowledge/calibration/gem_e4_oak.yaml
```

## get_intrinsic_by_chessboard.py
Chessboard-based Intrinsic Calibration

Compute camera intrinsic parameters using multiple images of a chessboard pattern.

### Usage
```bash
python3 get_intrinsic_by_chessboard.py IMG_DIR WIDTH HEIGHT [OUTPUT_PATH]
```

### Parameters
| Parameter | Description | Required | 
|-----------|-------------|----------|
| `IMG_DIR` | Calibration images folder | Yes |
| `WIDTH` | Chessboard width (squares-1) | Yes |
| `HEIGHT` | Chessboard height (squares-1) | Yes |
| `OUTPUT_PATH` | Output .yaml path | No |


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
| `--workspace` | Temporary directory path | Yes |
| `--out_file` | Output .yaml path | No |

### Example
```bash
root='/mnt/GEMstack'
python3 intrinsic_calibration_chessboard.py \
    --input_imgs data/fl/images/0000[0-8][147].png \
    --workspace /tmp/SfM_intrinsic_fl \
    --out_file $root/GEMstack/knowledge/calibration/camera_intrinsics2/gem_e4_fl_in.yaml
```

# Credit
Michal Juscinski, [Renjie Sun](https://github.com/rjsun06), Dev Sakaria


