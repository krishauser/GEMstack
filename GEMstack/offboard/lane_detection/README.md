# Lane Detection and 3D Reconstruction with CLRerNet

This project detects 2D lane lines from camera images using [CLRerNet](https://github.com/naver/clrernet) and projects them to 3D ground coordinates using Inverse Perspective Mapping (IPM). The final 3D lane points are saved in `.ply` and `.npy` formats for visualization and downstream tasks.

## Features

- Lane detection using MMDetection-compatible CLRerNet
- Image undistortion using camera intrinsics
- 2D to 3D projection using inverse camera model
- Per-image 2D visualization with overlayed lane predictions
- Exports `.ply` point cloud and `.npy` array of all 3D lanes

## Installation

First, clone this repository and install dependencies:

```bash
git clone https://github.com/naver/clrernet.git
cd clrernet
conda create -n mmlab python=3.10 -y
conda activate mmlab
pip install torch==2.0.1+cu117 torchvision==0.15.2+cu117 torchaudio==2.0.2 --index-url https://download.pytorch.org/whl/cu117
pip install -U openmim
mim install "mmcv>=2.0.0rc4,<2.2.0"
pip install mmdet
pip install -r requirements.txt
pip install "numpy<2"
conda install -c conda-forge cudatoolkit-dev
cd libs/models/layers/nms
pip install -e .
```

Then, make sure your project directory has the following structure:
download the checkpoints and put them under a folder called 'checkpoints'
```
your_project/
├── run_lane_detection.py          # The main lane detection script
├── clrernet/                      # The cloned CLRerNet repo
    └── checkpoints                # The checkpoint folder
├── image_data/
│   └── sample1.jpg                # Example input images
└── output/
    └── ...                        # Outputs will be written here
```

## Configuration

Ensure your `settings.py` file provides the following parameters for each camera type used:

```python
settings = {
    "calibration": {
        "front_right_camera": {
            "intrinsics": {
                "focal": [fx, fy],
                "center": [cx, cy],
                "skew": 0.0,
                "distort": [k1, k2, p1, p2, k3]
            },
            "extrinsics": {
                "rotation": [[...], [...], [...]],
                "position": [x, y, z]
            }
        },
        ...
    }
}
```

## Usage

Run the lane detection and projection pipeline:

```bash
python run_lane_detection.py \
    --image_folder image_data \
    --output_folder output \
    --output_file_prefix sample \
    --camera_type front_right \
    --config CLRerNet/configs/lane_detection.py \
    --checkpoint CLRerNet/checkpoints/lane_detection.pth \
    --device cuda:0
```

### Arguments

| Argument | Description |
|----------|-------------|
| `--image_folder` | Folder with input images |
| `--output_folder` | Output directory for saving results |
| `--output_file_prefix` | Prefix for output filenames |
| `--camera_type` | Camera ID: `fr`, `fl`, `rr`, `rl` |
| `--config` | Path to the CLRerNet config file |
| `--checkpoint` | Path to model checkpoint |
| `--device` | PyTorch device (e.g., `cuda:0`, `cpu`) |

## Output

- `output/image_name_with_detected_lanes.png`: Visualizations with 2D lane overlays
- `output/sample_all_lanes_3d.ply`: 3D point cloud of projected lane points
- `output/sample_all_lanes_3d.npy`: Numpy array of all 3D lane points

## Acknowledgments

- [CLRerNet](https://github.com/naver/clrernet)
- [OpenCV](https://opencv.org/)
- [Open3D](http://www.open3d.org/)


**Author**: Henry Yi  
**License**: MIT
