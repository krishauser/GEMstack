# MASt3R-Based 3D Scene Reconstruction with GPS-Based Scaling

This script performs 3D scene reconstruction from multi-view images using [MASt3R](https://github.com/naver/mast3r), then scales the reconstructed scene using GPS data embedded in the image EXIF metadata.

The scaled point cloud is saved in `.ply` format, optionally with TSDF filtering and depth cleaning.

---

## Features

- Runs MASt3R pipeline to reconstruct a 3D point cloud from a set of input images
- Extracts GPS metadata (latitude, longitude, altitude) from image EXIF data
- Converts GPS coordinates into 3D Cartesian space using `pyproj`
- Estimates real-world scale using:
  - **RANSAC-based robust ratio estimation**
  - **Median ratio estimation**
- Exports `.ply` point cloud with real-world scale

---

## Installation

Install the required packages (assuming `mast3r` is cloned and download the checkpoint files):

The simplest way to install is to use the mast3r.yml file, I exported. It would create a conda virtual environment named `mast3r`.

And then please use the following command to install faiss-gpu:
```bash
canda install -c pytorch -r faiss-gpu
```

Make sure you have:
- `open3d`
- `numpy`, `opencv-python`, `torch`, `Pillow`, `pyproj`, `pandas`, `scipy`

And that `mast3r` is correctly installed and importable.

---

## 🚀 Usage

### Basic usage (with reconstruction)

```bash
python scale_pointcloud_based_on_geotag.py \
    --weights path/to/mast3r/core/checkpoint \
    --retrieval_model path/to/mast3r/retrieval/checkpoint \
    --folder_path path/to/images \
    --output_path output/output_scene.ply \
    --scale_method ransac \
    --scenegraph_type retrieval \
    --winsize 20 \
    --refid 10
```

### Usage (with precomputed scene):
 to directly get scaled pointcloud based on Geo Location info tagged to the images

```bash
python scale_pointcloud_based_on_geotag.py \
    --scene_path path/to/scene.pkl \
    --folder_path path/to/images \
    --output_path output/output_scene.ply \
    --scale_method median
```

---

## 🧠 Arguments

### Required (basic run)

| Argument         | Description                                                                 |
|------------------|-----------------------------------------------------------------------------|
| `--folder_path`  | Path to folder containing input images (JPG, PNG)                           |
| `--output_path`  | Output file path for the scaled `.ply` point cloud                          |
| `--scale_method` | Method to estimate scale (`ransac` or `median`)                             |

### Optional

| Argument         | Description                                                                 |
|------------------|-----------------------------------------------------------------------------|
| `--scene_path`   | (Optional) Path to a precomputed MASt3R scene `.pkl` file                   |
| `--crs_from`     | Input coordinate system (default: `EPSG:4979` — lat/lon/alt)                |
| `--crs_to`       | Target coordinate system (default: `EPSG:32616` — UTM)                      |

### MASt3R-specific args (if no `--scene_path` is given)

You can also configure MASt3R parameters such as:

| Argument             | Description                                 |
|----------------------|---------------------------------------------|
| `--optim_level`      | Optimization stage (default: `refine+depth`)|
| `--lr1`, `--niter1`  | Learning rate and iteration count (stage 1)  (default: 0.07, 300)|
| `--lr2`, `--niter2`  | Learning rate and iteration count (stage 2)  (default: 0.01, 300)|
| `--min_conf_thr`     | Minimum confidence threshold (default: 1.5) |
| `--matching_conf_thr`| Optional Matching confidence threshold               |
| `--scenegraph_type`  | Graph type: `retrieval`, `swin`, `complete`, etc |
| `--winsize`          | Optional Window size for `swin` or `logwin`          |
| `--clean_depth`      | Clean depth maps (default: `True`)          |
| `--TSDF_thresh`      | Optional TSDF threshold for filtering       |
| `--refid`            | Optional Reference images. In `retrieval` scenegraph_type, this indicates the number of neighboring images      |

---

## Scale Estimation

The script computes the relative scale between reconstructed camera centers and GPS-based 3D coordinates. Two methods are supported:

- `ransac`: Robust estimate using inlier filtering
- `median`: Simple median ratio of pairwise distances

---

## Output

- A `.ply` point cloud in the specified `output_path`, scaled using estimated GPS-to-reconstruction ratio
- The point cloud preserves geometry from MASt3R and optionally applies depth/TSDF filtering

---

## Notes

- Input images **must contain valid EXIF GPS tags**.
- If no `--scene_path` is given, the full MASt3R pipeline will run (can be slow).
- You can reuse a `scene.pkl` from MASt3R by supplying `--scene_path`.

---

## 📚 Reference

- [MASt3R GitHub](https://github.com/naver/mast3r)
- [pyproj](https://pyproj4.github.io/pyproj/stable/)

---

**Author**: Henry Yi  
**License**: MIT
