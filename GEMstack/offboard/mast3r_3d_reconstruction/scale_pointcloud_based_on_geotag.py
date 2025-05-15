from contextlib import nullcontext
from itertools import combinations
from pathlib import Path
import numpy as np
from mast3r.mast3r.model import AsymmetricMASt3R
from mast3r.mast3r.utils.misc import hash_md5
from pyproj import Transformer
import numpy as np
from PIL import Image
from PIL.ExifTags import TAGS, GPSTAGS
import os
import torch

import numpy as np
from itertools import combinations
import random

import pickle
import pandas as pd
import tempfile

import numpy as np
import open3d as o3d
from scipy.spatial.transform import Rotation

from mast3r.mast3r.demo import get_args_parser, main_demo
import argparse
from mast3r_runner import get_reconstructed_scene, convert_scene_output_to_ply


def todevice(batch, device, callback=None, non_blocking=False):
    ''' Transfer some variables to another device (i.e. GPU, CPU:torch, CPU:numpy).

    batch: list, tuple, dict of tensors or other things
    device: pytorch device or 'numpy'
    callback: function that would be called on every sub-elements.
    '''
    if callback:
        batch = callback(batch)

    if isinstance(batch, dict):
        return {k: todevice(v, device) for k, v in batch.items()}

    if isinstance(batch, (tuple, list)):
        return type(batch)(todevice(x, device) for x in batch)

    x = batch
    if device == 'numpy':
        if isinstance(x, torch.Tensor):
            x = x.detach().cpu().numpy()
    elif x is not None:
        if isinstance(x, np.ndarray):
            x = torch.from_numpy(x)
        if torch.is_tensor(x):
            x = x.to(device, non_blocking=non_blocking)
    return x


def to_numpy(x): return todevice(x, 'numpy')
def to_cpu(x): return todevice(x, 'cpu')
def to_cuda(x): return todevice(x, 'cuda')

def dms_to_decimal(d, m, s, ref):
    dd = d + m / 60 + s / 3600
    return -dd if ref in ['S', 'W'] else dd

def get_gps_from_exif(image_path):
    img = Image.open(image_path)
    exif = img._getexif()
    if not exif:
        return None
    gps_info = {}
    for tag, value in exif.items():
        decoded = TAGS.get(tag)
        # print(decoded)
        if decoded == "GPSInfo":
            for t in value:
                sub_decoded = GPSTAGS.get(t)
                # print(sub_decoded)
                gps_info[sub_decoded] = value[t]
    
    return gps_info

def parse_gps_info(image_path):
    gps_info = get_gps_from_exif(image_path)
    # Latitude
    lat_ref = gps_info.get('GPSLatitudeRef', 'N')
    lat_dms = gps_info.get('GPSLatitude')
    lat = dms_to_decimal(lat_dms[0], lat_dms[1], lat_dms[2], lat_ref)

    # Longitude
    lon_ref = gps_info.get('GPSLongitudeRef', 'E')
    lon_dms = gps_info.get('GPSLongitude')
    lon = dms_to_decimal(lon_dms[0], lon_dms[1], lon_dms[2], lon_ref)

    # Altitude
    alt = gps_info.get('GPSAltitude', 0.0)
    alt_ref = gps_info.get('GPSAltitudeRef', b'\x00')
    if isinstance(alt_ref, bytes) and alt_ref == b'\x01':
        alt = -alt  # below sea level

    # Timestamp (optional)
    timestamp = gps_info.get('GPSTimeStamp', (0.0, 0.0, 0.0))
    timestamp = f'{int(timestamp[0])}:{int(timestamp[1])}:{int(timestamp[2])} UTC'
    datestamp = gps_info.get('GPSDateStamp', '0000:00:00')
    
    return {
        'latitude': float(lat),
        'longitude': float(lon),
        'altitude': float(alt),
        'timestamp': timestamp,
        'date': datestamp
    }

def gps_to_xyz(gps_lookup):
    xyz_lookup = {}
    transformer = Transformer.from_crs("EPSG:4979", "EPSG:32616", always_xy=True)  # includes altitude
    for image_name, (lat, lon, alt) in gps_lookup.items():
        xyz = transformer.transform(lon, lat, alt)
        xyz_lookup[image_name] = xyz
    return xyz_lookup

def estimate_3d_scale_from_gps(camera_centers, gps_xyz, camera_image_names, min_dist_threshold=1.0):
    """
    Estimate scale factor between MASt3r's camera centers and GPS 3D coordinates.
    
    Inputs:
        camera_centers: (N, 3) array in MASt3r units (arbitrary scale)
        gps_xyz: (N, 3) array in meters [x, y, z] from lat/lon/alt
    Returns:
        scale: estimated meters-per-unit scale factor
    """
    assert camera_centers.shape == np.ones((len(gps_xyz), len(list(gps_xyz.items())[0][1]))).shape
    sfm_dists = []
    gps_dists = []

    for i, j in combinations(range(len(camera_centers)), 2):
        file_name_i = camera_image_names[i]
        file_name_j = camera_image_names[j]
        d_sfm = np.linalg.norm(camera_centers[i] - camera_centers[j])
        # print(file_name_i, file_name_j)
        # print(gps_xyz[file_name_i], gps_xyz[file_name_j])
        d_gps = np.linalg.norm(np.array(gps_xyz[file_name_i]) - np.array(gps_xyz[file_name_j]))

        if d_gps > min_dist_threshold:
            sfm_dists.append(d_sfm)
            gps_dists.append(d_gps)

    sfm_dists = np.array(sfm_dists)
    gps_dists = np.array(gps_dists)

    if len(gps_dists) == 0:
        raise ValueError("Not enough valid GPS pairs for scale estimation.")

    scale = np.median(gps_dists / sfm_dists)
    return scale, sfm_dists, gps_dists


def estimate_scale_ransac(camera_centers, gps_xyz, camera_image_names, threshold=0.05, iterations=1000, min_dist=1.0):
    scales = []
    pairs = []

    # Build all valid distance pairs
    for i, j in combinations(range(len(camera_centers)), 2):
        file_name_i = camera_image_names[i]
        file_name_j = camera_image_names[j]
        d_sfm = np.linalg.norm(camera_centers[i] - camera_centers[j])
        d_gps = np.linalg.norm(np.array(gps_xyz[file_name_i]) - np.array(gps_xyz[file_name_j]))
        if d_gps > min_dist:
            scale = d_gps / d_sfm
            scales.append(scale)
            pairs.append((i, j))

    scales = np.array(scales)
    pairs = np.array(pairs)

    best_inliers = []
    best_scale = None

    for _ in range(iterations):
        # Sample one index randomly
        idx = random.randint(0, len(scales) - 1)
        candidate_scale = scales[idx]

        # Compute residuals for all
        residuals = np.abs(scales - candidate_scale)

        # Inliers: within the threshold
        inliers = np.where(residuals < threshold)[0]

        if len(inliers) > len(best_inliers):
            best_inliers = inliers
            best_scale = np.median(scales[inliers])

    return best_scale, len(best_inliers), len(scales)

def extract_image_names(image_paths):
    return [path.split('/')[-1] for path in image_paths]



def collect_gps_data(data_folder):
    records = []
    for fname in sorted(os.listdir(data_folder)):
        if fname.lower().endswith(('.jpg', '.jpeg', '.png')):
            full_path = os.path.join(data_folder, fname)
            try:
                gps_data = parse_gps_info(full_path)
                if gps_data:
                    parsed_gps_data = gps_data
                    records.append({
                        'filename': fname,
                        'latitude': parsed_gps_data['latitude'],
                        'longitude': parsed_gps_data['longitude'],
                        'altitude': parsed_gps_data['altitude'],
                        'timestamp': parsed_gps_data['timestamp'],
                        'date': parsed_gps_data['date']
                    })
            except Exception as e:
                print(f"Warning: {fname} - {e}")
    return pd.DataFrame(records)


def run_mast3r(args):
    model = AsymmetricMASt3R.from_pretrained(args.weights_path).to(args.device)
    chkpt_tag = hash_md5(args.weights_path)

    def get_context(tmp_dir):
        return tempfile.TemporaryDirectory(suffix='_mast3r_context') if tmp_dir is None \
            else nullcontext(tmp_dir)
    with get_context(args.tmp_dir) as tmpdirname:
        cache_path = os.path.join(tmpdirname, chkpt_tag)
        os.makedirs(cache_path, exist_ok=True)
        inputfiles = [os.path.join(args.folder_path, f) for f in os.listdir(args.folder_path) if f.lower().endswith(('.jpg', '.jpeg', '.png'))]
        scene_state, outfile = get_reconstructed_scene(tmpdirname, False, model,
                                  args.retrieval_model, args.device, args.silent, args.image_size, None, inputfiles, args.optim_level, args.lr1, args.niter1, args.lr2, args.niter2, args.min_conf_thr, args.matching_conf_thr,
                                  True, False, args.clean_depth,
                                  args.scenegraph_type, args.winsize, args.win_cyclic, args.refid, 0, args.shared_intrinsics)
    return scene_state, outfile

def add_parse_args(parser):
    parser.add_argument('--folder_path', type=str, required=True)
    parser.add_argument('--output_path', type=str, required=True)
    parser.add_argument('--scale_method', type=str, required=True)

    parser.add_argument('--weights_path', type=str, required=True, help='Path to the core mast3rweights file')
    parser.add_argument('--retrieval_model', type=str, required=True, help='Retrieval model weights path that is used to make image pairs')
    parser.add_argument('--device', type=str, required=True, help='Device to run the model on')
    parser.add_argument('--silent', type=bool, required=True, help='Whether to run the model silently')
    parser.add_argument('--image_size', type=int, required=True, help='Image size')
    parser.add_argument('--optim_level', type=int, required=True, help='Optimization level')
    parser.add_argument('--lr1', type=float, required=True, help='Learning rate for the first refinement iteration stage')
    parser.add_argument('--niter1', type=int, required=True, help='Number of iterations for the first refinement iteration stage')
    parser.add_argument('--lr2', type=float, required=True, help='Learning rate for the second refinement iteration stage')
    parser.add_argument('--niter2', type=int, required=True, help='Number of iterations for the second refinement iteration stage')
    parser.add_argument('--min_conf_thr', type=float, required=True, help='Minimum confidence threshold')
    parser.add_argument('--matching_conf_thr', type=float, required=True, help='Matching confidence threshold')
    # parser.add_argument('--as_pointcloud', type=bool, required=True, help='Whether to output a pointcloud')
    # parser.add_argument('--mask_sky', type=bool, required=True, help='Whether to mask the sky')
    parser.add_argument('--clean_depth', type=bool, required=True, help='Whether to clean the depth')
    parser.add_argument('--transparent_cams', type=bool, required=True, help='Whether to make the cameras transparent')
    parser.add_argument('--cam_size', type=float, required=True, help='Camera size')
    parser.add_argument('--scenegraph_type', type=str, required=True, help='Scenegraph type')
    parser.add_argument('--winsize', type=int, required=True, help='Window size for sliding window pair making scenegraph_type')
    parser.add_argument('--win_cyclic', type=bool, required=True, help='Whether to use a cyclic sliding window')
    parser.add_argument('--refid', type=str, required=True, help='Reference image for retrieval')
    # parser.add_argument('--TSDF_thresh', type=float, required=True, help='TSDF refinement threshold')
    parser.add_argument('--shared_intrinsics', type=bool, required=True, help='Whether to use a shared intrinsics model')
    
    
    return parser

def scale_pointcloud_based_on_geotag():
    parser = argparse.ArgumentParser()

    # Add known args
    parser.add_argument('--scene_path', type=str, help='This is the path to the scene file', required=False)

    # Parse known and unknown args
    args, unknown = parser.parse_known_args()
    
    if not args.scene_path:
        args_parser = get_args_parser()
        args_parser = add_parse_args(args_parser)
        args = args_parser.parse_args()
        scene, outfile = run_mast3r(args)
        scene.get_dense_pts3d()
        data = scene
    else:
        args_parser = argparse.ArgumentParser()
        args_parser = add_parse_args(args_parser)
        args = args_parser.parse_args()
    
        with open(args.scene_path, 'rb') as f:
                    data = pickle.load(f)
    
    
    cam2w = data.get_im_poses()
    camera_centers = cam2w[:, :3, 3]  # Extract translation component from [R|t]
    df = collect_gps_data(args.folder_path)
    image_gps_data = df.to_numpy()
    gps_lookup = {
        row[0]: [float(row[1]), float(row[2]), float(row[3])]
        for row in image_gps_data
    }
    image_names = extract_image_names(data.img_paths)
    xyz_lookup = gps_to_xyz(gps_lookup)
    for method in ['ransac', 'median']:
        if method == 'ransac':
            scale, sfm_dists, gps_dists = estimate_scale_ransac(camera_centers, xyz_lookup, image_names)
        else:
            scale, sfm_dists, gps_dists = estimate_3d_scale_from_gps(camera_centers, xyz_lookup, image_names)
        print(f"Estimated scale: {scale}")
    convert_scene_output_to_ply(args.output_path, data, scale=scale, apply_y_flip=False)

if __name__ == "__main__":
    scale_pointcloud_based_on_geotag()