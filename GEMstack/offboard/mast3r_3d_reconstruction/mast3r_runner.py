import math
import gradio
import os
import numpy as np
import functools
import trimesh
import copy
from scipy.spatial.transform import Rotation
import tempfile
import shutil
import torch

from mast3r.cloud_opt.sparse_ga import sparse_global_alignment,anchor_depth_offsets,make_pts3d,show_reconstruction,forward_mast3r,prepare_canonical_data,condense_data,compute_min_spanning_tree,sparse_scene_optimizer
from mast3r.cloud_opt.tsdf_optimizer import TSDFPostProcess
from mast3r.image_pairs import make_pairs
from mast3r.retrieval.processor import Retriever

import mast3r.utils.path_to_dust3r  # noqa
from dust3r.utils.geometry import inv, geotrf  # noqa
from dust3r.utils.image import load_images
from dust3r.utils.device import to_numpy
from dust3r.viz import add_scene_cam, CAM_COLORS, OPENGL, pts3d_to_trimesh, cat_meshes
from dust3r.demo import get_args_parser as dust3r_get_args_parser
from dust3r.cloud_opt.base_opt import clean_pointcloud

import matplotlib.pyplot as pl
import open3d as o3d

import scipy.cluster.hierarchy as sch


def get_args_parser():
    parser = dust3r_get_args_parser()
    parser.add_argument('--share', action='store_true')
    parser.add_argument('--gradio_delete_cache', default=None, type=int,
                        help='age/frequency at which gradio removes the file. If >0, matching cache is purged')
    parser.add_argument('--retrieval_model', default=None, type=str, help="retrieval_model to be loaded")

    actions = parser._actions
    for action in actions:
        if action.dest == 'model_name':
            action.choices = ["MASt3R_ViTLarge_BaseDecoder_512_catmlpdpt_metric"]
    # change defaults
    parser.prog = 'mast3r demo'
    return parser

import pickle
import os
from pathlib import Path


class SparseGA():
    def __init__(self, img_paths, pairs_in, res_fine, anchors, canonical_paths=None):
        def fetch_img(im):
            def torgb(x): return (x[0].permute(1, 2, 0).numpy() * .5 + .5).clip(min=0., max=1.)
            for im1, im2 in pairs_in:
                if im1['instance'] == im:
                    return torgb(im1['img'])
                if im2['instance'] == im:
                    return torgb(im2['img'])
        self.canonical_paths = canonical_paths
        self.img_paths = img_paths
        self.imgs = [fetch_img(img) for img in img_paths]
        self.intrinsics = res_fine['intrinsics']
        self.cam2w = res_fine['cam2w']
        self.depthmaps = res_fine['depthmaps']
        self.pts3d = res_fine['pts3d']
        self.pts3d_colors = []
        self.working_device = self.cam2w.device
        for i in range(len(self.imgs)):
            im = self.imgs[i]
            x, y = anchors[i][0][..., :2].detach().cpu().numpy().T
            self.pts3d_colors.append(im[y, x])
            assert self.pts3d_colors[-1].shape == self.pts3d[i].shape
        self.n_imgs = len(self.imgs)

    def get_focals(self):
        return torch.tensor([ff[0, 0] for ff in self.intrinsics]).to(self.working_device)

    def get_principal_points(self):
        return torch.stack([ff[:2, -1] for ff in self.intrinsics]).to(self.working_device)

    def get_im_poses(self):
        return self.cam2w

    def get_sparse_pts3d(self):
        return self.pts3d

    def get_dense_pts3d(self, clean_depth=True, subsample=8):
        assert self.canonical_paths, 'cache_path is required for dense 3d points'
        device = self.cam2w.device
        confs = []
        base_focals = []
        anchors = {}
        for i, canon_path in enumerate(self.canonical_paths):
            (canon, canon2, conf), focal = torch.load(canon_path, map_location=device)
            confs.append(conf)
            base_focals.append(focal)

            H, W = conf.shape
            pixels = torch.from_numpy(np.mgrid[:W, :H].T.reshape(-1, 2)).float().to(device)
            idxs, offsets = anchor_depth_offsets(canon2, {i: (pixels, None)}, subsample=subsample)
            anchors[i] = (pixels, idxs[i], offsets[i])

        # densify sparse depthmaps
        pts3d, depthmaps = make_pts3d(anchors, self.intrinsics, self.cam2w, [
                                      d.ravel() for d in self.depthmaps], base_focals=base_focals, ret_depth=True)
        self.confs_dense = confs
        if clean_depth:
            confs = clean_pointcloud(confs, self.intrinsics, inv(self.cam2w), depthmaps, pts3d)
            self.confs_dense_clean = confs

        self.pts3d_dense = pts3d
        self.depthmaps_dense = depthmaps
        return pts3d, depthmaps, confs

    def get_pts3d_colors(self):
        return self.pts3d_colors

    def get_depthmaps(self):
        return self.depthmaps

    def get_masks(self):
        return [slice(None, None) for _ in range(len(self.imgs))]

    def show(self, show_cams=True):
        pts3d, _, confs = self.get_dense_pts3d()
        show_reconstruction(self.imgs, self.intrinsics if show_cams else None, self.cam2w,
                            [p.clip(min=-50, max=50) for p in pts3d],
                            masks=[c > 1 for c in confs])


def convert_dust3r_pairs_naming(imgs, pairs_in):
    for pair_id in range(len(pairs_in)):
        for i in range(2):
            pairs_in[pair_id][i]['instance'] = imgs[pairs_in[pair_id][i]['idx']]
    return pairs_in


def sparse_global_alignment(imgs, pairs_in, cache_path, model, subsample=8, desc_conf='desc_conf',
                            kinematic_mode='hclust-ward', device='cuda', dtype=torch.float32, shared_intrinsics=False, **kw):
    """ Sparse alignment with MASt3R
        imgs: list of image paths
        cache_path: path where to dump temporary files (str)

        lr1, niter1: learning rate and #iterations for coarse global alignment (3D matching)
        lr2, niter2: learning rate and #iterations for refinement (2D reproj error)

        lora_depth: smart dimensionality reduction with depthmaps
    """
    # Convert pair naming convention from dust3r to mast3r
    pairs_in = convert_dust3r_pairs_naming(imgs, pairs_in)
    # forward pass
    pairs, cache_path = forward_mast3r(pairs_in, model,
                                       cache_path=cache_path, subsample=subsample,
                                       desc_conf=desc_conf, device=device)

    # extract canonical pointmaps
    tmp_pairs, pairwise_scores, canonical_views, canonical_paths, preds_21 = \
        prepare_canonical_data(imgs, pairs, subsample, cache_path=cache_path, mode='avg-angle', device=device)

    # smartly combine all useful data
    imsizes, pps, base_focals, core_depth, anchors, corres, corres2d, preds_21 = \
        condense_data(imgs, tmp_pairs, canonical_views, preds_21, dtype)

    # Build kinematic chain
    if kinematic_mode == 'mst':
        # compute minimal spanning tree
        mst = compute_min_spanning_tree(pairwise_scores)

    elif kinematic_mode.startswith('hclust'):
        mode, linkage = kinematic_mode.split('-')

        # Convert the affinity matrix to a distance matrix (if needed)
        n_patches = (imsizes // subsample).prod(dim=1)
        max_n_corres = 3 * torch.minimum(n_patches[:,None], n_patches[None,:])
        pws = (pairwise_scores.clone() / max_n_corres).clip(max=1)
        pws.fill_diagonal_(1)
        pws = to_numpy(pws)
        distance_matrix = np.where(pws, 1 - pws, 2)

        # Compute the condensed distance matrix
        condensed_distance_matrix = sch.distance.squareform(distance_matrix)

        # Perform hierarchical clustering using the linkage method
        Z = sch.linkage(condensed_distance_matrix, method=linkage)
        # dendrogram = sch.dendrogram(Z)

        tree = np.eye(len(imgs))
        new_to_old_nodes = {i:i for i in range(len(imgs))}
        for i, (a, b) in enumerate(Z[:,:2].astype(int)):
            # given two nodes to be merged, we choose which one is the best representant
            a = new_to_old_nodes[a]
            b = new_to_old_nodes[b]
            tree[a,b] = tree[b,a] = 1
            best = a if pws[a].sum() > pws[b].sum() else b
            new_to_old_nodes[len(imgs)+i] = best
            pws[best] = np.maximum(pws[a], pws[b]) # update the node

        pairwise_scores = torch.from_numpy(tree) # this output just gives 1s for connected edges and zeros for other, i.e. no scores or priority
        mst = compute_min_spanning_tree(pairwise_scores)

    else:
        raise ValueError(f'bad {kinematic_mode=}')

    # remove all edges not in the spanning tree?
    # min_spanning_tree = {(imgs[i],imgs[j]) for i,j in mst[1]}
    # tmp_pairs = {(a,b):v for (a,b),v in tmp_pairs.items() if {(a,b),(b,a)} & min_spanning_tree}

    imgs, res_coarse, res_fine = sparse_scene_optimizer(
        imgs, subsample, imsizes, pps, base_focals, core_depth, anchors, corres, corres2d, preds_21, canonical_paths, mst,
        shared_intrinsics=shared_intrinsics, cache_path=cache_path, device=device, dtype=dtype, **kw)

    scene = SparseGA(imgs, pairs_in, res_fine or res_coarse, anchors, canonical_paths)
    # scene.get_dense_pts3d()
    # path = Path("saved_data") / "scene.pkl"
    # with path.open("wb") as f:
    #     pickle.dump(scene, f)

    print("Current working directory:", os.getcwd())
    return scene

def _convert_scene_output_to_glb(outfile, imgs, pts3d, mask, focals, cams2world, cam_size=0.05,
                                 cam_color=None, as_pointcloud=False,
                                 transparent_cams=False, silent=False):
    assert len(pts3d) == len(mask) <= len(imgs) <= len(cams2world) == len(focals)
    pts3d = to_numpy(pts3d)
    imgs = to_numpy(imgs)
    focals = to_numpy(focals)
    cams2world = to_numpy(cams2world)

    scene = trimesh.Scene()

    # full pointcloud
    if as_pointcloud:
        pts = np.concatenate([p[m.ravel()] for p, m in zip(pts3d, mask)]).reshape(-1, 3)
        col = np.concatenate([p[m] for p, m in zip(imgs, mask)]).reshape(-1, 3)
        valid_msk = np.isfinite(pts.sum(axis=1))
        pct = trimesh.PointCloud(pts[valid_msk], colors=col[valid_msk])
        scene.add_geometry(pct)
    else:
        meshes = []
        for i in range(len(imgs)):
            pts3d_i = pts3d[i].reshape(imgs[i].shape)
            msk_i = mask[i] & np.isfinite(pts3d_i.sum(axis=-1))
            meshes.append(pts3d_to_trimesh(imgs[i], pts3d_i, msk_i))
        mesh = trimesh.Trimesh(**cat_meshes(meshes))
        scene.add_geometry(mesh)

    # add each camera
    for i, pose_c2w in enumerate(cams2world):
        if isinstance(cam_color, list):
            camera_edge_color = cam_color[i]
        else:
            camera_edge_color = cam_color or CAM_COLORS[i % len(CAM_COLORS)]
        add_scene_cam(scene, pose_c2w, camera_edge_color,
                      None if transparent_cams else imgs[i], focals[i],
                      imsize=imgs[i].shape[1::-1], screen_width=cam_size)

    rot = np.eye(4)
    rot[:3, :3] = Rotation.from_euler('y', np.deg2rad(180)).as_matrix()
    scene.apply_transform(np.linalg.inv(cams2world[0] @ OPENGL @ rot))
    if not silent:
        print('(exporting 3D scene to', outfile, ')')
    scene.export(file_obj=outfile)
    return outfile

def convert_scene_output_to_ply_impl(outfile, imgs, pts3d, mask, scale=1.0, apply_y_flip=False, silent=False):
    """
    Export a scaled and colored 3D point cloud to PLY format using Open3D.

    Args:
        outfile (str): Path to save the .ply file.
        imgs (list of np.ndarray): RGB images (H, W, 3) per view.
        pts3d (list of np.ndarray): 3D points per view, shape (H * W, 3).
        mask (list of np.ndarray): Boolean masks indicating valid points per view (H, W).
        scale (float): Scale factor to apply to the 3D points.
        silent (bool): If False, print export message.

    Returns:
        str: Output file path to the saved PLY.
    """
    # Convert per-image valid 3D points and corresponding colors
    all_pts = []
    all_colors = []

    for p, m, img in zip(pts3d, mask, imgs):
        pts = p[m.ravel()] * scale                      # shape (N_valid, 3)
        colors = img[m]                                 # shape (N_valid, 3)
        valid = np.isfinite(pts).all(axis=1)            # remove NaNs or Infs

        all_pts.append(pts[valid])
        all_colors.append(colors[valid])        # normalize RGB to [0, 1]

    # Concatenate across all views
    all_pts = np.concatenate(all_pts, axis=0)
    all_colors = np.concatenate(all_colors, axis=0)

    apply_y_flip = True
    if apply_y_flip:
        rot = Rotation.from_euler('y', np.deg2rad(180)).as_matrix()
        all_pts = all_pts @ rot.T  # apply rotation to all points
    # Create Open3D point cloud
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(all_pts)
    pcd.colors = o3d.utility.Vector3dVector(all_colors)
    # Save to .ply
    final_outfile = os.path.join(os.getcwd(), outfile)
    os.makedirs(os.path.dirname(final_outfile), exist_ok=True)
    o3d.io.write_point_cloud(final_outfile, pcd)
    if not silent:
        print(f"✅ Exported scaled point cloud to: {outfile}")

    return outfile

def convert_scene_output_to_ply(outfile, data, scale=1.0, apply_y_flip=False, min_conf_thr=1.5, clean=True, TSDF_thresh=0):
    imgs = to_numpy(data.imgs)
    if TSDF_thresh > 0:
        tsdf = TSDFPostProcess(data, TSDF_thresh=TSDF_thresh)
        dense_pts3d, _, confs = to_numpy(tsdf.get_dense_pts3d(clean_depth=clean))
        msk = to_numpy([c > min_conf_thr for c in confs])
    else :
        confs = data.confs_dense_clean if clean else data.confs_dense
        dense_pts3d = to_numpy(data.pts3d_dense)
        msk = to_numpy([c > min_conf_thr for c in confs])
    # 3D pointcloud from depthmap, poses and intrinsics
    return convert_scene_output_to_ply_impl(outfile, imgs, dense_pts3d, msk, scale=scale, apply_y_flip=apply_y_flip)

def get_3D_model_from_scene(silent, scene_state, min_conf_thr=2, as_pointcloud=False, mask_sky=False,
                            clean_depth=False, transparent_cams=False, cam_size=0.05, TSDF_thresh=0):
    """
    extract 3D_model (glb file) from a reconstructed scene
    """
    if scene_state is None:
        return None
    outfile = scene_state.outfile_name
    if outfile is None:
        return None

    # get optimized values from scene
    scene = scene_state.sparse_ga
    rgbimg = scene.imgs
    focals = scene.get_focals().cpu()
    cams2world = scene.get_im_poses().cpu()

    # 3D pointcloud from depthmap, poses and intrinsics
    if TSDF_thresh > 0:
        tsdf = TSDFPostProcess(scene, TSDF_thresh=TSDF_thresh)
        pts3d, _, confs = to_numpy(tsdf.get_dense_pts3d(clean_depth=clean_depth))
    else:
        pts3d, _, confs = to_numpy(scene.get_dense_pts3d(clean_depth=clean_depth))
    msk = to_numpy([c > min_conf_thr for c in confs])
    return _convert_scene_output_to_glb(outfile, rgbimg, pts3d, msk, focals, cams2world, as_pointcloud=as_pointcloud,
                                        transparent_cams=transparent_cams, cam_size=cam_size, silent=silent)

def sort_images_from_longest_endpoint(D_square, data_length):
    D_square = D_square.copy()
    # Find the two farthest points
    i, j = np.unravel_index(np.argmax(D_square), D_square.shape)
    start_idx = i  # or j — either works

    # Greedy traversal using the precomputed distance matrix
    N = data_length
    visited = np.zeros(N, dtype=bool)
    visited[start_idx] = True
    path = [start_idx]

    current_idx = start_idx
    for _ in range(N - 1):
        dists = D_square[current_idx]
        dists[visited] = np.inf  # Ignore visited
        next_idx = np.argmin(dists)
        path.append(next_idx)
        visited[next_idx] = True
        current_idx = next_idx
    return path

def get_reconstructed_scene(outdir, gradio_delete_cache, model, retrieval_model, device, silent, image_size,
                            current_scene_state, filelist, optim_level, lr1, niter1, lr2, niter2, min_conf_thr,
                            matching_conf_thr, as_pointcloud, mask_sky, clean_depth,
                            scenegraph_type, winsize, win_cyclic, refid, TSDF_thresh, shared_intrinsics, **kw):
    """
    from a list of images, run mast3r inference, sparse global aligner.
    then run get_3D_model_from_scene
    """
    imgs = load_images(filelist, size=image_size, verbose=not silent)
    if len(imgs) == 1:
        imgs = [imgs[0], copy.deepcopy(imgs[0])]
        imgs[1]['idx'] = 1
        filelist = [filelist[0], filelist[0] + '_2']

    scene_graph_params = [scenegraph_type]
    if scenegraph_type in ["swin", "logwin"]:
        scene_graph_params.append(str(winsize))
    elif scenegraph_type == "oneref":
        scene_graph_params.append(str(refid))
    elif scenegraph_type == "retrieval":
        scene_graph_params.append(str(winsize))  # Na
        scene_graph_params.append(str(refid))  # k
    if scenegraph_type in ["swin", "logwin"] and not win_cyclic:
        scene_graph_params.append('noncyclic')
    scene_graph = '-'.join(scene_graph_params)

    sim_matrix = None
    if 'retrieval' in scenegraph_type:
        assert retrieval_model is not None
        retriever = Retriever(retrieval_model, backbone=model, device=device)
        with torch.no_grad():
            sim_matrix = retriever(filelist)

        # Cleanup
        del retriever
        torch.cuda.empty_cache()

    pairs = make_pairs(imgs, scene_graph=scene_graph, prefilter=None, symmetrize=True, sim_mat=sim_matrix)
    if optim_level == 'coarse':
        niter2 = 0
    # Sparse GA (forward mast3r -> matching -> 3D optim -> 2D refinement -> triangulation)
    if current_scene_state is not None and \
        not current_scene_state.should_delete and \
            current_scene_state.cache_dir is not None:
        cache_dir = current_scene_state.cache_dir
    elif gradio_delete_cache:
        cache_dir = tempfile.mkdtemp(suffix='_cache', dir=outdir)
    else:
        cache_dir = os.path.join(outdir, 'cache')
    os.makedirs(cache_dir, exist_ok=True)
    scene = sparse_global_alignment(filelist, pairs, cache_dir,
                                    model, lr1=lr1, niter1=niter1, lr2=lr2, niter2=niter2, device=device,
                                    opt_depth='depth' in optim_level, shared_intrinsics=shared_intrinsics,
                                    matching_conf_thr=matching_conf_thr, **kw)
    # if current_scene_state is not None and \
    #     not current_scene_state.should_delete and \
    #         current_scene_state.outfile_name is not None:
    #     outfile_name = current_scene_state.outfile_name
    # else:
    #     outfile_name = tempfile.mktemp(suffix='_scene.glb', dir=outdir)

    # scene_state = SparseGAState(scene, gradio_delete_cache, cache_dir, outfile_name)
    # outfile = get_3D_model_from_scene(silent, scene_state, min_conf_thr, as_pointcloud, mask_sky,
    #                                   clean_depth, transparent_cams, cam_size, TSDF_thresh)
    scene.get_dense_pts3d()
    # outfile = convert_scene_output_to_ply(outfile_name, scene, scale=1.0, apply_y_flip=False, min_conf_thr=min_conf_thr, clean=clean_depth)
    return scene

