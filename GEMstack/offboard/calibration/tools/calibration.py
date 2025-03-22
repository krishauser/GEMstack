#%%
root = '/mnt/GEMstack'
#%%
import numpy as np
import pycolmap
import open3d as o3d
from pathlib import Path
from scipy.spatial.transform import Rotation as R

def pc_relative(pc0,pc1):
    #get relative translation and rotation from pc0 to pc1
    #in: Nx3 array, Nx3 array
    #out: 3 array, 3 array
    # Load point clouds
    source = o3d.geometry.PointCloud()
    source.points = o3d.utility.Vector3dVector(pc0)  
    target = o3d.geometry.PointCloud()
    target.points = o3d.utility.Vector3dVector(pc1)  

    # Run ICP
    reg_result = o3d.pipelines.registration.registration_icp(
        source, target, max_distance=0.1,
        estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint()
    )
    print("Transformation Matrix:\n", reg_result.transformation)
    return(reg_result.transformation)

def r_from_vector(direction):
    direction = np.array(direction)
    direction = direction / np.linalg.norm(direction)
    target = np.array([0, 1, 0])
    rotation_axis = np.cross(direction, target)
    if np.allclose(rotation_axis, 0):
        return R.from_quat([0, 0, 0, 1])
    else:
        angle = np.arccos(np.clip(np.dot(direction, target), -1.0, 1.0))
        rotation_axis = rotation_axis / np.linalg.norm(rotation_axis)
        r = R.from_rotvec(angle * rotation_axis)
        return r

#%%
# img_dir = root + "/data/calib1/img"
img_dir = root + "/data/test"
output_dir = root + '/GEMstack/offboard/calibration/output'

output_path = Path(output_dir)
output_path.mkdir(exist_ok=True, parents=True)

# 1. Feature extraction
extract_options = pycolmap.SiftExtractionOptions()
extract_options.num_threads = 2
pycolmap.extract_features(
    database_path=output_path/"database.db",
    image_path=img_dir,  
    camera_mode=pycolmap.CameraMode.PER_FOLDER,
    sift_options=extract_options,
    # device=pycolmap._core.Device.cuda
)

# 2. Feature matching
match_options = pycolmap.SequentialMatchingOptions()
match_options.overlap = 3
pycolmap.match_sequential(
    database_path=output_path/"database.db",
    matching_options=match_options
)
# match_options = pycolmap.VocabTreeMatchingOptions()
# match_options.vocab_tree_path = "/mnt/GEMstack/GEMstack/offboard/calibration/tools/vocab_tree_flickr100K_words32K.bin"
# pycolmap.match_vocabtree(
#     database_path=output_path/"database.db",
#     matching_options=match_options
# )

# 3. Sparse reconstruction
mapper_options = pycolmap.IncrementalPipelineOptions()
mapper_options.ba_refine_focal_length = 0
mapper_options.ba_refine_principal_point = 0
mapper_options.ba_refine_extra_params = 0
reconstructions = pycolmap.incremental_mapping(
    database_path=output_path/"database.db",
    image_path=img_dir,
    output_path=output_path,
    options=mapper_options
)

#%%
dense_dir = output_path
dense_dir.mkdir(exist_ok=True)

pycolmap.patch_math_stereo(
    input_path=output_path,
    output_path=dense_dir,
    output_type="TXT",
)

pycolmap.stereo_fusion(
    output_path=dense_dir,
    workspace_path=dense_dir,
    workspace_format="COLMAP",
    # max_image_size=2000,
    # gpu_index="0",
)

# 4. Depth Map Fusion (Global Registration)
# fused_pc = pycolmap.dense_fusion(
#     workspace_path=dense_dir,
#     output_path=dense_dir/"fused.ply",
#     max_image_size=2000,
#     gpu_index="0",
# )

#%%
[(id,reconstructions[0].image(id).name) for id in reconstructions[0].reg_image_ids()]

#%%
rec = reconstructions[0].__copy__()
#%%
trans = rec.image(142).cam_from_world
rec.transform(pycolmap.Sim3d(rotation=trans.rotation,translation=trans.translation))
rec.transform(pycolmap.Sim3d(rotation=np.array(R.from_euler('yx',(np.pi/2,-np.pi/2)).as_quat())))
rec.image(142).viewing_direction()
#%%
def load_scene(path):
    sc = np.load(path)['arr_0'] 
    sc = sc[~np.all(sc == 0, axis=1)] # remove (0,0,0)'s
    return sc

#%%
# import re
# ind = re.match("oak(\d+)\.png$",base).group(1)
ind = 1
sc = load_scene(f"/mnt/GEMstack/data/calib1/pc/ouster{ind}.npz")


#%%
pc = np.array([p.xyz for p in rec.points3D.values()])
#%%
from visualizer import visualizer as vis
import os
os.environ['DISPLAY'] = ':0'
#%%
v = vis()
v.add_pc(pc,color='red')
v.add_pc(sc,color='blue')
v.show()
v.close()



# %%
pc.shape