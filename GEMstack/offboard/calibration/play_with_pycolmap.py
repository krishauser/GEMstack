#%%
root = '/mnt/GEMstack'
#%%
import numpy as np
import pycolmap
import open3d as o3d
from pathlib import Path
from scipy.spatial.transform import Rotation as R
from tools.save_cali import load_ex, save_ex, load_in, save_in
from tools.visualizer import visualizer as vis

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

#%%
img_dir = root + "/data/calib1/img"
# img_dir = root + "/data/test"
output_dir = root + '/GEMstack/offboard/calibration/output'
output_path = Path(output_dir)
output_path.mkdir(exist_ok=True, parents=True)

# 1. Feature extraction. google 'sift' to know what it does.
extract_options = pycolmap.SiftExtractionOptions()
extract_options.num_threads = 2
pycolmap.extract_features(
    database_path=output_path/"database.db",
    image_path=img_dir,  
    camera_mode=pycolmap.CameraMode.PER_FOLDER,
    sift_options=extract_options,
    # device=pycolmap._core.Device.cuda
)

# 2. Feature matching. match_sequential tries matching only on pairs close in filename ordering 
match_options = pycolmap.SequentialMatchingOptions()
match_options.overlap = 3   # how many subsequent images should be tried matching. 
                            # 3 is too small actually. do larger if you figured out pycolmap+cuda
pycolmap.match_sequential(
    database_path=output_path/"database.db",
    matching_options=match_options
)

# vocabtree matching. alternative to sequantial matching. 
# requires a vocab tree from https://demuc.de/colmap/
# match_options = pycolmap.VocabTreeMatchingOptions()
# match_options.vocab_tree_path = "/mnt/GEMstack/GEMstack/offboard/calibration/output/vocab_tree_flickr100K_words32K.bin"
# pycolmap.match_vocabtree(
#     database_path=output_path/"database.db",
#     matching_options=match_options
# )

# 3. Sparse reconstruction. ba_refine = 0 so it don't mess with the intrinsics we got in feature extraction stage.
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
#%%
# this part tries 3d reconstruction with stereo depth given the images registered in the previous part.
# but patch_match_stereo needs cuda and compiling pycolmap from source so haven't managed to run it.

# dense_dir = output_path
# dense_dir.mkdir(exist_ok=True)

# pycolmap.patch_match_stereo(
#     input_path=output_path,
#     output_path=dense_dir,
#     output_type="TXT",
# )

# pycolmap.stereo_fusion(
#     output_path=dense_dir,
#     workspace_path=dense_dir,
#     workspace_format="COLMAP",
#     max_image_size=2000,
#     gpu_index="0",
# )

# 4. Depth Map Fusion (Global Registration)
# fused_pc = pycolmap.dense_fusion(
#     workspace_path=dense_dir,
#     output_path=dense_dir/"fused.ply",
#     max_image_size=2000,
#     gpu_index="0",
# )

#%%
#%%
# get correspondence between image registrations and filenames, 
# and find image142 is from oak
[(id,reconstructions[0].image(id).name) for id in reconstructions[0].reg_image_ids()]

#%%
#make a copy because I was in jupyter lab trying to test different stuff with it
rec = reconstructions[0].__copy__()

#%%
# anchor the coordination system to image #142 
# because I found #142 is the first registered images from the front camera
trans = rec.image(142).cam_from_world
rec.transform(pycolmap.Sim3d(rotation=trans.rotation,translation=trans.translation))
print(rec.image(142).viewing_direction()) # shoud be a trivial direction

#%%
#%%
pc = np.array([p.xyz for p in rec.points3D.values()])
##################################################################################
#so far we have got pointcloud reconstructed from camera images
#################################################################################
# tranform to vehicle coordination just for visualization
camera_ex = load_ex(root + "/GEMstack/knowledge/calibration/gem_e4_oak.yaml",mode='matrix')
pc = np.pad(pc,((0,0),(0,1)),constant_values=1) @ camera_ex.T[:,:3]

#%%
def load_scene(path):
    sc = np.load(path)['arr_0'] 
    sc = sc[~np.all(sc == 0, axis=1)] # remove (0,0,0)'s
    return sc

#%%
#%%
# get lidar pointcloud
sc = load_scene(root + f"/data/calib1/pc/ouster1.npz")
lidar_ex = load_ex(root + "/GEMstack/knowledge/calibration/gem_e4_ouster.yaml",mode='matrix')
sc = np.pad(sc,((0,0),(0,1)),constant_values=1) @ lidar_ex.T[:,:3]

#%%
pc = np.array([p.xyz for p in rec.points3D.values()])
#%%
#import os
#os.environ['DISPLAY'] = ':0'
#%%
v = vis()
v.add_pc(pc,color='red')
v.add_pc(sc,color='blue')
v.show()
v.close()



# %%
pc.shape