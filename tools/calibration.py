#%%
import numpy as np
import pycolmap
import open3d as o3d
from pathlib import Path

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

def SfM(img_dir,img_list=[],output_dir='/tmp'):
    """
    Process images into a 3D point cloud with automatic intrinsic estimation.
    
    Args:
        image_paths: List of paths to input images
        output_dir: Directory for temporary files and results
    
    Returns:
        Nx3 numpy array of 3D points in the coordinate system of the first camera
    """
    output_path = Path(output_dir)
    output_path.mkdir(exist_ok=True, parents=True)
    
    # 1. Feature extraction
    extract_options = pycolmap.SiftExtractionOptions()
    extract_options.num_threads = 4
    pycolmap.extract_features(
        database_path=output_path/"database.db",
        image_path=img_dir,  # Expect images in same directory
        image_list=img_list, 
        camera_mode=pycolmap.CameraMode.AUTO,
        sift_options=extract_options,
        # device=pycolmap._core.Device.cuda
    )
    
    # 2. Feature matching
    pycolmap.match_exhaustive(output_path/"database.db")
    
    # 3. Sparse reconstruction
    reconstructions = pycolmap.incremental_mapping(
        database_path=output_path/"database.db",
        image_path=img_dir,
        output_path=output_path,
    )
    
    if not reconstructions:
        raise RuntimeError("Reconstruction failed")
    
    # Get first reconstruction (usually the best one)
    reconstruction = reconstructions[0]
    
    # Convert to Nx3 numpy array
    points3D = np.array([p.xyz for p in reconstruction.points3D.values()])
    
    return points3D




#%%
def load_scene(path):
    sc = np.load(path)['arr_0'] 
    sc = sc[~np.all(sc == 0, axis=1)] # remove (0,0,0)'s
    return sc
# %%
from visualizer import visualizer as vis
pc = SfM("/mnt/GEMstack/data/test")
v = vis()
sc = load_scene("/mnt/GEMstack/data/calib1/septentio1.npz")
v.add_pc(pc,color='blue')
v.add_pc(sc,color='red')
v.show()
v.close()


