import argparse
import os
import shutil
import pycolmap
import subprocess
from GEMstack.GEMstack.knowledge.calibration.calib_util import save_in

def run_colmap_command(args):
    result = subprocess.run(args, capture_output=True, text=True)
    if result.returncode != 0:
        print(f"Command failed: {' '.join(args)}")
        print(f"Error: {result.stderr}")
        raise RuntimeError("COLMAP command failed")
    return result

def main(input_imgs, output_dir, out, refine=True):
    # Setup directory structure
    workspace_dir = os.path.join(output_dir, 'sfm_workspace')
    image_dir = os.path.join(workspace_dir, 'images')
    os.makedirs(image_dir, exist_ok=True)
    
    # Copy images to workspace
    print(f"Copying images to workspace...")
    for path in input_imgs:
        filename = path.split('/')[-1]
        dst = os.path.join(image_dir, filename)
        shutil.copy(path, dst)
    
    # Create database path
    database_path = os.path.join(workspace_dir, 'database.db')
    if os.path.exists(database_path):
        os.remove(database_path)
    
    # Feature extraction
    print("Extracting features...")
    # pycolmap.extract_features(
    #     database_path, image_dir, 
    #     camera_mode=pycolmap.CameraMode.SINGLE,
    #     camera_model=pycolmap.CameraModelId.OPENCV
    #     )
    run_colmap_command([
        "colmap", "feature_extractor",
        "--database_path", database_path,
        "--image_path", image_dir,
        "--ImageReader.single_camera", "1",
        "--ImageReader.camera_model", "OPENCV",
        "--SiftExtraction.estimate_affine_shape", "1",
        "--SiftExtraction.domain_size_pooling", "1"
    ])
    
    # Feature matching
    print("Matching features...")
    match_options = pycolmap.SequentialMatchingOptions()
    match_options.overlap = 2
    pycolmap.match_sequential(
        database_path,
        matching_options=match_options
    )
    # Run SfM reconstruction
    mapper_options = pycolmap.IncrementalPipelineOptions()
    if refine:
        mapper_options.ba_refine_focal_length = 1
        mapper_options.ba_refine_principal_point = 1
        mapper_options.ba_refine_extra_params = 1
    else:
        mapper_options.ba_refine_focal_length = 0
        mapper_options.ba_refine_principal_point = 0
        mapper_options.ba_refine_extra_params = 0

    print("Running incremental SfM...")
    output_path = os.path.join(workspace_dir, 'sparse')
    os.makedirs(output_path, exist_ok=True)
    reconstructions = pycolmap.incremental_mapping(
        database_path=database_path,
        image_path=image_dir,
        output_path=output_path,
        options=mapper_options
    )
    
    # Process results
    if not reconstructions:
        print("SfM failed to reconstruct the scene!")
        return
    
    camera:pycolmap._core.Camera = 0
    print("\nCamera calibration parameters:")
    for idx, reconstruction in reconstructions.items():
        print(f"\nReconstruction {idx + 1}:")
        for camera_id, camera in reconstruction.cameras.items():
            print(f"\nCamera ID {camera_id}:")
            print(f"Model: {camera.model}")
            print(f"Parameters: {camera.params}")
            print(f"Parameters info: {camera.params_info}")
            # print(f"Focal length: {camera.focal_length}")
            print(f"Focal length x: {camera.focal_length_x}")
            print(f"Focal length y: {camera.focal_length_y}")
            print(f"Principal point x: {camera.principal_point_x}")
            print(f"Principal point y: {camera.principal_point_y}")
            save_in(
                path=out,
                focal=[camera.focal_length_x,camera.focal_length_y],
                center=[camera.principal_point_x,camera.principal_point_y],
                distort=list(camera.params[4:])+[0.0],
                )
    print("\nCalibration complete! Results saved to:", workspace_dir)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Camera calibration using SfM')
    parser.add_argument('--input_imgs','-i', nargs='+', help='List of input imgs', required=True)
    parser.add_argument('--workspace','-w', type=str, required=False, default= '/tmp/colmap_tmp',
                        help='Output directory for results')
    parser.add_argument('--out_file','-o', type=str, required=True,
                        help='output yaml file')
    args = parser.parse_args()
    
    main(args.input_imgs, args.workspace, args.out_file)