#%%
#%%
import numpy as np
import open3d as o3d
import os
import argparse

def npz_to_pcd(input_dir, output_dir, data_key='arr_0'):
    # Create output directory if it doesn't exist
    os.makedirs(output_dir, exist_ok=True)

    # Iterate over all .npz files in the input directory
    for npz_file in os.listdir(input_dir):
        if npz_file.endswith(".npz"):
            npz_path = os.path.join(input_dir, npz_file)
            output_pcd_path = os.path.join(output_dir, npz_file.replace('.npz', '.pcd'))

            try:
                # Load data from .npz file
                data = np.load(npz_path)
                if data_key not in data:
                    print(f"Skipping {npz_file}: Key '{data_key}' not found.")
                    continue

                point_cloud_np = data[data_key]

                # Validate shape (Nx3 for XYZ, Nx6 for XYZRGB, etc.)
                if point_cloud_np.ndim != 2 or point_cloud_np.shape[1] < 3:
                    print(f"Skipping {npz_file}: Invalid shape {point_cloud_np.shape}.")
                    continue

                # Create Open3D point cloud object
                pcd = o3d.geometry.PointCloud()

                # Assign points (required: Nx3 array)
                pcd.points = o3d.utility.Vector3dVector(point_cloud_np[:, :3])

                # Optionally assign colors (if available, e.g., Nx6 array with RGB)
                if point_cloud_np.shape[1] >= 6:
                    # RGB values (assuming they are stored as 0-255 integers)
                    colors = point_cloud_np[:, 3:6] / 255.0  # Normalize to [0, 1] for Open3D
                    pcd.colors = o3d.utility.Vector3dVector(colors)

                # Save to .pcd (supports ASCII or binary)
                o3d.io.write_point_cloud(output_pcd_path, pcd, write_ascii=True)
                print(f"Converted: {npz_file} -> {output_pcd_path}")

            except Exception as e:
                print(f"Failed to convert {npz_file}: {str(e)}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Convert .npz to .pcd using Open3D")
    parser.add_argument("--input_dir", type=str, required=True, help="Input directory with .npz files")
    parser.add_argument("--output_dir", type=str, required=True, help="Output directory for .pcd files")
    parser.add_argument("--data_key", type=str, default='points', help="Key for point cloud data in .npz files")
    args = parser.parse_args()

    npz_to_pcd(args.input_dir, args.output_dir, args.data_key)