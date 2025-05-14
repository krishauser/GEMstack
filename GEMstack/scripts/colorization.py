import os
from pathlib import Path
import numpy as np
import argparse
import open3d as o3d
import cv2
import open3d as o3d

def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--folder_path", type=str, required=True)
    parser.add_argument("--output_path", type=str, required=True)
    parser.add_argument("--camera_types", type=str, required=True)
    return parser.parse_args()


def save_ply_with_open3d(points, filename):
    pc = o3d.geometry.PointCloud()
    pc.points = o3d.utility.Vector3dVector(points[:, :3])
    if points.shape[1] == 6:
        colors = points[:, 3:6] / 255.0  # normalize RGB to [0, 1]
        pc.colors = o3d.utility.Vector3dVector(colors)
    o3d.io.write_point_cloud(filename, pc)

def get_camera_extrinsic_matrix(camera_type):
    if camera_type == "front_right" or camera_type == "fr":
        # From your file
        rotation = np.array([
            [-0.7168464770690616, -0.10046018208578958,  0.6899557088168523],
            [-0.6970911725372957,  0.12308618950445319, -0.7063382243117325],
            [-0.01396515249660048, -0.9872981017750231, -0.15826380744561577]
        ])

        translation = np.array([1.8861563355156226, -0.7733611068168774, 1.6793040225335112])

        # Build 4x4 homogeneous transformation matrix
        fr_to_vehicle = np.eye(4)
        fr_to_vehicle[:3, :3] = rotation
        fr_to_vehicle[:3, 3] = translation
        return fr_to_vehicle
    elif camera_type == "rear_right" or camera_type == "rr":
        # From your file
        rotation = np.array([
            [-0.7359657309159472, 0.15986191414426415, -0.6578743127098735], 
            [0.6768157805459531, 0.14993386619459964, -0.7207220233709469], 
            [-0.016578363047300385, -0.9756864271752846, -0.21854325362408236]
        ])

        translation = np.array([0.11419591502518789, -0.6896311735924415, 1.711181163333824])

        # Build 4x4 homogeneous transformation matrix
        rr_to_vehicle = np.eye(4)
        rr_to_vehicle[:3, :3] = rotation
        rr_to_vehicle[:3, 3] = translation
        return rr_to_vehicle 
    else:
        raise ValueError(f"Camera type {camera_type} not supported")
    
def get_camera_intrinsic_matrix(camera_type):
    if camera_type == "front_right" or camera_type == "fr":
        # Provided intrinsics
        focal = [1176.2554468073797, 1175.1456876174707]  # fx, fy
        center = [966.4326452411585, 608.5803255934914]   # cx, cy
        fr_cam_distort = [-0.2701363254469883, 0.16439325523243875, -0.001607207824773341, -7.412467081891699e-05,
        -0.06199397580030171]
        skew = 0  # assume no skew

        fx, fy = focal
        cx, cy = center

        # Build K matrix
        fr_cam_K = np.array([
            [fx, skew, cx],
            [0,  fy,   cy],
            [0,   0,    1]
        ])
        return fr_cam_K, np.array(fr_cam_distort)
    elif camera_type == "rear_right" or camera_type == "rr":
        # Provided intrinsics
        focal = [1162.3787554048329, 1162.855381183851]  # fx, fy
        center = [956.2663906909728, 569.2039945552984]   # cx, cy
        rr_cam_distort = [-0.25040910859151444, 0.1109210921906881, -0.00041247665414900384, 0.0008205455176671751,
        -0.026395952816984845]
        skew = 0  # assume no skew

        fx, fy = focal
        cx, cy = center

        # Build K matrix
        rr_cam_K = np.array([
            [fx, skew, cx],
            [0,  fy,   cy],
            [0,   0,    1]
        ])
        return rr_cam_K, np.array(rr_cam_distort)
    else:
        raise ValueError(f"Camera type {camera_type} not supported")
    
def get_lidar_extrinsic_matrix(lidar_type):
    if lidar_type == "ouster":
        # From your file
        rotation = np.array([
            [1,0,0],
            [0,1,0],
            [0,0,1]
        ])

        translation = np.array([1.10,0,2.03])

        # Build 4x4 homogeneous transformation matrix
        ouster_to_vehicle = np.eye(4)
        ouster_to_vehicle[:3, :3] = rotation
        ouster_to_vehicle[:3, 3] = translation
        return ouster_to_vehicle
    else:
        raise ValueError(f"Lidar type {lidar_type} not supported")

def sample_rgb_colors(image_rgb, u_f, v_f):
    """
    Sample RGB colors at subpixel positions (u_f, v_f) from image.
    Args:
        image_rgb: np.ndarray (H, W, 3), float32 in [0,1]
        u_f: (N,) float32
        v_f: (N,) float32
    Returns:
        (N, 3) array of RGB colors
    """
    assert image_rgb.dtype == np.float32 and image_rgb.max() <= 1.0
    colors = []

    for x, y in zip(u_f, v_f):
        patch = cv2.getRectSubPix(image_rgb, patchSize=(1, 1), center=(x, y))
        colors.append(patch[0, 0])  # (1, 1, 3) → get RGB triplet

    return np.array(colors)


def main():
    args = parse_args()
    folder_path = args.folder_path  
    output_path = args.output_path
    camera_types = args.camera_types.split(',')
    print('camera_types', camera_types)

    folder = Path(folder_path)
    files = list(folder.glob("*.b"))  # all files

    for file in files:
        # print(file.name)
        data = np.load(file, allow_pickle=True)
        # Save
        os.makedirs(output_path, exist_ok=True)
        save_ply_with_open3d(data, f"{output_path}/pure_pointcloud_{file.name}.ply")
        colors_full = np.zeros((data.shape[0], 3), dtype=np.float32)  # default black
        points_h = np.hstack([data, np.ones((data.shape[0], 1))])  # shape: (N, 4)

        for camera_type in camera_types:

            # Get camera extrinsic matrix
            camera_to_vehicle = get_camera_extrinsic_matrix(camera_type)

            # Get lidar extrinsic matrix
            lidar_to_vehicle = get_lidar_extrinsic_matrix("ouster")

            # Get camera intrinsic matrix
            camera_K, camera_distort = get_camera_intrinsic_matrix(camera_type)

            # Project points to camera
            # Transform lidar points into camera frame
            T_vehicle_to_cam = np.linalg.inv(camera_to_vehicle)
            T_lidar_to_cam = T_vehicle_to_cam @ lidar_to_vehicle

            # Convert points to homogeneous coordinates

            # Transform to camera frame
            points_cam = (T_lidar_to_cam @ points_h.T).T                   # shape: (N, 4)
            points_cam = points_cam[:, :3]                                 # drop homogeneous component

            # Filter out points behind the camera
            mask = points_cam[:, 2] > 0
            # points_cam = points_cam[mask]

            # Load image
            point_cloud_number = file.name.split("_")[1].split('.')[0]
            # image_path = f"{file.parent.name}/{camera_type}_{point_cloud_number}.png"
            image_path = file.parent / f"{camera_type}_{point_cloud_number}.png"
            image_path = str(image_path)
            if not Path(image_path).exists():
                print(f"❌ Image file does not exist: {image_path}")
                continue
            image_raw = cv2.imread(image_path)
            h, w = image_raw.shape[:2]

            cam_distort_matrix = camera_distort
            cam_k = camera_K
            # Get optimal camera matrix (undistorted intrinsics)
            cam_K_new, roi = cv2.getOptimalNewCameraMatrix(cam_k, cam_distort_matrix, (w, h), 0)

            # Undistort the image
            image_undistorted = cv2.undistort(image_raw, cam_k, cam_distort_matrix, None, cam_K_new)

            # Project to image
            proj = (cam_K_new @ points_cam.T).T  # shape: (N, 3)
            with np.errstate(divide='ignore', invalid='ignore'):
                proj[:, 0] = np.divide(proj[:, 0], proj[:, 2], out=np.zeros_like(proj[:, 0]), where=proj[:, 2] != 0)
                proj[:, 1] = np.divide(proj[:, 1], proj[:, 2], out=np.zeros_like(proj[:, 1]), where=proj[:, 2] != 0)

            # proj[:, 0] /= proj[:, 2]
            # proj[:, 1] /= proj[:, 2]
            pixels = proj[:, :2]  # shape: (N, 2)

            # Step 1: filter front-facing
            finite_mask = np.isfinite(proj[:, 0]) & np.isfinite(proj[:, 1])
            # mask_front = mask  # Z_cam > 0
            mask_front = mask & finite_mask

            # Projected pixel coords (u, v)
            u = pixels[:, 0]
            v = pixels[:, 1]
            if np.any(np.isnan(u)) or np.any(np.isnan(v)):
                print(f"❌ NaN values in u or v for {camera_type}")
                continue
            if np.any(np.isinf(u)) or np.any(np.isinf(v)):
                print(f"❌ Inf values in u or v for {camera_type}")
                continue
            # Step 2: in image bounds

            # Load image and convert to RGB
            image_undistorted = cv2.cvtColor(image_undistorted, cv2.COLOR_BGR2RGB)
            image_undistorted = image_undistorted.astype(np.float32) / 255.0
            h, w, _ = image_undistorted.shape
            in_bounds = (u >= 0) & (u < w - 1) & (v >= 0) & (v < h - 1)
            # in_bounds = (u >= 0) & (v >= 0)
            # Step 3: Combine masks
            visible_mask = mask_front & in_bounds
            # visible_mask = mask_front

            # Step 4: Get visible pixel coords
            u_f = u[visible_mask].astype(np.float32)
            v_f = v[visible_mask].astype(np.float32)
            if u_f.shape[0] == 0 or v_f.shape[0] == 0:
                print(f"❌ No visible points for {camera_type}")
                continue
            # Step 5: Interpolate RGB
            colors_interpolated = sample_rgb_colors(image_undistorted, u_f, v_f)
            # Step 6: Assign to full color array
            # colors_interpolated = np.concatenate(colors_interpolated, axis=1)  # shape: (N, 3)
            colors_full[visible_mask] = colors_interpolated  # only visible points get color

        # Step 7: Create point cloud with full color info

        colored_pcd = o3d.geometry.PointCloud()
        colored_pcd.points = o3d.utility.Vector3dVector(data)         # full point set
        colored_pcd.colors = o3d.utility.Vector3dVector(colors_full)   # partial color

        o3d.io.write_point_cloud(f"{output_path}/colored_pointcloud_{file.name}.ply", colored_pcd)
            
        

if __name__ == "__main__":
    main()
