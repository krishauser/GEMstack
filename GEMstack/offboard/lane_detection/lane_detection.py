
import argparse
from CLRerNet.libs.datasets.pipelines import Compose
from CLRerNet.libs.api.inference import get_prediction
from CLRerNet.libs.utils.visualizer import visualize_lanes
import cv2
import open3d as o3d
import numpy as np
import os
import cv2
import torch

from mmdet.apis import init_detector

from ...utils import settings


def undistort_image(image, K, D):
    '''
    Undistort the image using the camera intrinsic matrix and distortion coefficients.
    '''
    h, w = image.shape[:2]
    newK, _ = cv2.getOptimalNewCameraMatrix(K, D, (w, h), 1, (w, h))
    undistort_map1, undistort_map2 = cv2.initUndistortRectifyMap(K, D, R=None,
                                                                            newCameraMatrix=newK, size=(w, h),
                                                                            m1type=cv2.CV_32FC1)
    undistorted = cv2.remap(image, undistort_map1, undistort_map2, interpolation=cv2.INTER_NEAREST)
    return undistorted, newK

def inference_one_imave(model, img):
    """Inference on an image with the detector.
    Args:
        model (nn.Module): The loaded detector.
        img_path (str): Image path.
    Returns:
        img (np.ndarray): Image data with shape (width, height, channel).
        preds (List[np.ndarray]): Detected lanes.
    """
    ori_shape = img.shape
    data = dict(
        sub_img_name=None,
        img=img,
        gt_points=[],
        id_classes=[],
        id_instances=[],
        img_shape=ori_shape,
        ori_shape=ori_shape,
    )

    cfg = model.cfg
    model.bbox_head.test_cfg.as_lanes = False

    test_pipeline = Compose(cfg.test_dataloader.dataset.pipeline)

    data = test_pipeline(data)
    data_ = dict(
        inputs=[data["inputs"]],
        data_samples=[data["data_samples"]],
    )

    # forward the model
    with torch.no_grad():
        results = model.test_step(data_)

    lanes = results[0]['lanes']
    preds = get_prediction(lanes, ori_shape[0], ori_shape[1])

    return img, preds


def ipm_2d_to_3d(points_2d, K, R, T):
    """
    Projects 2D image lane points to 3D ground-plane coordinates using IPM.
    
    Args:
        points_2d: (N, 2) array of 2D points in image space (u, v)
        K: (3, 3) camera intrinsic matrix
        R: (3, 3) camera rotation matrix (world to camera)
        T: (3, 1) camera translation vector (world to camera)

    Returns:
        points_3d: (N, 3) array of 3D ground-plane points in world coordinates
    """
    # Inverse camera extrinsics: camera-to-world transformation
    Rt = np.hstack((R, T))  # (3,4)
    cam_to_world = np.linalg.inv(np.vstack((Rt, [0, 0, 0, 1])))  # (4,4)

    # Inverse of camera intrinsics
    K_inv = np.linalg.inv(K)

    points_3d = []
    for (u, v) in points_2d:
        uv1 = np.array([u, v, 1.0])
        ray_cam = K_inv @ uv1  # normalized camera coords
        ray_cam = ray_cam / np.linalg.norm(ray_cam)

        # Extend to homogeneous 4D ray direction
        ray_dir_homo = np.append(ray_cam, [0.0])  # direction (no translation)
        cam_origin_homo = np.array([0.0, 0.0, 0.0, 1.0])  # camera center

        # Transform ray to world coordinates
        ray_origin_world = cam_to_world @ cam_origin_homo
        ray_dir_world = cam_to_world @ ray_dir_homo
        ray_dir_world = ray_dir_world[:3]
        ray_origin_world = ray_origin_world[:3]

        # Intersect with ground plane Z=0: solve for t in O + tD → Z=0
        t = -ray_origin_world[2] / ray_dir_world[2]
        point_ground = ray_origin_world + t * ray_dir_world
        points_3d.append(point_ground)

    return np.array(points_3d)

def detect_lanes(model, img, K, D, R, T, save_path):
    undistorted_img, current_K = undistort_image(img, K, D)
    resized_img = cv2.resize(undistorted_img, (1640, 590))
    img, results_normal = inference_one_imave(model, resized_img)
    combined_lanes = []
    combined_lanes_3d = []
    for lane in results_normal:
        combined_lanes.append(lane)
        combined_lanes_3d.append(ipm_2d_to_3d(lane, current_K, R, T))

    visualize_lanes(img, combined_lanes, save_path=save_path)
    return combined_lanes_3d

def get_camera_intrinsic_matrix(camera_type):
    '''
    Get the camera intrinsic matrix for the given camera type
    '''
    if camera_type == "front_right" or camera_type == "fr":
        # Provided intrinsics
        focal = settings.get("calibration.front_right_camera.intrinsics.focal")  # fx, fy
        center = settings.get("calibration.front_right_camera.intrinsics.center")   # cx, cy
        fr_cam_distort = settings.get("calibration.front_right_camera.intrinsics.distort")
        skew = settings.get("calibration.front_right_camera.intrinsics.skew")

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
        focal = settings.get("calibration.rear_right_camera.intrinsics.focal")  # fx, fy
        center = settings.get("calibration.rear_right_camera.intrinsics.center")   # cx, cy
        rr_cam_distort = settings.get("calibration.rear_right_camera.intrinsics.distort")
        skew = settings.get("calibration.rear_right_camera.intrinsics.skew")

        fx, fy = focal
        cx, cy = center

        # Build K matrix
        rr_cam_K = np.array([
            [fx, skew, cx],
            [0,  fy,   cy],
            [0,   0,    1]
        ])
        return rr_cam_K, np.array(rr_cam_distort)
    elif camera_type == "front_left" or camera_type == "fl":
        # Provided intrinsics
        focal = settings.get("calibration.front_left_camera.intrinsics.focal")  # fx, fy
        center = settings.get("calibration.front_left_camera.intrinsics.center")   # cx, cy
        fl_cam_distort = settings.get("calibration.front_left_camera.intrinsics.distort")
        skew = settings.get("calibration.front_left_camera.intrinsics.skew")

        fx, fy = focal
        cx, cy = center

        # Build K matrix
        fl_cam_K = np.array([
            [fx, skew, cx],
            [0,  fy,   cy],
            [0,   0,    1]
        ])
        return fl_cam_K, np.array(fl_cam_distort)
    elif camera_type == "rear_left" or camera_type == "rl":
        # Provided intrinsics
        focal = settings.get("calibration.rear_left_camera.intrinsics.focal")  # fx, fy
        center = settings.get("calibration.rear_left_camera.intrinsics.center")   # cx, cy
        rl_cam_distort = settings.get("calibration.rear_left_camera.intrinsics.distort")
        skew = settings.get("calibration.rear_left_camera.intrinsics.skew")

        fx, fy = focal
        cx, cy = center

        # Build K matrix
        rl_cam_K = np.array([
            [fx, skew, cx],
            [0,  fy,   cy],
            [0,   0,    1]
        ])
        return rl_cam_K, np.array(rl_cam_distort)
    else:
        raise ValueError(f"Camera type {camera_type} not supported")

def get_camera_extrinsic_matrix(camera_type):
    '''
    Get the camera extrinsic matrix for the given camera type
    '''
    if camera_type == "front_right" or camera_type == "fr":
        # From the settings
        rotation = np.array(settings.get("calibration.front_right_camera.extrinsics.rotation"))

        translation = np.array(settings.get("calibration.front_right_camera.extrinsics.position"))

        # Build 4x4 homogeneous transformation matrix
        fr_to_vehicle = np.eye(4)
        fr_to_vehicle[:3, :3] = rotation
        fr_to_vehicle[:3, 3] = translation
        return fr_to_vehicle
    elif camera_type == "rear_right" or camera_type == "rr":
        # From the settings
        rotation = np.array(settings.get("calibration.rear_right_camera.extrinsics.rotation"))

        translation = np.array(settings.get("calibration.rear_right_camera.extrinsics.position"))

        # Build 4x4 homogeneous transformation matrix
        rr_to_vehicle = np.eye(4)
        rr_to_vehicle[:3, :3] = rotation
        rr_to_vehicle[:3, 3] = translation
        return rr_to_vehicle 
    elif camera_type == "front_left" or camera_type == "fl":
        rotation = np.array(settings.get("calibration.front_left_camera.extrinsics.rotation"))
        translation = np.array(settings.get("calibration.front_left_camera.extrinsics.position"))
        
        # Build 4x4 homogeneous transformation matrix
        fl_to_vehicle = np.eye(4)
        fl_to_vehicle[:3, :3] = rotation
        fl_to_vehicle[:3, 3] = translation
        return fl_to_vehicle
    elif camera_type == "rear_left" or camera_type == "rl":
        rotation = np.array(settings.get("calibration.rear_left_camera.extrinsics.rotation"))
        translation = np.array(settings.get("calibration.rear_left_camera.extrinsics.position"))
        
        # Build 4x4 homogeneous transformation matrix
        rl_to_vehicle = np.eye(4)
        rl_to_vehicle[:3, :3] = rotation
        rl_to_vehicle[:3, 3] = translation
        return rl_to_vehicle
    else:
        raise ValueError(f"Camera type {camera_type} not supported")
    
def add_args(parser):
    parser.add_argument('--image_folder', type=str, help='Path to the folder containing the images')
    parser.add_argument('--output_folder', type=str, help='Path to the folder to save the output')
    parser.add_argument('--output_file_prefix', type=str, help='Prefix of the output file')
    parser.add_argument('--camera_type', type=str, choices=['fr', 'fl', 'rr', 'rl', 'front_right', 'rear_right', 'front_left', 'rear_left'], default='front_right', help='Camera type to load the camera intrinsic and extrinsic matrix')
    parser.add_argument('--config', type=str, default='clrernet/configs/lane_detection.py', help='Path to the config file')
    parser.add_argument('--checkpoint', type=str, default='clrernet/checkpoints/lane_detection.pth', help='Path to the checkpoint file')
    parser.add_argument('--device', type=str, default='cuda:0', help='Device to run the model on')

def run_lane_detection():
    '''
    Run the lane detection pipeline
    '''
    parser = argparse.ArgumentParser()
    add_args(parser)
    args = parser.parse_args()

    K, D = get_camera_intrinsic_matrix(args.camera_type)
    extrinsic_matrix = get_camera_extrinsic_matrix(args.camera_type)
    R = extrinsic_matrix[:3, :3]
    T = extrinsic_matrix[:3, 3]

    model = init_detector(args.config, args.checkpoint, device=args.device)
    image_folder = args.image_folder
    output_folder = args.output_folder
    inputfiles = [os.path.join(image_folder, f) for f in os.listdir(image_folder) if f.lower().endswith(('.jpg', '.jpeg', '.png'))]
    
    predicted_lanes = []
    for img_path in inputfiles:
        img = cv2.imread(img_path)
        img_name = os.path.basename(img_path)
        save_path = os.path.join(output_folder, f'{img_name.split(".")[0]}_with_detected_lanes.png')
        os.makedirs(os.path.dirname(save_path), exist_ok=True)
        predicted_lanes.append(detect_lanes(model, img, K, D, R, T, save_path))

    # Convert predicted_lanes to a single array of 3D points
    all_lanes_3d = np.concatenate(predicted_lanes, axis=0)

    # Save all_lanes_3d to a .ply file
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(all_lanes_3d)
    o3d.io.write_point_cloud(os.path.join(output_folder, f'{args.output_file_prefix}_all_lanes_3d.ply'), pcd)

    # save numpy array
    np.save(os.path.join(output_folder, f'{args.output_file_prefix}_all_lanes_3d.npy'), all_lanes_3d)
    

if __name__ == '__main__':
    run_lane_detection()

