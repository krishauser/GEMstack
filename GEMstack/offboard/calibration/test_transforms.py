
import numpy as np
import cv2 as cv
import cv2 as cv
import matplotlib.pyplot as plt
import argparse
import argparse
from scipy.spatial.transform import Rotation as R
from matplotlib.widgets import Slider
from GEMstack.GEMstack.knowledge.calibration.calib_util import load_ex, load_in, save_ex, undistort_image

x_rot = y_rot = z_rot = x_trans = y_trans = z_trans = None

def project_points(T_lidar_to_camera, lidar_homogeneous, K, shape):
    # Transform LiDAR points to Camera Frame
    lidar_points_camera = (T_lidar_to_camera @ lidar_homogeneous.T).T  # (N, 4)

    # Extract 3D points in camera frame
    X_c = lidar_points_camera[:, 0]
    Y_c = lidar_points_camera[:, 1]
    Z_c = lidar_points_camera[:, 2]  # Depth
    # Extract 3D points in camera frame
    X_c = lidar_points_camera[:, 0]
    Y_c = lidar_points_camera[:, 1]
    Z_c = lidar_points_camera[:, 2]  # Depth

    # Avoid division by zero
    valid = Z_c > 0
    X_c, Y_c, Z_c = X_c[valid], Y_c[valid], Z_c[valid]
    # Avoid division by zero
    valid = Z_c > 0
    X_c, Y_c, Z_c = X_c[valid], Y_c[valid], Z_c[valid]

    # Project points onto image plane
    u = (K[0, 0] * X_c / Z_c) + K[0, 2]
    v = (K[1, 1] * Y_c / Z_c) + K[1, 2]
    # Project points onto image plane
    u = (K[0, 0] * X_c / Z_c) + K[0, 2]
    v = (K[1, 1] * Y_c / Z_c) + K[1, 2]

    # Filter points within image bounds
    img_h, img_w, _ = shape
    valid_pts = (u >= 0) & (u < img_w) & (v >= 0) & (v < img_h)
    u, v = u[valid_pts], v[valid_pts]
    return u, v

def modify_transform(T_lidar_to_camera):
    modified = np.eye(4)
    rotation_mat = R.from_euler('xyz', [x_rot.val, y_rot.val, z_rot.val], degrees=True).as_matrix()
    translation_vec = np.array([x_trans.val, y_trans.val, z_trans.val])
    modified[:3, :3] = rotation_mat
    modified[:3, 3] = translation_vec
    modified = modified @ T_lidar_to_camera
    return modified

def create_sliders(fig, update_func):
    # Create space for sliders
    fig.subplots_adjust(bottom=0.4)
    x_rot_ax = fig.add_axes([0.25, 0.3, 0.65, 0.03])
    y_rot_ax = fig.add_axes([0.25, 0.25, 0.65, 0.03])
    z_rot_ax = fig.add_axes([0.25, 0.2, 0.65, 0.03])
    x_trans_ax = fig.add_axes([0.25, 0.15, 0.65, 0.03])
    y_trans_ax = fig.add_axes([0.25, 0.1, 0.65, 0.03])
    z_trans_ax = fig.add_axes([0.25, 0.05, 0.65, 0.03])

    # Make sliders to control the rotation
    global x_rot
    x_rot = Slider(
        ax=x_rot_ax,
        label="X Rotation",
        valmin=-30,
        valmax=30,
        valinit=0,
        orientation="horizontal"
    )

    global y_rot
    y_rot = Slider(
        ax=y_rot_ax,
        label="Y Rotation",
        valmin=-30,
        valmax=30,
        valinit=0,
        orientation="horizontal"
    )

    global z_rot
    z_rot = Slider(
        ax=z_rot_ax,
        label="Z Rotation",
        valmin=-30,
        valmax=30,
        valinit=0,
        orientation="horizontal"
    )

    # Make sliders to control the translation
    global x_trans
    x_trans = Slider(
        ax=x_trans_ax,
        label="X Translation",
        valmin=-10,
        valmax=10,
        valinit=0,
        orientation="horizontal"
    )

    global y_trans
    y_trans = Slider(
        ax=y_trans_ax,
        label="Y Translation",
        valmin=-10,
        valmax=10,
        valinit=0,
        orientation="horizontal"
    )

    global z_trans
    z_trans = Slider(
        ax=z_trans_ax,
        label="Z Translation",
        valmin=-10,
        valmax=10,
        valinit=0,
        orientation="horizontal"
    )

    # Register the update function with each slider
    x_rot.on_changed(update_func)
    y_rot.on_changed(update_func)
    z_rot.on_changed(update_func)
    x_trans.on_changed(update_func)
    y_trans.on_changed(update_func)
    z_trans.on_changed(update_func)

def main():
    # Collect arguments
    parser = argparse.ArgumentParser(description='calculate intrinsics from checkerboard images',
                                     formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('-p', '--img_path', type=str, required=True,
                       help='Path to PNG image')
    parser.add_argument('-l', '--lidar_path', type=str, required=True,
                       help='Path to lidar NPZ point cloud')
    parser.add_argument('-t', '--lidar_transform_path', type=str, required=True,
                       help='Path to lidar extrinsics')
    parser.add_argument('-e', '--camera_transform_path', type=str, required=True,
                       help='Path to camera extrinsics')
    parser.add_argument('-i', '--img_intrinsics_path', type=str, required=True,
                       help='Path to yaml file for image intrinsics')
    parser.add_argument('-o', '--out_path', type=str, required=False,
                       help='Path to output ymal file for camera intrinsics')
    parser.add_argument('-u','--undistort', action='store_true',
                       help='Whether to use distortion parameters')
    
    args = parser.parse_args()

    # Load LiDAR points
    lidar_data = np.load(args.lidar_path)
    lidar_points = lidar_data['arr_0']  # Ensure the correct key

    # Load Camera Image
    image = cv.imread(args.img_path)
    image = cv.cvtColor(image, cv.COLOR_BGR2RGB)  # Convert BGR to RGB

    # Load Transformation Matrices
    T_lidar_to_vehicle = load_ex(args.lidar_transform_path, 'matrix')
    T_camera_to_vehicle = load_ex(args.camera_transform_path, 'matrix')
    T_lidar_to_camera = np.linalg.inv(T_camera_to_vehicle) @ T_lidar_to_vehicle

    # Load Camera Intrinsics
    if args.undistort:
        K, distortion_coefficients = load_in(args.img_intrinsics_path, return_distort=True)
        image, K = undistort_image(image, K, distortion_coefficients)
    else:
        K = load_in(args.img_intrinsics_path)

    # Convert LiDAR points to homogeneous coordinates
    num_points = lidar_points.shape[0]
    lidar_homogeneous = np.hstack((lidar_points, np.ones((num_points, 1))))  # (N, 4)

    # Initialize plot
    plt.ion()
    fig, ax = plt.subplots()

    # Project lidar points to camera frame
    u, v = project_points(T_lidar_to_camera, lidar_homogeneous, K, image.shape)
    
    # Plot projected points on camera image
    ax.imshow(image)
    dots = ax.scatter(u, v, s=2, c='cyan', alpha=0.2)  # Dots for projected points
    ax.set_title("Projected LiDAR Points on Camera Image", pad=0)

    # Define update function
    def update(val):
        modified = modify_transform(T_lidar_to_camera)

        # Update projected points
        u, v = project_points(modified, lidar_homogeneous, K, image.shape)
        dots.set_offsets(np.c_[u, v])
        plt.draw()
    
    # Generate sliders and display plot
    create_sliders(fig, update)
    plt.draw()

    # Update and keep displaying the modified plot
    keep_running = True
    while keep_running:
        try:
            if plt.get_fignums():
                plt.pause(0.1)
            else:
                keep_running = False
        except:
            keep_running = False

    # Output and write the fine-tuned translation matrix
    modified = modify_transform(T_lidar_to_camera)
    print(T_lidar_to_vehicle @ np.linalg.inv(modified))
    if args.out_path:
        save_ex(args.out_path, matrix=T_lidar_to_vehicle @ np.linalg.inv(modified))

if __name__ == '__main__':
    main()
