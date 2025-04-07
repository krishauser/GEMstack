import numpy as np
import cv2
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
from matplotlib.widgets import Slider

keep_running = True

# Load LiDAR points
lidar_data = np.load("./data/ouster32.npz")  # Update filename if needed
lidar_points = lidar_data['arr_0']  # Ensure the correct key

# Load Camera Image
image = cv2.imread("./data/fr_rect32.png")  # Update filename if needed
image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)  # Convert BGR to RGB

# Transformation Matrix
T_lidar_camera = np.array(
    [[-0.65266466, -0.75745371, -0.01710912, -0.54157703],
     [-0.08467423,  0.09536332, -0.99183472, -0.37038288],
     [ 0.75290047, -0.64588677, -0.12637708,  0.0245309 ],
     [ 0.        ,  0.        ,  0.        ,  1.        ]]
)

# Convert LiDAR points to homogeneous coordinates
num_points = lidar_points.shape[0]
lidar_homogeneous = np.hstack((lidar_points, np.ones((num_points, 1))))  # (N, 4)

# Transform LiDAR points to Camera Frame
lidar_points_camera = (T_lidar_camera @ lidar_homogeneous.T).T  # (N, 4)


# intrinsic matrix
K = np.array([
    [1180.753804,  0.        , 934.859447],
    [ 0.        , 1177.034929, 607.266974],
    [ 0.        ,  0.        ,  1.       ]
], dtype=np.float32)

# Extract 3D points in camera frame
X_c = lidar_points_camera[:, 0]
Y_c = lidar_points_camera[:, 1]
Z_c = lidar_points_camera[:, 2]  # Depth

# Avoid division by zero
valid = Z_c > 0
X_c, Y_c, Z_c = X_c[valid], Y_c[valid], Z_c[valid]

# Project points onto image plane
u = (K[0, 0] * X_c / Z_c) + K[0, 2]
v = (K[1, 1] * Y_c / Z_c) + K[1, 2]

# Filter points within image bounds
img_h, img_w, _ = image.shape
valid_pts = (u >= 0) & (u < img_w) & (v >= 0) & (v < img_h)
u, v = u[valid_pts], v[valid_pts]

# Create space for sliders
plt.ion()
fig, ax = plt.subplots()
fig.subplots_adjust(bottom=0.4)
scale_ax = fig.add_axes([0.25, 0.35, 0.65, 0.03])
x_rot_ax = fig.add_axes([0.25, 0.3, 0.65, 0.03])
y_rot_ax = fig.add_axes([0.25, 0.25, 0.65, 0.03])
z_rot_ax = fig.add_axes([0.25, 0.2, 0.65, 0.03])
x_trans_ax = fig.add_axes([0.25, 0.15, 0.65, 0.03])
y_trans_ax = fig.add_axes([0.25, 0.1, 0.65, 0.03])
z_trans_ax = fig.add_axes([0.25, 0.05, 0.65, 0.03])

# Plot projected points on camera image
ax.imshow(image)
dots = ax.scatter(u, v, s=2, c='cyan', alpha=0.2)  # Dots for projected points
ax.set_title("Projected LiDAR Points on Camera Image", pad=0)

# Make a slider to control the scale
scale = Slider(
    ax=scale_ax,
    label="Scale",
    valmin=0.75,
    valmax=1.25,
    valinit=1,
    orientation="horizontal"
)

# Make sliders to control the rotation
x_rot = Slider(
    ax=x_rot_ax,
    label="X Rotation",
    valmin=-30,
    valmax=30,
    valinit=0,
    orientation="horizontal"
)

y_rot = Slider(
    ax=y_rot_ax,
    label="Y Rotation",
    valmin=-30,
    valmax=30,
    valinit=0,
    orientation="horizontal"
)

z_rot = Slider(
    ax=z_rot_ax,
    label="Z Rotation",
    valmin=-30,
    valmax=30,
    valinit=0,
    orientation="horizontal"
)

# Make sliders to control the scale
x_trans = Slider(
    ax=x_trans_ax,
    label="X Translation",
    valmin=-10,
    valmax=10,
    valinit=0,
    orientation="horizontal"
)

y_trans = Slider(
    ax=y_trans_ax,
    label="Y Translation",
    valmin=-10,
    valmax=10,
    valinit=0,
    orientation="horizontal"
)

z_trans = Slider(
    ax=z_trans_ax,
    label="Z Translation",
    valmin=-10,
    valmax=10,
    valinit=0,
    orientation="horizontal"
)

def update(val):
    global T_lidar_camera
    modified = np.copy(T_lidar_camera)
    rotation_mat = R.from_euler('xyz', [x_rot.val, y_rot.val, z_rot.val], degrees=True).as_matrix()
    scale_mat = np.array([[scale.val, 0, 0], [0, scale.val, 0], [0, 0, scale.val]])
    translation_vec = np.array([x_trans.val, y_trans.val, z_trans.val])
    modified[:3, :3] = T_lidar_camera[:3, :3] @ rotation_mat @ scale_mat
    modified[:3, 3] = T_lidar_camera[:3, 3] + translation_vec

    # Transform LiDAR points to Camera Frame
    lidar_points_camera = (modified @ lidar_homogeneous.T).T  # (N, 4)


    # intrinsic matrix
    K = np.array([
        [1180.753804,  0.        , 934.859447],
        [ 0.        , 1177.034929, 607.266974],
        [ 0.        ,  0.        ,  1.       ]
    ], dtype=np.float32)

    # Extract 3D points in camera frame
    X_c = lidar_points_camera[:, 0]
    Y_c = lidar_points_camera[:, 1]
    Z_c = lidar_points_camera[:, 2]  # Depth

    # Avoid division by zero
    valid = Z_c > 0
    X_c, Y_c, Z_c = X_c[valid], Y_c[valid], Z_c[valid]

    # Project points onto image plane
    u = (K[0, 0] * X_c / Z_c) + K[0, 2]
    v = (K[1, 1] * Y_c / Z_c) + K[1, 2]

    # Filter points within image bounds
    img_h, img_w, _ = image.shape
    valid_pts = (u >= 0) & (u < img_w) & (v >= 0) & (v < img_h)
    u, v = u[valid_pts], v[valid_pts]
    dots.set_offsets(np.c_[u, v])
    plt.draw()


# register the update function with each slider
scale.on_changed(update)
x_rot.on_changed(update)
y_rot.on_changed(update)
z_rot.on_changed(update)
x_trans.on_changed(update)
y_trans.on_changed(update)
z_trans.on_changed(update)

plt.draw()
while keep_running:
    try:
        if plt.get_fignums():
            plt.pause(0.1)
        else:
            keep_running = False
    except:
        keep_running = False

modified = np.copy(T_lidar_camera)
rotation_mat = R.from_euler('xyz', [x_rot.val, y_rot.val, z_rot.val], degrees=True).as_matrix()
scale_mat = np.array([[scale.val, 0, 0], [0, scale.val, 0], [0, 0, scale.val]])
translation_vec = np.array([x_trans.val, y_trans.val, z_trans.val])
modified[:3, :3] = T_lidar_camera[:3, :3] @ rotation_mat @ scale_mat
modified[:3, 3] = T_lidar_camera[:3, 3] + translation_vec
print(modified)