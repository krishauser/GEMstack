#%%
import cv2
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt
import numpy as np
from visualizer import visualizer
import argparse
import yaml

parser = argparse.ArgumentParser(description='Select corresponding lidar, color, depth files based on index')
parser.add_argument('--data_path', type=str, required=True, help='Path to the dataset')
parser.add_argument('--index', type=int, required=True, help='Index for selecting the files')
parser.add_argument('--config', type=str, required=True, help='Path to YAML configuration file')
args = parser.parse_args()
args = parser.parse_args()

# Construct file paths based on the provided index
lidar_path = f'{args.data_path}/lidar{args.index}.bin'
rgb_path = f'{args.data_path}/color{args.index}.jpg'
depth_path = f'{args.data_path}/depth{args.index}.png'

N = 8 #how many point pairs you want to select

img = cv2.imread(rgb_path, cv2.IMREAD_UNCHANGED)

lidar_points = np.load(lidar_path)['arr_0']
lidar_points = lidar_points[~np.all(lidar_points== 0, axis=1)] # remove (0,0,0)'s



# Load transformation parameters from YAML file
with open(args.config, 'r') as yaml_file:
    config = yaml.safe_load(yaml_file)

tx, ty, tz = config['position']
rot = np.array(config['rotation'])

# Construct transformation matrix
lidar_ex = np.hstack([rot, np.array([[tx], [ty], [tz]])])
lidar_ex = np.vstack([lidar_ex, [0, 0, 0, 1]])

camera_in = np.array([ # Update intrinsics if necessary
    [684.83331299,   0.        , 573.37109375],
    [  0.        , 684.60968018, 363.70092773],
    [  0.        ,   0.        ,   1.        ]
], dtype=np.float32)


#%%
# blurred = cv2.GaussianBlur(img, (5, 5), 0)
# edges = cv2.Canny(blurred, threshold1=50, threshold2=150)
# plt.imshow(edges)
plt.imshow(cv2.cvtColor(img,cv2.COLOR_BGR2RGB))

#%%
import pyvista as pv
def vis(title='', ratio=1,notebook=False):
    print(title)
    pv.set_jupyter_backend('client')

    return visualizer().set_cam()
def crop(pc,ix=None,iy=None,iz=None):
    # crop a subrectangle in a pointcloud
    # usage: crop(pc, ix = (x_min,x_max), ...)
    # return: array(N,3)
    mask = True
    for dim,intervel in zip([0,1,2],[ix,iy,iz]):
        if not intervel: continue
        d,u = intervel
        mask &= pc[:,dim] >= d
        mask &= pc[:,dim] <= u 
        print(f'points left after cropping {dim}\'th dim',mask.sum())
    return pc[mask]


lidar_post = np.pad(lidar_points,((0,0),(0,1)),constant_values=1) @ lidar_ex.T[:,:3]
lidar_post = crop(lidar_post,ix=(0,10),iy=(-5,5))
# vis(notebook=False).add_pc(lidar_post).show()

#%%
def pick_n_img(img,n=4):
    corners = []  # Reset the corners list
    def click_event(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            corners.append((x, y))
            cv2.circle(param, (x, y), 5, (0, 255, 0), -1)
            cv2.imshow('Image', param)
    
    cv2.imshow('Image', img)
    cv2.setMouseCallback('Image', click_event, img)
    
    while True:
        if len(corners) == n:
            break
        if cv2.waitKey(1) & 0xFF == ord('q'):
            return None
    
    cv2.destroyAllWindows()
    
    return corners
cpoints = np.array(pick_n_img(img,N)).astype(float)
print(cpoints)

#%%
def pick_n_pc(point_cloud,n=4):
    points = []
    def cb(pt,*args):
        points.append(pt)
    while len(points)!=n:
        points = []
        cloud = pv.PolyData(point_cloud)
        plotter = pv.Plotter(notebook=False)
        plotter.camera.position = (-20,0,20)
        plotter.camera.focal_point = (0,0,0)
        plotter.add_mesh(cloud, color='lightblue', point_size=5, render_points_as_spheres=True)
        plotter.enable_point_picking(cb)
        plotter.show()
    return points

lpoints = np.array(pick_n_pc(lidar_post,N))
print(lpoints)
# %%
success, rvec, tvec = cv2.solvePnP(lpoints, cpoints, camera_in, None)
R, _ = cv2.Rodrigues(rvec)

T = np.eye(4)
T[:3, :3] = R
T[:3, 3] = tvec.flatten()
print(T)


#%%
def depth_to_points(depth_img: np.ndarray, intrinsics: np.ndarray):
    """
    Convert a single-channel depth image into an Nx3 array of 3D points
    in the camera coordinate system.
    - depth_img: 2D array of type uint16 or float with depth values
    - intrinsics: 3x3 camera matrix
    """
    fx = intrinsics[0,0]
    fy = intrinsics[1,1]
    cx = intrinsics[0,2]
    cy = intrinsics[1,2]

    # Indices of each pixel
    h, w = depth_img.shape
    i_range = np.arange(h)
    j_range = np.arange(w)
    jj, ii = np.meshgrid(j_range, i_range)  # shape (h,w)

    # Flatten
    ii = ii.flatten().astype(np.float32)
    jj = jj.flatten().astype(np.float32)
    depth = depth_img.flatten().astype(np.float32)

    # Filter out zero / invalid depths
    valid_mask = (depth > 0)
    ii = ii[valid_mask]
    jj = jj[valid_mask]
    depth = depth[valid_mask]

    z = depth / 10000
    x = (jj - cx) * z / fx
    y = (ii - cy) * z / fy
    points3d = np.stack((x, y, z), axis=-1)  # shape (N,3)

    return points3d



depth_img = cv2.imread(depth_path, cv2.IMREAD_UNCHANGED)
camera_points = depth_to_points(depth_img, camera_in)

#%%
v2c = T
print('vehicle->camera:',v2c)
c2v = np.linalg.inv(v2c)
print('camera->vehicle:',c2v)

