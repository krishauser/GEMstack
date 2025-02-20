#%%
import cv2
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt
import numpy as np

rgb_path = '/mount/wp/GEMstack/data/color21.png'
depth_path = '/mount/wp/GEMstack/data/depth21.tif'
lidar_path = '/mount/wp/GEMstack/data/lidar21.npz'

img = cv2.imread(rgb_path, cv2.IMREAD_UNCHANGED)

lidar_points = np.load(lidar_path)['arr_0']
lidar_points = lidar_points[~np.all(lidar_points== 0, axis=1)] # remove (0,0,0)'s

rx,ry,rz = 0.006898647163954201, 0.023800082245145304, -0.025318355743942974
<<<<<<< HEAD
tx,ty,tz = 1.1, 0.03773044170906172, 1.9525244316515322
=======
tx,ty,tz = 1.1, 0.037735827433173136, 1.953202227766785
>>>>>>> 3891eedef59a3b450d8beaead04b7309d36517be
rot = R.from_euler('xyz',[rx,ry,rz]).as_matrix()
lidar_ex = np.hstack([rot,[[tx],[ty],[tz]]])
lidar_ex = np.vstack([lidar_ex,[0,0,0,1]])

camera_in = np.array([
    [684.83331299,   0.        , 573.37109375],
    [  0.        , 684.60968018, 363.70092773],
    [  0.        ,   0.        ,   1.        ]
], dtype=np.float32)

N = 8

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
    plotter = pv.Plotter(notebook=notebook)
    plotter.show_axes()
    class foo:
        def set_cam(self,pos=(-20*ratio,0,20*ratio),foc=(0,0,0)):
            plotter.camera.position = pos
            plotter.camera.focal_point = foc
            return self
        def add_pc(self,pc,ratio=ratio,**kargs):
            plotter.add_mesh(
                pv.PolyData(pc*ratio), 
                render_points_as_spheres=True, 
                point_size=2,
                **kargs)
            return self
        def add_line(self,p1,p2,ratio=ratio,**kargs):
            plotter.add_mesh(
                pv.Line(p1*ratio,p2*ratio), 
                **kargs,
                line_width=1)
            return self
        def add_box(self,bound,trans,ratio=ratio):
            l,w,h = map(lambda x:x*ratio,bound)
            box = pv.Box(bounds=(-l/2,l/2,-w/2,w/2,-h/2,h/2))
            box = box.translate(list(map(lambda x:x*ratio,trans)))
            plotter.add_mesh(box, color='yellow')
            return self
        def show(self):
            plotter.show()
            return self
        def close(self):
            plotter.close()
            return None

    return foo().set_cam()
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
lidar_post = crop(lidar_post,ix=(0,8),iy=(-5,5))
vis(notebook=True).add_pc(lidar_post).show()

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

v=vis(notebook=False)
v.add_pc(np.pad(lidar_points,((0,0),(0,1)),constant_values=1)@lidar_ex.T@T.T[:,:3],color='blue')
v.add_pc(camera_points,color='red')
v.show()

#%%
print(np.vstack([(lidar_ex.T@T.T[:,:3]).T,[0,0,0,1]]))