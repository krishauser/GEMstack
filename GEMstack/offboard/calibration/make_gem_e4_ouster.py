#%%
import os
import sys
os.getcwd()
VIS = False

#%% things to extract
tx,ty,tz,rx,ry,rz = [None] * 6
#%%
#TODO make into command line arguments
pbg = '/mount/wp/GEMstack/data/lidar76.npz' #null scene
pbgAndLine = '/mount/wp/GEMstack/data/lidar78.npz'

#%% load and wash data
import numpy as np

bg = np.load(pbg)['arr_0'] # load null scene -- no added objects
bgAndLine = np.load(pbgAndLine)['arr_0'] # load data pc -- with added object
bg = bg[~np.all(bg == 0, axis=1)] # remove (0,0,0)'s
bgAndLine = bgAndLine[~np.all(bgAndLine == 0, axis=1)]

#%% visualize cropped null scene and data scene
import pyvista as pv
import panel as pn
import matplotlib.pyplot as plt
def vispc(pc):
    pv.set_jupyter_backend('client')
    cloud = pv.PolyData(pc)
    plotter = pv.Plotter(notebook=True)
    plotter.add_mesh(cloud, render_points_as_spheres=True, point_size=2, color='blue')
    plotter.camera.position = (-20,0,20)
    plotter.camera.focal_point = (0,0,0)
    plotter.show_axes()
    plotter.show()
if VIS:
    vispc(bg)
    vispc(bgAndLine)


#%%==============================================================
#======================= util functions =========================
#================================================================
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

from sklearn.linear_model import RANSACRegressor
from sklearn.linear_model import LinearRegression
def fit_plane_ransac(pc,tol=0.01):
    # fit a plane in a pointcloud 
    # and visualize inliers
    # pc: np array (N,3). the pointcloud
    # tol: the tolerance. default 0.01
    # return: float, float, float, array(N,3)
    # ^: coeff1, coeff2, intercept toward the plane, inliers of the fit
    model = RANSACRegressor(LinearRegression(), residual_threshold=tol)
    model.fit(pc[:,:2], pc[:,2]) 
    a, b = model.estimator_.coef_
    inter = model.estimator_.intercept_
    return a,b,inter,cropped_bg[model.inlier_mask_]

from scipy.spatial import cKDTree
def pc_diff(pc0,pc1,tol=0.1):
    # remove points in pc0 from pc1 s.t. euclidian distance is within tolerance
    # return: array(N,3)
    tree = cKDTree(pc0)
    diff = []
    for i, point in enumerate(pc1):
        _, idx = tree.query(point)
        distance = np.linalg.norm(point - pc0[idx])  # Compute the distance
        if distance > tol:  
            diff.append(point)
    # tree = cKDTree(pc1)
    # for i, point in enumerate(pc0):
    #     _, idx = tree.query(point)  
    #     distance = np.linalg.norm(point - pc1[idx])  # Compute the distance
    #     if distance > tol:  
    #         diff.append(point)
    return np.array(diff)



#%%==============================================================
#========================== tx ty rz ============================
#================================================================
#%% crop to only keep a frontal box area
area = (-0,10),(-2,2),(-3,1)
cropped_bg = crop(bg,*area)
cropped_bgAndLine = crop(bgAndLine,*area)
print(cropped_bg.shape)
print(cropped_bgAndLine.shape)

#%% Take difference to only keep added object
diff = pc_diff(cropped_bg,cropped_bgAndLine)
if VIS:
    vispc(diff) #visualize diff, hopefully the added objects

# %% use the added objects to find rz. 
# TODO after dataset retake
# right now we assume tx = ty = 0 and \
# just use median to find a headding direction
tx = ty = 0
hx,hy = np.median(diff,axis=0)[:2]
rz = -np.arctan2(hy,hx)




#%%==============================================================
#========================= tz rx ry =============================
#================================================================
# %% this time we crop to keep the ground
cropped_bg = crop(bg,iz = (-3,-2))
if VIS:
    print(cropped_bg.shape)
    vispc(cropped_bg)

#%%
from math import sqrt
a, b, height, inliers = fit_plane_ransac(cropped_bg,tol=0.001)
# TODO MAGIC NUMBER WARNING
real_tz = 203 #https://publish.illinois.edu/robotics-autonomy-resources/gem-e4/hardware/
real_height = 203 + 27.94 # 11 inches that we measured
tz = height * real_tz/real_height
if VIS:
    vispc(inliers)
normv = np.array([a, b, -1])
normv /= np.linalg.norm(normv)
nx,ny,nz = normv
ry = np.arctan2(-nx, sqrt(ny**2 + nz**2))
rx = np.arctan2(ny,sqrt(nx**2 + nz**2))





#%% visualize calibrated pointcloud
if VIS:
    from scipy.spatial.transform import Rotation as R
    rot = R.from_euler('xyz',[rx,ry,rz]).as_matrix()

    calibrated_bgAndLine = bgAndLine @ rot.T + [tx,ty,tz]

    vispc(calibrated_bgAndLine)
# %%
print(f"""
translation: ({tx,ty,tz})
rotation: ({rx,ry,rz})
""")
