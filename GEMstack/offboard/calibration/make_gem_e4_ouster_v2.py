#%%
import os
import sys
import math
import numpy as np
from scipy.spatial.transform import Rotation as R
os.getcwd()
VIS = False

#%% things to extract
tx,ty,tz,rx,ry,rz = [None] * 6

#%%==============================================================
#======================= util functions =========================
#================================================================
import pyvista as pv
import panel as pn
import matplotlib.pyplot as plt
def vis(ratio = 1):
    pv.set_jupyter_backend('client')
    plotter = pv.Plotter(notebook=True)
    plotter.camera.position = (-20*ratio,0,20*ratio)
    plotter.camera.focal_point = (0,0,0)
    plotter.show_axes()
    class foo:
        def add_pc(self,pc,ratio=ratio,**kargs):
            plotter.add_mesh(
                pv.PolyData(pc*ratio), 
                render_points_as_spheres=True, 
                point_size=2,
                **kargs)
            return self
        def add_line(self,p1,p2,ratio=ratio):
            plotter.add_mesh(
                pv.Line(p1*ratio,p2*ratio), 
                color='red', 
                line_width=1)
            return self
        def add_box(self,bound,trans,ratio=ratio):
            l,w,h = map(lambda x:x*ratio,bound)
            box = pv.Box(bounds=(-l/2,l/2,-w/2,w/2,-h/2,h/2))
            box = box.translate(list(map(lambda x:x*ratio,trans)))
            plotter.add_mesh(box, color='yellow',show_edges=True)
            return self
        def show(self):
            plotter.show()
            return self

    return foo()
def load_scene(path):
    sc = np.load(path)['arr_0'] 
    sc = sc[~np.all(sc == 0, axis=1)] # remove (0,0,0)'s
    return sc
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
    # fit a line/plane/hyperplane in a pointcloud 
    # pc: np array (N,D). the pointcloud
    # tol: the tolerance. default 0.01
    model = RANSACRegressor(LinearRegression(), residual_threshold=tol)
    model.fit(pc[:,:-1], pc[:,-1]) 
    a = model.estimator_.coef_
    inter = model.estimator_.intercept_
    class foo:
        def plot(self):
            inliers = pc[model.inlier_mask_]
            if pc.shape[1] == 2:
                plt.scatter(pc[:,0], pc[:,1], color='blue', marker='o', s=1)
                plt.scatter(inliers[:,0], inliers[:,1], color='red', marker='o', s=1)
                x_line = np.linspace(0, max(pc[:,0]), 100).reshape(-1,1)
                plt.plot(x_line, x_line * a[0] + inter, color='red', label='Fitted Line')
            elif pc.shape[1] == 3:
                vis().add_pc(pc).add_pc(inliers,color='red').show()
            return self
            
        def results(self):
            # return: array(D-1), float, array(N,3)
            # ^: , coeffs, intercept toward the plane, inliers of the fit
            return a,inter
    return foo()

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
    tree = cKDTree(pc1)
    for i, point in enumerate(pc0):
        _, idx = tree.query(point)  
        distance = np.linalg.norm(point - pc1[idx])  # Compute the distance
        if distance > tol:  
            diff.append(point)
    return np.array(diff)






#%%==============================================================
#========================= tz rx ry =============================
#================================================================

#%% load scene for ground plane
sc = load_scene('/mount/wp/GEMstack/data/lidar1.npz')

# %% we crop to keep the ground
cropped_sc = crop(sc,iz = (-3,-2))
if VIS:
    print(cropped_sc.shape)
    vis().add_pc(cropped_sc,color='blue').show()

#%%
from math import sqrt
fit = fit_plane_ransac(cropped_sc,tol=0.01)
c, inter = fit.results()
normv = np.array([c[0], c[1], -1])
normv /= np.linalg.norm(normv)
nx,ny,nz = normv
height = nz * inter
# TODO MAGIC NUMBER WARNING
height_axel = 0.2794 # 11 inches that we measured
tz =  height - height_axel
if VIS:
    fit.plot()
ry = math.atan2(nx, sqrt(ny**2 + nz**2))
rx = math.atan2(-ny,sqrt(nx**2 + nz**2))



#%%==============================================================
#========================== tx ty rz ============================
#================================================================

if False: # True to use the diff method to extract object.
    # load data
    sc0 = load_scene('/mount/wp/GEMstack/data/lidar70.npz')
    sc1 = load_scene('/mount/wp/GEMstack/data/lidar78.npz')

    # crop to only keep a frontal box area
    area = (-0,7),(-1,1),(-3,1)
    cropped0 = crop(sc0,*area)
    cropped1 = crop(sc1,*area)
    print(cropped0.shape)
    print(cropped1.shape)

    # Take difference to only keep added object
    objects = pc_diff(cropped0,cropped1)
else: #False to use only cropping
    sc1 = load_scene('/mount/wp/GEMstack/data/lidar1.npz')

    rot = R.from_euler('xyz',[rx,ry,0]).as_matrix()

    objects = sc1 @ rot.T + [0,0,tz]

    # crop to only keep a frontal box area
    area = (-0,20),(-1,1),(0,1)
    objects = crop(objects,*area)
    print(objects.shape)


#%%
if VIS:
    vis().add_pc(objects,color='blue').show() #visualize diff, hopefully the added objects

# %% use the added objects to find rz. 
# TODO after dataset retake
# right now we assume tx = ty = 0 and \
# just use median to find a headding direction
fit = fit_plane_ransac(objects[:,:2],tol=0.6)
c,inter = fit.results()
if VIS:
    fit.plot()
# tx = ty = 0
# hx,hy = np.median(diff,axis=0)[:2]
# rz = -np.arctan2(hy,hx)
tx = - (2.56 - 1.46) # https://publish.illinois.edu/robotics-autonomy-resources/gem-e4/hardware/
ty = - inter
rz = - math.atan(c)

if VIS:
    from scipy.spatial.transform import Rotation as R
    rot = R.from_euler('xyz',[0,0,rz]).as_matrix()
    cal_sc1 = sc1 @ rot.T + [tx,ty,0]
    line = np.arange(0,100)/100*max(objects[:,0])
    line = np.stack((line,c[0]*line+inter,np.zeros(100)),axis=1)
    line = line @ rot.T + [tx,ty,0]
    vis().add_pc(cal_sc1,color='blue').add_pc(line,color='red').show()


#%% visualize calibrated pointcloud
if VIS:
    rot = R.from_euler('xyz',[rx,ry,rz]).as_matrix()

    cal_sc1 = sc1 @ rot.T + [tx,ty,tz]
    # projection
    # cal_sc1[:,2] = 0
    v = vis(ratio=100)
    v.add_pc(cal_sc1,color='blue')
    v.add_box((2.56,.61*2,2.03+height_axel),[2.56/2,0,(2.03+height_axel)/2])
    v.show()
# %%
print(f"""
translation: ({tx,ty,tz})
rotation: ({rx,ry,rz})
""")


