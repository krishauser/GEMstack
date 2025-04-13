#%%
import sys
import math
import numpy as np
from scipy.spatial.transform import Rotation as R
import pyvista as pv
import matplotlib.pyplot as plt
from tools.visualizer import visualizer

VIS = False # True to show visuals
VIS = True # True to show visuals

#%% things to extract
tx,ty,tz,rx,ry,rz = [None] * 6

#%%==============================================================
#======================= util functions =========================
#================================================================
def vis(title='', ratio=1):
    print(title)
    pv.set_jupyter_backend('client')
    return visualizer().set_cam()
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
    class visual:
        def plot(self):
            inliers = pc[model.inlier_mask_]
            if pc.shape[1] == 2:
                plt.title('ransac fitting line')
                plt.scatter(pc[:,0], pc[:,1], color='blue', marker='o', s=1)
                plt.scatter(inliers[:,0], inliers[:,1], color='red', marker='o', s=1)
                x_line = np.linspace(0, max(pc[:,0]), 100).reshape(-1,1)
                plt.plot(x_line, x_line * a[0] + inter, color='red', label='Fitted Line')
            elif pc.shape[1] == 3:
                vis('ransac fitting a plane').add_pc(pc).add_pc(inliers,color='red').show()
            return self
            
        def results(self):
            # return: array(D-1), float, array(N,3)
            # ^: , coeffs, intercept toward the plane, inliers of the fit
            return a,inter
    return visual()

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
# Update depending on where data is stored
sc = load_scene('./data/lidar1.npz')

# %% we crop to keep the ground
cropped_sc = crop(sc,iz = (-3,-2))
if VIS:
    vis('ground points cropped').add_pc(cropped_sc,color='blue').show()

#%%
from math import sqrt
fit = fit_plane_ransac(cropped_sc,tol=0.01) # small tol because the ground is very flat
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
rx = -math.atan2(ny,-nz)
ry = -math.atan2(-nx,-nz)


if VIS:
    from scipy.spatial.transform import Rotation as R
    rot = R.from_euler('xyz',[rx,ry,0]).as_matrix()
    cal_sc = sc @ rot.T + [0,0,tz]
    vis('yz projection').add_pc(cal_sc*[0,1,1],color='blue').show()
    vis('xz projection').add_pc(cal_sc*[1,0,1],color='blue').show()

#%%==============================================================
#========================== tx ty rz ============================
#================================================================

rot = R.from_euler('xyz',[rx,ry,0]).as_matrix()

if False: # True to use the diff method to extract object.
    # load data: update depending on where data is stored
    sc0 = load_scene('./data/lidar70.npz')
    sc1 = load_scene('./data/lidar78.npz')

    sc0 = sc0 @ rot.T + [0,0,tz]
    sc1 = sc1 @ rot.T + [0,0,tz]

    # crop to only keep a frontal box area
    area = (-0,7),(-1,1),(-3,1)
    cropped0 = crop(sc0,*area)
    cropped1 = crop(sc1,*area)
    print(cropped0.shape)
    print(cropped1.shape)

    # Take difference to only keep added object
    objects = pc_diff(cropped0,cropped1)

else: #False to use only cropping
    # Update depending on where data is stored
    sc1 = load_scene('./data/lidar1.npz')

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
fit = fit_plane_ransac(objects[:,:2],tol=1) # tol=1 because 1m^3 foam boxes
c,inter = fit.results()
if VIS:
    fit.plot()
# tx = ty = 0
# hx,hy = np.median(diff,axis=0)[:2]
# rz = -np.arctan2(hy,hx)
rz = - math.atan(c)
tx = 2.56 - 1.46 # https://publish.illinois.edu/robotics-autonomy-resources/gem-e4/hardware/
ty = - inter * math.cos(rz)

if VIS:
    p1 = (0,inter,0)
    p2 = max(objects[:,0])*np.array([1,c[0],0])+np.array([0,inter,0])
    vis().add_pc(sc1*np.array([1,1,0]),color='blue').add_line(p1,p2,color='red').show()

    from scipy.spatial.transform import Rotation as R
    rot = R.from_euler('xyz',[0,0,rz]).as_matrix()
    cal_sc1 = sc1 @ rot.T + [tx,ty,0]
    vis().add_pc(cal_sc1,color='blue').show()


#%% visualize calibrated pointcloud
if VIS:
    rot = R.from_euler('xyz',[rx,ry,rz]).as_matrix()

    cal_sc1 = sc1 @ rot.T + [tx,ty,tz]
    v = vis(ratio=100)
    proj = [1,1,1]
    v.add_pc(cal_sc1*proj,color='blue')
    v.add_box((2.56,.61*2,2.03+height_axel),[2.56/2,0,(2.03+height_axel)/2])
    v.show() 
    # the yellow box should be 11 inches above the ground
    # rear-axel center should be at (0,0,0)
# %%
print(f"""
translation: ({tx,ty,tz})
rotation: ({rx,ry,rz})
""")



# %%
