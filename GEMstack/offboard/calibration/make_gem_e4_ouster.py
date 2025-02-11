#%%
import os
import sys
os.getcwd()

#%%
pbg = '/mount/wp/GEMstack/data/lidar76.npz'
pbgAndLine = '/mount/wp/GEMstack/data/lidar78.npz'

#%% load and wash data
import numpy as np
#load null scene -- no added objects
bg = np.load(pbg)['arr_0']
#load data pc -- with added object
bgAndLine = np.load(pbgAndLine)['arr_0']
#remove (0,0,0)'s
bg = bg[~np.all(bg == 0, axis=1)]
bgAndLine = bgAndLine[~np.all(bgAndLine == 0, axis=1)]

#%% crop to only keep a frontal box area
def crop(pc,ix=None,iy=None,iz=None):
    mask = True
    for dim,intervel in zip([0,1,2],[ix,iy,iz]):
        if not intervel: continue
        d,u = intervel
        mask &= pc[:,dim] >= d
        mask &= pc[:,dim] <= u 
        print(f'points left after cropping {dim}\'th dim',mask.sum())
    return pc[mask]
#
area = (-0,5),(-1,1)
bg = crop(bg,*area)
bgAndLine = crop(bgAndLine,*area)
print(bg.shape)
print(bgAndLine.shape)

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
    plotter.show()

vispc(bg)
vispc(bgAndLine)
#%% take difference to only keep added object
from scipy.spatial import cKDTree
tree = cKDTree(bg)

tolerance = 0.08
# Find points in pc1 that do not have a match in pc2 within the tolerance
idiff = []
for i, point in enumerate(bgAndLine):
    _, idx = tree.query(point)  # Find the nearest neighbor in pc2
    distance = np.linalg.norm(point - bg[idx])  # Compute the distance
    if distance > tolerance:  # If the nearest neighbor is outside the tolerance
        idiff.append(i)

diff = np.array(bgAndLine[diff])
#visualize
vispc(diff)
