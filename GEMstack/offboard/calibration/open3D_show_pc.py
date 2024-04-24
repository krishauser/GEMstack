import numpy as np
import open3d as o3d

import argparse
parser = argparse.ArgumentParser()

parser.add_argument('--x_range', type=str, default='-360,360', help='split by comma (ex: 0,30)')
parser.add_argument('--y_range', type=str, default='-360,360', help='split by comma (ex: 0,30)')
parser.add_argument('--z_range', type=str, default='-360,360', help='split by comma (ex: 0,30)')
args = parser.parse_args()

xmin, xmax = args.x_range.split(',')
ymin, ymax = args.y_range.split(',')
zmin, zmax = args.z_range.split(',')
xmin, xmax = float(xmin), float(xmax)
ymin, ymax = float(ymin), float(ymax)
zmin, zmax = float(zmin), float(zmax)
assert xmin < xmax
assert ymin < ymax
assert zmin < zmax

# Load .npz file using NumPy
data = np.load('data/lidar4.npz')

# Extract data from .npz file
# For example, if your .npz file contains point cloud data
# You can access it like this
points = data['arr_0']  # Assuming 'points' is the key for your point cloud data
points_stopsign = points[[24944, 25226, 25420, 25372, 25340, 25639, 25278, 25198, 25653, 25500, 25468, 25452, 25388, 25682, 25308, 25655, 25214, 25530, 25498, 25544, 25150, 25450, 24938, 25332, 25135, 25055, 25637, 25236, 25039, 25621, 24896, 25290, 25260, 25560, 25212, 25480, 25464, 25400, 25384, 25101, 25320, 25619, 25037, 25022, 25542, 25510, 25572, 25178, 25242, 24854, 25196, 25099, 24988, 25382, 25350, 24878, 25272, 25669, 25268, 25071, 25556, 24957, 25292, 25095, 25192, 25589, 25492, 24910, 25226, 25129, 24838, 25476, 25418, 25230, 25033, 24836, 25224, 25218, 25412, 25117, 25020, 25302, 25208, 25008, 25396, 25065, 25182, 25667, 24970, 25364, 25016, 25113, 25266, 25254, 25146, 25049, 24955, 25284, 24944, 25338, 25144, 25238, 25430, 25158, 25164, 25067, 24922, 25000, 24892, 25462, 25174, 25220, 25370, 25079, 24959, 25250]]
# points = points[25254:25667]
# print (points.shape)
# print ('xmin, xmax:', xmin, xmax)
# print ('ymin, ymax:', ymin, ymax)
# print ('zmin, zmax:', zmin, zmax)
# points = points[np.where((points[:, 0] > xmin) & (points[:, 0] < xmax))]
# points = points[np.where((points[:, 1] > ymin) & (points[:, 1] < ymax))]
# points = points[np.where((points[:, 2] > zmin) & (points[:, 2] < zmax))]

# Convert NumPy array to Open3D point cloud
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(points)

# Visualize the point cloud
o3d.visualization.draw_geometries([pcd])
