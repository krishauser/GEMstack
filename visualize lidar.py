import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Load the .npz file
data = np.load('data/foam_block/lidar1.npz')

# Extract the point cloud data (assuming it's stored under the key 'points')
points = data['points']  # This might need to be adjusted based on the actual key

# Visualizing the point cloud data
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Assuming points is an Nx3 array where columns represent X, Y, and Z coordinates
ax.scatter(points[:,0], points[:,1], points[:,2])

plt.show()
