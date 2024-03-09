import numpy as np
import yaml

if __name__ == '__main__':
    zed2lidar = np.loadtxt("GEMstack/knowledge/calibration/zed2lidar.txt")
    lidar2vechicle = np.loadtxt("GEMstack/knowledge/calibration/lidar2axle.txt")
    lidar2zed = np.linalg.inv(zed2lidar)
    zed2vechicle = np.dot(lidar2vechicle, zed2lidar)

        
    import pdb
    pdb.set_trace()