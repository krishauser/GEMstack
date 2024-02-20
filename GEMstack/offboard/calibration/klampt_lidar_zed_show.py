from klampt import vis
from klampt.math import so3,se3
from klampt.vis.colorize import colorize
from klampt import PointCloud,Geometry3D
from klampt.io import numpy_convert
from klampt.model.sensing import image_to_points
import cv2
import os
import numpy as np
import math
import time

#uncalibrated values -- TODO: load these from a calibration file
zed_K = np.array([[550, 0.0, 640],
                [0.0, 550, 360],
                [0.0, 0.0, 1.0]])
zed_intrinsics = [zed_K[0,0],zed_K[1,1],zed_K[0,2],zed_K[1,2]]
zed_w = 1280
zed_h = 720


def main(folder):
    lidar_xform = se3.identity()
    zed_xform = (so3.from_ndarray(np.array([[0,0,1],[-1,0,0],[0,-1,0]])),[0,0,0])
    lidar_pattern = os.path.join(folder,"lidar{}.npz")
    color_pattern = os.path.join(folder,"color{}.png")
    depth_pattern = os.path.join(folder,"depth{}.tif")
    data = {}
    def load_and_show_scan(idx):
        arr_compressed = np.load(lidar_pattern.format(idx))
        arr = arr_compressed['arr_0']
        arr_compressed.close()
        pc = numpy_convert.from_numpy(arr,'PointCloud')
        pc = colorize(pc,'z','plasma')
        data['lidar'] = Geometry3D(pc)

        color = cv2.imread(color_pattern.format(idx))
        depth = cv2.imread(depth_pattern.format(idx),cv2.IMREAD_UNCHANGED)
        depth = depth.astype(np.float32)
        print("depth range",np.min(depth),np.max(depth))
        zed_xfov = 2*np.arctan(zed_w/(2*zed_intrinsics[0]))
        zed_yfov = 2*np.arctan(zed_h/(2*zed_intrinsics[1]))
        print("estimated zed horizontal FOV",math.degrees(zed_xfov),"deg")
        pc = image_to_points(depth,color,zed_xfov,zed_yfov,depth_scale=4000.0/0xffff, points_format='PointCloud')

        data['zed'] = Geometry3D(pc)
        data['lidar'].setCurrentTransform(*lidar_xform)
        data['zed'].setCurrentTransform(*zed_xform)
        vis.add('lidar',data['lidar'])
        vis.add('zed',data['zed'])

    data['index'] = 1
    def increment_index():
        data['index'] += 1
        try:
            load_and_show_scan(data['index'])
        except Exception:
            data['index'] -= 1
            return
    def decrement_index():
        data['index'] -= 1
        try:
            load_and_show_scan(data['index'])
        except Exception:
            data['index'] += 1
            return
    def print_xforms():
        print("lidar:")
        print("rotation:",so3.ndarray(lidar_xform[0]))
        print("position:",lidar_xform[1])
        print("zed:")
        print("rotation:",so3.ndarray(zed_xform[0]))
        print("position:",zed_xform[1])


    vis.addAction(increment_index,"Increment index",'=')
    vis.addAction(decrement_index,"Decrement index",'-')
    vis.addAction(print_xforms,'Print transforms','p')
    load_and_show_scan(1)
    vis.add('zed_xform',zed_xform)
    vis.add('lidar_xform',lidar_xform)
    vis.edit('zed_xform')
    vis.edit('lidar_xform')
    vis.show()
    while vis.shown():
        lidar_xform = vis.getItemConfig('lidar_xform')
        lidar_xform = lidar_xform[:9],lidar_xform[9:]
        zed_xform = vis.getItemConfig('zed_xform')
        zed_xform = zed_xform[:9],zed_xform[9:]
        data['lidar'].setCurrentTransform(*lidar_xform)
        data['zed'].setCurrentTransform(*zed_xform)
        time.sleep(0.02)
    vis.kill()

if __name__ == '__main__':
    import sys
    folder = 'data'
    if len(sys.argv) >= 2:
        folder = sys.argv[1]
    if len(sys.argv) >= 3:
        calib = sys.argv[2]
        import yaml
        with open(calib,'r') as f:
            config = yaml.load(f,yaml.SafeLoader)
            zed_K = np.array(config['K']).reshape((3,3))
            zed_intrinsics = [zed_K[0,0],zed_K[1,1],zed_K[0,2],zed_K[1,2]]
            zed_w = config['width']
            zed_height = config['height']
    main(folder)
