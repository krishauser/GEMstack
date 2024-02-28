#needed to import GEMstack from top level directory
import sys
import os
import cv2
sys.path.append(os.getcwd())
abs_path = os.path.abspath(os.path.dirname(__file__))

from GEMstack.onboard.perception.pedestrian_detection import PedestrianDetector, lidar_to_image, lidar_to_vehicle, filter_lidar_by_range
from GEMstack.onboard.interface.gem import GEMInterface
import numpy as np
import pathlib

OUTPUT_DIR = 'output'

class TestHelper:
    def __init__(self, ped_detector, point_cloud, zed_img):
        self.ped_detector = ped_detector
        self.point_cloud = point_cloud
        self.zed_img = zed_img

    def test_lidar_to_image(self):
        print ('\nTest function lidar_to_image()...')
        filtered_point_cloud = filter_lidar_by_range(self.point_cloud, 
                                                     self.ped_detector.xrange,
                                                     self.ped_detector.yrange)
        
        point_cloud_image = lidar_to_image(filtered_point_cloud, 
                                           self.ped_detector.extrinsic, 
                                           self.ped_detector.intrinsic)
        vis = self.zed_img.copy()
        for proj_pt in point_cloud_image:
            color = (0, 255, 0) # green
            radius = 1
            center = int(proj_pt[0]), int(proj_pt[1])
            vis = cv2.circle(vis, center, radius, color, cv2.FILLED)
        
        pathlib.Path(OUTPUT_DIR).mkdir(parents=True, exist_ok=True)
        output_path = os.path.join(OUTPUT_DIR, 'lidar_to_image.png')
        print ('Output lidar_to_image result:', output_path)
        cv2.imwrite(output_path, vis)

    def test_detect_agents(self):
        print ('\nTest function detect_agents()...')
        self.ped_detector.test_set_data(self.zed_img, self.point_cloud)
        self.ped_detector.detect_agents()

if __name__=='__main__':
    gem_interface = GEMInterface()
    ped_detector = PedestrianDetector(gem_interface)
    
    data_idx = 10
    point_cloud = np.load(os.path.join(abs_path, f'../data/hw3/step1/lidar{data_idx}.npz'))
    point_cloud = point_cloud['arr_0']

    zed_img = cv2.imread(os.path.join(abs_path, f'../data/hw3/step1/color{data_idx}.png'))
    
    test_helper = TestHelper(ped_detector, point_cloud, zed_img)
    test_helper.test_lidar_to_image()
    test_helper.test_detect_agents()
    
    print ('\nDone!')
    