#needed to import GEMstack from top level directory
import sys
import os
import cv2
sys.path.append(os.getcwd())
abs_path = os.path.abspath(os.path.dirname(__file__))

from GEMstack.onboard.perception.pedestrian_detection import PedestrianDetector, lidar_to_image, lidar_to_vehicle, filter_lidar_by_range
from GEMstack.onboard.interface.gem import GEMInterface
# from GEMstack.utils.mpl_visualization import plot_object
from GEMstack.utils.klampt_visualization import plot_object
from GEMstack.state import VehicleState
import numpy as np
import pathlib
from ultralytics import YOLO
import matplotlib.pyplot as plt
import random

import cv2
import os
import numpy as np
import math
import time

import argparse
parser = argparse.ArgumentParser()

parser.add_argument('--test_target', '-t', type=str, required=True, 
                    choices=['fuse', 'lidar_to_image', 'lidar_to_image_dbscan',
                             'detect_agents', 'box_to_agent', 'update'])
parser.add_argument('--output_dir', '-o', type=str, default='save')
parser.add_argument('--src_dir', '-s', type=str, default='./data/gt')
parser.add_argument('--data_idx', '-i', type=int, default=1)

args = parser.parse_args()

OUTPUT_DIR = args.output_dir

import matplotlib.pyplot as plt
from sklearn.cluster import DBSCAN

colors = []
colors += [(i, 0, 255) for i in range(100, 256, 25)]
colors += [(i, 255, 0) for i in range(100, 256, 25)]
colors += [(0, i, 255) for i in range(100, 256, 25)]
colors += [(255, i, 0) for i in range(100, 256, 25)]
colors += [(255, 0, i) for i in range(100, 256, 25)]
colors += [(0, 255, i) for i in range(100, 256, 25)]
seed = 19
random.seed(seed)
random.shuffle(colors)

id2str = {
    0: "pedestrian",
    2: "car",
    11: "stop sign"
}
    

class TestHelper:
    def __init__(self, ped_detector, point_cloud, zed_image):
        self.ped_detector = ped_detector
        self.yolo_detector = ped_detector.detector
        self.point_cloud = point_cloud
        self.zed_image = zed_image
    
    def test_fuse_lidar_image(self):
        detection_result = self.yolo_detector(self.zed_image,verbose=False)
        
        #TODO: create boxes from detection result
        boxes = []
        target_ids = [0, 2, 11]
        bbox_ids = []
        for box in detection_result[0].boxes: # only one image, so use index 0 of result
            class_id = int(box.cls[0].item())
            if class_id in target_ids: # class 0 stands for pedestrian
                bbox_ids.append(class_id)
                bbox = box.xywh[0].tolist()
                boxes.append(bbox)
            
        # Only keep lidar point cloud that lies in roi area for agents
        filtered_point_cloud = filter_lidar_by_range(self.point_cloud, 
                                                     self.ped_detector.xrange,
                                                     self.ped_detector.yrange)
        
        epsilon = 0.09  # Epsilon parameter for DBSCAN
        
        # for epsilon in np.linspace(0.01, 0.2, 10):
        # Perform DBSCAN clustering
        min_samples = 5  # Minimum number of samples in a cluster
        dbscan = DBSCAN(eps=epsilon, min_samples=min_samples)
        clusters = dbscan.fit_predict(filtered_point_cloud)

        point_cloud_image = lidar_to_image(filtered_point_cloud, 
                                        self.ped_detector.extrinsic, 
                                        self.ped_detector.intrinsic)

        # Tansfer lidar point cloud to vehicle frame
        pc_3D = lidar_to_vehicle(filtered_point_cloud, self.ped_detector.T_lidar2_Gem)

        vis = self.zed_image.copy()
        for i in range(len(boxes)):
            box = boxes[i]
            id = bbox_ids[i]
            print ("id:", id)
            print (box)
            
            x,y,w,h = box
            xmin, xmax = x - w/2, x + w/2
            ymin, ymax = y - h/2, y + h/2
            
            # Filter. Get the idxs of point cloud that belongs to the agent
            idxs = np.where((point_cloud_image[:, 0] > xmin) & (point_cloud_image[:, 0] < xmax) &
                            (point_cloud_image[:, 1] > ymin) & (point_cloud_image[:, 1] < ymax) )
            agent_image_pc = point_cloud_image[idxs]
            agent_pc_3D = pc_3D[idxs]
            agent_clusters = clusters[idxs]
            
            # draw bbox
            color =  (255, 0, 255)
            left_up = (int(x-w/2), int(y-h/2))
            right_bottom = (int(x+w/2), int(y+h/2))
            cv2.rectangle(vis, left_up, right_bottom, color, thickness=2)

            # Get unique elements and their counts
            unique_elements, counts = np.unique(agent_clusters, return_counts=True)
            max_freq = np.max(counts)
            label_cluster = unique_elements[counts == max_freq]
            
            # filter again
            idxs = agent_clusters == label_cluster
            agent_image_pc = agent_image_pc[idxs]
            agent_clusters = agent_clusters[idxs]
            agent_pc_3D = agent_pc_3D[idxs]
            
            # calulate depth
            depth = np.mean( (agent_pc_3D[:, 0] ** 2 + agent_pc_3D[:, 1] ** 2) ** 0.5 ) # euclidean dist
            print ('depth:', depth)
            
            # draw
            text = str(id2str[id])
            text += f" | depth: {depth:.2f}"
            text_size, baseline = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.4, 1)
            p1 = (left_up[0], left_up[1] - text_size[1])
            cv2.rectangle(vis, (p1[0] - 2 // 2, p1[1] - 2 - baseline), (p1[0] + text_size[0], p1[1] + text_size[1]),
                        color, -1)
            cv2.putText(vis, text, (p1[0], p1[1] + baseline), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1, 8)
            
            # draw point cloud
            for proj_pt, cluster in zip(agent_image_pc, agent_clusters):
                if cluster != label_cluster:
                    continue
                color = colors[cluster % len(colors)]
                radius = 1
                center = int(proj_pt[0]), int(proj_pt[1])
                vis = cv2.circle(vis, center, radius, color, cv2.FILLED)
            
            
        # output
        pathlib.Path(OUTPUT_DIR).mkdir(parents=True, exist_ok=True)
        output_path = os.path.join(OUTPUT_DIR, f'fuse_{epsilon}_{args.data_idx}.png')
        print ('Output lidar_to_image result:', output_path)
        cv2.imwrite(output_path, vis)
    
    def filter_lidar_by_range(self, point_cloud, xrange, yrange, zrange):
        xmin, xmax = xrange
        ymin, ymax = yrange
        zmin, zmax = zrange
        idxs = np.where((point_cloud[:, 0] > xmin) & (point_cloud[:, 0] < xmax) &
                        (point_cloud[:, 1] > ymin) & (point_cloud[:, 1] < ymax) &
                        (point_cloud[:, 2] > zmin) & (point_cloud[:, 2] < zmax))
        return point_cloud[idxs]

    def test_lidar_to_image_dbscan(self):
        # for i in np.linspace(1, 10, 10):
        print ('\nTest function lidar_to_image_dbscan()...')
        xrange = (0, 20)
        yrange = (-10, 10)
        zrange = (-3, 1)
        filtered_point_cloud = self.filter_lidar_by_range(self.point_cloud, 
                                                            xrange,
                                                            yrange,
                                                            zrange)
        
        epsilon = 0.1  # Epsilon parameter for DBSCAN
        
        # for epsilon in np.linspace(0.01, 0.2, 10):
        # Perform DBSCAN clustering
        min_samples = 5  # Minimum number of samples in a cluster
        dbscan = DBSCAN(eps=epsilon, min_samples=min_samples)
        clusters = dbscan.fit_predict(filtered_point_cloud)

        point_cloud_image = lidar_to_image(filtered_point_cloud, 
                                        self.ped_detector.extrinsic, 
                                        self.ped_detector.intrinsic)
    
        vis = self.zed_image.copy()
        for proj_pt, cluster in zip(point_cloud_image, clusters):
            color = colors[cluster % len(colors)]
            radius = 1
            center = int(proj_pt[0]), int(proj_pt[1])
            vis = cv2.circle(vis, center, radius, color, cv2.FILLED)
        
        pathlib.Path(OUTPUT_DIR).mkdir(parents=True, exist_ok=True)
        output_path = os.path.join(OUTPUT_DIR, f'lidar_to_image_{epsilon}_{args.data_idx}_x{xrange}_y{yrange}_z{zrange}.png')
        print ('Output lidar_to_image result:', output_path)
        cv2.imwrite(output_path, vis)
        
    def test_lidar_to_image(self):
        print ('\nTest function lidar_to_image()...')
        filtered_point_cloud = filter_lidar_by_range(self.point_cloud, 
                                                     self.ped_detector.xrange,
                                                     self.ped_detector.yrange)
        
        point_cloud_image = lidar_to_image(filtered_point_cloud, 
                                           self.ped_detector.extrinsic, 
                                           self.ped_detector.intrinsic)
        vis = self.zed_image.copy()
        for proj_pt in point_cloud_image:
            color = (0, 255, 0) # green
            radius = 1
            center = int(proj_pt[0]), int(proj_pt[1])
            vis = cv2.circle(vis, center, radius, color, cv2.FILLED)
        
        pathlib.Path(OUTPUT_DIR).mkdir(parents=True, exist_ok=True)
        output_path = os.path.join(OUTPUT_DIR, 'lidar_to_image.png')
        print ('Output lidar_to_image result:', output_path)
        cv2.imwrite(output_path, vis)

    def test_update(self):
        print ('\nTest function update()...')
        self.ped_detector.test_set_data(self.zed_image, self.point_cloud)
        self.ped_detector.update(VehicleState.zero())

    

if __name__=='__main__':
    extrinsic = np.array([[-0.00519, -0.99997, 0.005352, 0.1627], 
                 [-0.0675, -0.00499, -0.9977, -0.03123], 
                 [0.99771, -0.00554, -0.06743, -0.7284],
                 [0,       0 ,             0 ,      1]])
    
    """ Adjust extrinsic matrix """
    import math
    degrees = np.array([-0.5, -2.5, 0]) # xyz axis in degree
    translation = np.array([-0.25, 0, 0]) # xyz in meters
    
    # x axis
    angle = math.radians(degrees[0])
    rotation_matrix = np.array([[1, 0, 0],
                                [0, np.cos(angle), -np.sin(angle)],
                                [0, np.sin(angle), np.cos(angle)]])
    extrinsic[:3, :3] = rotation_matrix @ extrinsic[:3, :3]
    
    # y axis
    angle = math.radians(degrees[1])
    rotation_matrix = np.array([[np.cos(angle), 0, -np.sin(angle)],
                                [0, 1, 0],
                                [np.sin(angle), 0, np.cos(angle)]])
    extrinsic[:3, :3] = rotation_matrix @ extrinsic[:3, :3]
    
    # z axis
    angle = math.radians(degrees[2])
    rotation_matrix = np.array([[np.cos(angle), -np.sin(angle), 0],
                                [np.sin(angle), np.cos(angle), 0],
                                [0, 0, 1]])
    extrinsic[:3, :3] = rotation_matrix @ extrinsic[:3, :3]
    
    # translation
    extrinsic[:3, -1] = extrinsic[:3, -1] + translation
    # np.savetxt('new_extrinsic.txt', extrinsic)
    
    gem_interface = GEMInterface()
    ped_detector = PedestrianDetector(gem_interface, extrinsic)
    
    # load data
    lidar_fn = os.path.join(args.src_dir, f'lidar{args.data_idx}.npz')
    image_fn = os.path.join(args.src_dir, f'color{args.data_idx}.png')
    # depth_fn = os.path.join(args.src_dir, f'depth{args.data_idx}.tif')
    
    point_cloud = np.load(lidar_fn)['arr_0']
    image = cv2.imread(image_fn)
    # depth = cv2.imread(depth_fn)
    
    test_helper = TestHelper(ped_detector, point_cloud, image)
    
    if args.test_target == 'lidar_to_image':
        test_helper.test_lidar_to_image()
    if args.test_target == 'lidar_to_image_dbscan':
        test_helper.test_lidar_to_image_dbscan()
    
    print ('\nDone!')
    