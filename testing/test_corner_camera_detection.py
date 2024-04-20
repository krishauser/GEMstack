#needed to import GEMstack from top level directory
import sys
import os
import cv2
sys.path.append(os.getcwd())
abs_path = os.path.abspath(os.path.dirname(__file__))

from GEMstack.onboard.perception.pedestrian_detection_corner import CornerDetector, lidar_to_image, lidar_to_vehicle, filter_lidar_by_range
from GEMstack.onboard.interface.gem import GEMInterface
# from GEMstack.utils.mpl_visualization import plot_object
from GEMstack.utils.klampt_visualization import plot_object
from GEMstack.state import VehicleState
import numpy as np
import pathlib
from ultralytics import YOLO
import matplotlib.pyplot as plt
import random

# for klampt
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

import argparse
parser = argparse.ArgumentParser()

parser.add_argument('--test_target', '-t', type=str, required=True, 
                    choices=['fuse', 'lidar_to_image', 'lidar_to_image_dbscan',
                             'detect_agents', 'box_to_agent', 'update'])
parser.add_argument('--output_dir', '-o', type=str, default='save')
parser.add_argument('--src_dir', '-s', type=str, default='./data/check_calib/fr')
parser.add_argument('--data_idx', '-i', type=int, default=1)

args = parser.parse_args()

OUTPUT_DIR = args.output_dir

def klampt_vis(zed_image, lidar_point_cloud, depth):
    
    zed_K = [527.5779418945312, 0.0, 616.2459716796875, 0.0, 527.5779418945312, 359.2155456542969, 0.0, 0.0, 1.0]
    zed_K = np.array(zed_K).reshape((3, 3))
    zed_intrinsics = [zed_K[0,0],zed_K[1,1],zed_K[0,2],zed_K[1,2]]
    zed_w = 1920
    zed_h = 1200


    lidar_xform = se3.identity()
    zed_xform = (so3.from_ndarray(np.array([[0,0,1],[-1,0,0],[0,-1,0]])),[0,0,0])
    data = {}
    def load_and_show_scan(idx, depth=depth):
        pc = numpy_convert.from_numpy(lidar_point_cloud,'PointCloud')
        pc = colorize(pc,'z','plasma')
        data['lidar'] = Geometry3D(pc)

        depth = depth.astype(np.float32)
        print("depth range",np.min(depth),np.max(depth))
        zed_xfov = 2*np.arctan(zed_w/(2*zed_intrinsics[0]))
        zed_yfov = 2*np.arctan(zed_h/(2*zed_intrinsics[1]))
        print("estimated zed horizontal FOV",math.degrees(zed_xfov),"deg")
        pc = image_to_points(depth,zed_image,zed_xfov,zed_yfov,depth_scale=4000.0/0xffff, points_format='PointCloud')

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
    def __init__(self, ped_detector, point_cloud, zed_image, depth):
        self.ped_detector = ped_detector
        self.yolo_detector = ped_detector.detector
        self.point_cloud = point_cloud
        self.zed_image = zed_image
        self.depth = depth
    
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

        print("ext", self.ped_detector.extrinsic)
        print("int", self.ped_detector.intrinsic)
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
        
    def test_lidar_to_image_dbscan(self):
        print ('\nTest function lidar_to_image_dbscan()...')
        filtered_point_cloud = filter_lidar_by_range(self.point_cloud, 
                                                     self.ped_detector.xrange,
                                                     self.ped_detector.yrange)
        
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
        output_path = os.path.join(OUTPUT_DIR, f'lidar_to_image_{epsilon}_{args.data_idx}.png')
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

    def test_detect_agents(self):
        print ('\nTest function detect_agents()...')
        self.ped_detector.test_set_data(self.zed_image, self.point_cloud)
        self.ped_detector.detect_agents()

    def test_box_to_agent(self):
        print ('\nTest function box_to_agent()...')
        self.ped_detector.test_set_data(self.zed_image, self.point_cloud)
        
        yolo_path = os.path.join(abs_path, '../GEMstack/knowledge/detection/yolov8n.pt')
        detector = YOLO(yolo_path)
        detection_result = detector(self.zed_image,verbose=False)
        
        bbox_image = self.zed_image.copy()
        
        #TODO: create boxes from detection result
        pedestrian_boxes = []
        for box in detection_result[0].boxes: # only one image, so use index 0 of result
           class_id = int(box.cls[0].item())
           if class_id == 0: # class 0 stands for pedestrian
               bbox = box.xywh[0].tolist()
               pedestrian_boxes.append(bbox)
 
               # draw bbox
               x,y,w,h = bbox
               xmin, xmax = x - w/2, x + w/2
               ymin, ymax = y - h/2, y + h/2
               bbox_image = cv2.rectangle(bbox_image, (int(xmin), int(ymin)), (int(xmax), int(ymax)), (0, 0, 255), 2) 
        
        pathlib.Path(OUTPUT_DIR).mkdir(parents=True, exist_ok=True)
        output_path = os.path.join(OUTPUT_DIR, 'bbox_image.png')
        print ('Output image with bbox result:', output_path)
        cv2.imwrite(output_path, bbox_image)
    
        # Only keep lidar point cloud that lies in roi area for agents
        point_cloud_lidar = filter_lidar_by_range(ped_detector.point_cloud, 
                                                  ped_detector.xrange, 
                                                  ped_detector.yrange)
        
        
        # Perform DBSCAN clustering
        epsilon = 0.1  # Epsilon parameter for DBSCAN
        min_samples = 5  # Minimum number of samples in a cluster
        dbscan = DBSCAN(eps=epsilon, min_samples=min_samples)
        clusters = dbscan.fit_predict(point_cloud_lidar)

        # Tansfer lidar point cloud to camera frame
        point_cloud_image = lidar_to_image(point_cloud_lidar, ped_detector.extrinsic, ped_detector.intrinsic)
        
        # Tansfer lidar point cloud to vehicle frame
        point_cloud_image_world = lidar_to_vehicle(point_cloud_lidar, ped_detector.T_lidar2_Gem)

        # Find agents
        detected_agents = []
        print ('Detected {} persons'.format(len(pedestrian_boxes)))
        for i,b in enumerate(pedestrian_boxes):
            agent = ped_detector.box_to_agent(b, point_cloud_image, point_cloud_image_world, clusters)
        #     plot_object('agent', agent)
        # klampt_vis(self.zed_image, point_cloud_lidar, self.depth)
          


if __name__=='__main__':
    extrinsic = [[-0.7754, -0.6314, 0.00231, 0.33155 ],
                 [ -0.154711, 0.186429, -0.97021, 0.073365],
                 [ 0.61219, -0.75284, -0.2422,  -1.014598],
                 [0,       0 ,             0 ,      1]]
    
   
    

    gem_interface = GEMInterface()
    ped_detector = CornerDetector(gem_interface, extrinsic)
    
    
    # load data
    lidar_fn = os.path.join(args.src_dir, f'lidar{args.data_idx}.npz')
    image_fn = os.path.join(args.src_dir, f'color{args.data_idx}.png')
    depth_fn = os.path.join(args.src_dir, f'depth{args.data_idx}.tif')
    
    point_cloud = np.load(lidar_fn)['arr_0']
    image = cv2.imread(image_fn)
    depth = cv2.imread(depth_fn)
    
    test_helper = TestHelper(ped_detector, point_cloud, image, depth)
    
    if args.test_target == 'fuse':
        test_helper.test_fuse_lidar_image()

    if args.test_target == 'lidar_to_image':
        test_helper.test_lidar_to_image()
    if args.test_target == 'lidar_to_image_dbscan':
        test_helper.test_lidar_to_image_dbscan()
    if args.test_target == 'detect_agents':
        test_helper.test_detect_agents()
    if args.test_target == 'box_to_agent':
        test_helper.test_box_to_agent()
    if args.test_target == 'update':
        test_helper.test_update()
    
    print ('\nDone!')
    