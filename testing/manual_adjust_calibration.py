# needed to import GEMstack from top level directory
import sys
import os
import cv2
import argparse
import random
import numpy as np
from typing import Dict, Tuple, List
import pathlib
import os
import cv2
import math
sys.path.append(os.getcwd())
abs_path = os.path.abspath(os.path.dirname(__file__))

from GEMstack.onboard.perception.pixelwise_3D_lidar_coord_handler import PixelWise3DLidarCoordHandler
from manual_adjust_calibration_config import *
from ultralytics.utils.plotting import Annotator, colors
from ultralytics import YOLO
from collections import defaultdict


parser = argparse.ArgumentParser()

parser.add_argument('--src_dir', '-s', type=str, default='./data/gt')
parser.add_argument('--lidar2oak_fn', type=str, 
                    default="GEMstack/knowledge/calibration/gem_e4_lidar2oak.txt")
parser.add_argument('--lidar2vehicle_fn', type=str, 
                    default="GEMstack/knowledge/calibration/gem_e4_lidar2vehicle.txt")
parser.add_argument('--data_idx', '-i', type=int, default=1)
parser.add_argument('--kernel_size', '-k', type=int, default=5)
args = parser.parse_args()

OUTPUT_DIR = "save"

from sklearn.cluster import DBSCAN
def vis_dbscan(point_cloud_image, vis, epsilon = 0.1, min_samples = 5):
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

    # for epsilon in np.linspace(0.01, 0.2, 10):
    # Perform DBSCAN clustering
    dbscan = DBSCAN(eps=epsilon, min_samples=min_samples)
    clusters = dbscan.fit_predict(filtered_point_cloud)

    for proj_pt, cluster in zip(point_cloud_image, clusters):
        color = colors[cluster % len(colors)]
        radius = 1
        center = int(proj_pt[0]), int(proj_pt[1])
        cv2.circle(vis, center, radius, color, cv2.FILLED)
                
                
# return boxes, class_ids
def vis_bbox_3D(boxes, clss, frame, coord_3d_map, names):
    annotator = Annotator(frame, line_width=2)
    detect_ids = [0]
    for box, cls in zip(boxes, clss):
        if cls not in detect_ids:
            continue
        
        color = colors(int(cls), True)
        class_name = names[int(cls)]
        label = f"{class_name} "
        
        x1,y1,x2,y2 = box
        center_x = (x1 + x2) / 2
        center_y = (y1 + y2) / 2
        
        width = x2 - x1
        height = y2 - y1
        x1s = int(x1 + width // 4)
        x2s = int(x1 + width // 4 * 3)
        y1s = int(y1 + height // 4)
        y2s = int(y1 + height // 4 * 3)
        
        cv2.rectangle(frame, (x1s, y1s), (x2s, y2s), (0, 255, 0), thickness=2)
        
        small_coord_3d_map = coord_3d_map[y1s:y2s, x1s:x2s]
        coord_3d = [0, 0, 0]
        for i in range(3): # channels
            coord_3d[i] = np.median(small_coord_3d_map[:, :, i])
        
        x_3d, y_3d, z_3d = coord_3d
        
        # x_3d, y_3d, z_3d = coord_3d_map[int(center_y)][int(center_x)]
        label = label + f' x={x_3d:.2f}, y={y_3d:.2f}, z={z_3d:.2f}'
        annotator.box_label(box, color=color, label=label)
    
def adjust_matrix(extrinsic, degrees, translation):
    
    # x axis
    angle = math.radians(degrees[0]) # convert degree to radian
    rotation_matrix_x = np.array([[1, 0, 0],
                                  [0, np.cos(angle), -np.sin(angle)],
                                  [0, np.sin(angle), np.cos(angle)]])
    # y axis
    angle = math.radians(degrees[1])
    rotation_matrix_y = np.array([[np.cos(angle), 0, -np.sin(angle)],
                                  [0, 1, 0],
                                  [np.sin(angle), 0, np.cos(angle)]])
    
    # z axis
    angle = math.radians(degrees[2])
    rotation_matrix_z = np.array([[np.cos(angle), -np.sin(angle), 0],
                                  [np.sin(angle), np.cos(angle), 0],
                                  [0, 0, 1]])
    extrinsic[:3, :3] = rotation_matrix_z @ rotation_matrix_y @ rotation_matrix_x @ extrinsic[:3, :3]
    
    # translation
    extrinsic[:3, -1] = extrinsic[:3, -1] + translation

    return extrinsic

if __name__ == '__main__':

    # load lidar2oak matrix
    lidar2oak_mat = np.loadtxt(args.lidar2oak_fn)
    degrees = np.array([lidar2oak_degree_x, lidar2oak_degree_y, lidar2oak_degree_z]) # xyz axis in degree
    translation = np.array([lidar2oak_trans_x, lidar2oak_trans_y, lidar2oak_trans_z]) # xyz in meters
    lidar2oak_mat = adjust_matrix(lidar2oak_mat, degrees, translation)
    print ('lidar2oak_mat:')
    print (lidar2oak_mat)
    
    # load lidar2vehicle matrix
    lidar2vehicle_mat = np.loadtxt(args.lidar2vehicle_fn)
    degrees = np.array([lidar2vehicle_degree_x, lidar2vehicle_degree_y, lidar2vehicle_degree_z]) # xyz axis in degree
    translation = np.array([lidar2vehicle_trans_x, lidar2vehicle_trans_y, lidar2vehicle_trans_z]) # xyz in meters
    lidar2vehicle_mat = adjust_matrix(lidar2vehicle_mat, degrees, translation)
    print ('lidar2vehicle_mat:')
    print (lidar2vehicle_mat)
    
    
    # load data
    lidar_fn = os.path.join(args.src_dir, f'lidar{args.data_idx}.npz')
    image_fn = os.path.join(args.src_dir, f'color{args.data_idx}.png')

    point_cloud = np.load(lidar_fn)['arr_0']
    image = cv2.imread(image_fn)

    print('\nStart testing...')

    # init handler
    handler = PixelWise3DLidarCoordHandler(args.kernel_size, 
                                           lidar2oak_mat=lidar2oak_mat,
                                           lidar2vehicle_mat=lidar2vehicle_mat)
    coord_3d_map = handler.get3DCoord(image, point_cloud)
    
    # track agents
    yolo_path = os.path.join(abs_path, '../GEMstack/knowledge/detection/yolov8s.pt')
    detector = YOLO(yolo_path)
    # track_agents(detector, image, coord_3d_map)
    
    # detect bbox
    results = detector.track(image, persist=True, verbose=False)
    boxes = results[0].boxes.xyxy.cpu()

    names = detector.model.names
    if results[0].boxes.id is not None:
        clss = results[0].boxes.cls.cpu().tolist()
        track_ids = results[0].boxes.id.int().cpu().tolist()
        
    # get point_cloud_image
    handler.filter_lidar_by_range(point_cloud)
    filtered_point_cloud = handler.filter_lidar_by_range(point_cloud)
    point_cloud_image = handler.lidar_to_image(filtered_point_cloud)

    vis_dbscan(point_cloud_image, image)
    
    # vis bbox & 3D coord
    vis_bbox_3D(boxes, clss, image, coord_3d_map, detector.model.names)
    
    # output
    pathlib.Path(OUTPUT_DIR).mkdir(parents=True, exist_ok=True)
    output_path = os.path.join(OUTPUT_DIR, 'manual_calibrate.png')
    print ('Output image with bbox result:', output_path)
    cv2.imwrite(output_path, image)

