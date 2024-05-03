"""
Program for testing object detection using paired lidar & camera data.

Args:
    [folder]: path to the folder containing the paired scans
    [n]: scan number (eg: color[n].png, lidar[n].npz)

Usage: From the main GEMstack folder, run
            python3 testing/test_detection_paired_scan.py [folder] [n]
"""

#needed to import GEMstack from top level directory
import sys
import os
sys.path.append(os.getcwd())

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import cv2

from GEMstack.onboard.interface.gem import GEMInterface
from GEMstack.state import AgentState, Route, SceneState, Path, ObjectFrameEnum, SignEnum
from GEMstack.onboard.perception.agent_detection_v2 import AgentDetector
from GEMstack.onboard.perception.sign_detection import SignDetector

def run_detection_paired_scan(data_folder, idx): 
    lidar_path = os.path.join(data_folder, 'lidar{}.npz'.format(idx))
    rgb_path = os.path.join(data_folder, 'color{}.png'.format(idx)) 
    # depth_path = os.path.join(data_folder, 'depth{}.tif'.format(idx))
    
    agent_detector = AgentDetector(None)
    agent_detector.camera_image = cv2.imread(rgb_path)
    agent_detector.lidar_point_cloud = np.load(lidar_path)['arr_0']

    agent_states = agent_detector.update(None)

    print('Detected {} agents'.format(len(agent_states)))
    for i in range(len(agent_states)):
        print('Agent {}:'.format(i+1))
        s = agent_states[i]
        print('- position: ({0:.3f}, {1:.3f}, {2:.3f})'.format(s.pose.x, s.pose.y, s.pose.z))
        print('- dimensions: ({0:.3f}, {1:.3f}, {2:.3f})'.format(s.dimensions[0], s.dimensions[1], s.dimensions[2]))
        print('- type:', s.type)

    sign_detector = SignDetector(None)
    sign_detector.camera_image = cv2.imread(rgb_path)
    sign_detector.lidar_point_cloud = np.load(lidar_path)['arr_0']

    signs = sign_detector.update(None, None)

    print('Detected {} signs'.format(len(signs)))
    for type in signs:
        print('Sign {}:'.format(i+1))
        print('- type:', type)
        s = signs[type]
        print('- position: ({0:.3f}, {1:.3f}, {2:.3f})'.format(s.pose.x, s.pose.y, s.pose.z))
        print('- dimensions: ({0:.3f}, {1:.3f}, {2:.3f})'.format(s.dimensions[0], s.dimensions[1], s.dimensions[2]))
        

if __name__ == '__main__':
    data_folder = sys.argv[1]
    idx = sys.argv[2]
    run_detection_paired_scan(data_folder, idx)
