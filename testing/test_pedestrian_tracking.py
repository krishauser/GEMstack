# needed to import GEMstack from top level directory
import sys
import os
import cv2
sys.path.append(os.getcwd())
abs_path = os.path.abspath(os.path.dirname(__file__))

from GEMstack.onboard.perception.pedestrian_detection import PedestrianDetector
from GEMstack.onboard.perception.pedestrian_tracking import PedestrianTracker
from GEMstack.onboard.interface.gem import GEMInterface


from GEMstack.state import AgentEnum
import numpy as np
import pathlib


import argparse
parser = argparse.ArgumentParser()


parser.add_argument('--output_dir', '-o', type=str, default='save')
parser.add_argument('--src_dir', '-s', type=str, default='./data/gt')
parser.add_argument('--write_all', '-w', type=int, default=0)
parser.add_argument('--tracking_data', '-d', type=str, default='GEMstack/onboard/prediction/written_frames.txt')

args = parser.parse_args()

OUTPUT_DIR = args.output_dir

class TestHelper:
    def __init__(self, ped_detector, ped_tracker):
        self.ped_detector = ped_detector
        self.yolo_detector = ped_detector.detector
        self.ped_tracker = ped_tracker
   
    def test_track_agents(self, framenum=80):
        for i in range(1, framenum + 1):
            lidar_fn = os.path.join(args.src_dir, f'lidar{i}.npz')
            image_fn = os.path.join(args.src_dir, f'color{i}.png')
            depth_fn = os.path.join(args.src_dir, f'depth{i}.tif')
        
            point_cloud = np.load(lidar_fn)['arr_0']
            image = cv2.imread(image_fn)
            depth = cv2.imread(depth_fn)
            self.depth = depth

            self.ped_detector.test_set_data(image, point_cloud)

            detected_agents, detection_result = self.ped_detector.detect_agents(test=True)
            
            detected_pedestrians = [x for x in detected_agents if x.type==AgentEnum.PEDESTRIAN]
            
            tracking_results, matches = self.ped_tracker.track_agents(detected_agents)
            
            # Write recent frames to file or state variable
            self.ped_tracker.output_tracking_results()
            
            rev_matches = {v:k for k,v in matches.items()}

            
            bbox_image = image.copy()
            
            #TODO: create boxes from detection result
            pedestrian_boxes = []
            detected_ped_id = 0
            for box in detection_result[0].boxes: # only one image, so use index 0 of result
                class_id = int(box.cls[0].item())
                if class_id == 0: # class 0 stands for pedestrian
                    bbox = box.xywh[0].tolist()
                    pedestrian_boxes.append(bbox)

                    pid = rev_matches[detected_ped_id]
                    ag_state = tracking_results[pid]
        
                    # draw bbox
                    x,y,w,h = bbox
                    xmin, xmax = x - w/2, x + w/2
                    ymin, ymax = y - h/2, y + h/2
                    
                    # What our program measured before kalman
                    m = detected_pedestrians[detected_ped_id]
                    
                    bbox_image = cv2.rectangle(bbox_image, (int(xmin), int(ymin)), (int(xmax), int(ymax)), (0, 0, 255), 2) 
                    bbox_image = cv2.putText(
                        img = bbox_image,
                        # text = f"PID:{pid}, XY:{round(ag_state.pose.x, 2)},{round(ag_state.pose.y, 2)}, VELXY:{round(ag_state.velocity[0], 2)},{round(ag_state.velocity[1], 2)}",
                        text = f"PID:{pid}, XY:{round(ag_state.pose.x, 2)},{round(ag_state.pose.y, 2)}, mXY:{round(m.pose.x, 2)},{round(m.pose.y, 2)}",

                        org = (int(xmin) - 300, int(ymax)-10),
                        fontFace = cv2.FONT_HERSHEY_SIMPLEX,
                        fontScale = 0.5,
                        color = (0, 0, 255),
                        thickness = 2
                    )
                    detected_ped_id += 1
            
            pathlib.Path(OUTPUT_DIR).mkdir(parents=True, exist_ok=True)
            output_path = os.path.join(OUTPUT_DIR, f'bbox_image{i}.png')
            print ('Output image with bbox result:', output_path)
            cv2.imwrite(output_path, bbox_image)
            

if __name__=='__main__':
    extrinsic = [[-0.00519, -0.99997, 0.005352, 0.1627], 
                 [-0.0675, -0.00499, -0.9977, -0.03123], 
                 [0.99771, -0.00554, -0.06743, -0.7284],
                 [0,       0 ,             0 ,      1]]

    gem_interface = GEMInterface()
    ped_detector = PedestrianDetector(gem_interface, extrinsic)
    
    # Create PedTracker
    ped_tracker = PedestrianTracker(
        test=True,
        write_all=(args.write_all > 0),
        detection_file_name=args.tracking_data
    )

    test_helper = TestHelper(ped_detector, ped_tracker)

    test_helper.test_track_agents(framenum=100)

    print ('\nDone!')
