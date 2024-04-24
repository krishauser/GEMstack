from ...state import AllState,VehicleState,ObjectPose,ObjectFrameEnum,AgentState,AgentEnum,AgentActivityEnum
from ..interface.gem import GEMInterface
from ..component import Component
from ultralytics import YOLO
import cv2
from typing import Dict
import numpy as np
from ..utils import settings

def box_to_fake_agent(box):
    """Creates a fake agent state from an (x,y,w,h) bounding box.
    
    The location and size are pretty much meaningless since this is just giving a 2D location.
    """
    x,y,w,h = box
    pose = ObjectPose(t=0,x=x+w/2,y=y+h/2,z=0,yaw=0,pitch=0,roll=0,frame=ObjectFrameEnum.CURRENT)
    dims = (w,h,0)
    return AgentState(pose=pose,dimensions=dims,outline=None,type=AgentEnum.PEDESTRIAN,activity=AgentActivityEnum.MOVING,velocity=(0,0,0),yaw_rate=0)


class PedestrianDetector2D(Component):
    """Detects pedestrians."""
    def __init__(self,vehicle_interface : GEMInterface):
        self.vehicle_interface = vehicle_interface
        self.detector = YOLO(settings.get('pedestrian_detection.model'))
        # self.detector = YOLO('GEMstack/GEMstack/knowledge/detection/yolov8n.pt')
        self.last_person_boxes = []

    def rate(self):
        return settings.get('pedestrian_detection.rate')
    
    def state_inputs(self):
        return ['vehicle']
    
    def state_outputs(self):
        return ['agents']
    
    def initialize(self):
        #tell the vehicle to use image_callback whenever 'front_camera' gets a reading, and it expects images of type cv2.Mat
        self.vehicle_interface.subscribe_sensor('front_camera',self.image_callback,cv2.Mat)
        pass
    
    def image_callback(self, image : cv2.Mat):
        results = self.detector(image)
        boxes = results[0].boxes
        self.last_person_boxes = []
        self.last_car_boxes = [] # add car objects
        self.last_stop_boxes = [] # add stop sign

        for i in range(len(boxes.cls)):
            if (boxes.cls[i] == settings.get('pedestrian_detection.pedestrian_class')) and (boxes.conf[i] >= settings.get('prediction_confidence')):
                x, y, w, h = boxes[i].xywh[0].cpu().detach().numpy().tolist()
                self.last_person_boxes.append((x,y,w,h))

            if (boxes.cls[i] == settings.get('pedestrian_detection.car_class')) and (boxes.conf[i] >= settings.get('prediction_confidence')):
                x, y, w, h = boxes[i].xywh[0].cpu().detach().numpy().tolist()
                self.last_person_boxes.append((x,y,w,h))
                
            if (boxes.cls[i] == settings.get('pedestrian_detection.stop_sign_class')) and (boxes.conf[i] >= settings.get('prediction_confidence')):
                x, y, w, h = boxes[i].xywh[0].cpu().detach().numpy().tolist()
                self.last_person_boxes.append((x,y,w,h))
        
        # detection_result = self.detector(image)
        # self.last_person_boxes = []
        # for r in results:
        #     for b in r.boxes:
        #         if b.cls.tolist()[0] == 0:
        #             self.last_person_boxes.append(b.xywh.tolist()[0])
        
        ## uncomment if you want to debug the detector...
        # for bb in self.last_person_boxes:
        #     x,y,w,h = bb
        #     cv2.rectangle(image, (int(x-w/2), int(y-h/2)), (int(x+w/2), int(y+h/2)), (255, 0, 255), 3)
        # cv2.imwrite("pedestrian_detections.png",image)
    
    # def update(self, vehicle : VehicleState) -> Dict[str,AgentState]:
    #     res = {}
    #     for i,b in enumerate(self.last_person_boxes):
    #         x,y,w,h = b
    #         res['pedestrian'+str(i)] = box_to_fake_agent(b)
    #     if len(res) > 0:
    #         print("Detected",len(res),"pedestrians")
    #     return res
    def update(self, vehicle: VehicleState) -> Dict[str, AgentState]:
        res = {}
        for i, b in enumerate(self.last_person_boxes):
            agent_id = 'pedestrian' + str(i)
            res[agent_id] = box_to_fake_agent(b)
        for i, b in enumerate(self.last_car_boxes):
            agent_id = 'car' + str(i)
            res[agent_id] = box_to_fake_agent(b)
        for i, b in enumerate(self.last_stop_boxes):
            agent_id = 'stop_sign' + str(i)
            res[agent_id] = box_to_fake_agent(b)
        return res


class FakePedestrianDetector2D(Component):
    """Triggers a pedestrian detection at some random time ranges"""
    def __init__(self,vehicle_interface : GEMInterface):
        self.vehicle_interface = vehicle_interface
        self.times = [(5.0,20.0),(30.0,35.0)]
        self.t_start = None

    def rate(self):
        return 4.0
    
    def state_inputs(self):
        return ['vehicle']
    
    def state_outputs(self):
        return ['agents']
    
    def update(self, vehicle : VehicleState) -> Dict[str,AgentState]:
        if self.t_start is None:
            self.t_start = self.vehicle_interface.time()
        t = self.vehicle_interface.time() - self.t_start
        res = {}
        for times in self.times:
            if t >= times[0] and t <= times[1]:
                res['pedestrian0'] = box_to_fake_agent((0,0,0,0))
                print("Detected a pedestrian")
        return res
