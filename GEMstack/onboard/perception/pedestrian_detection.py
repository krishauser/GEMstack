import math
import random
from ...state import AllState,VehicleState,ObjectPose,ObjectFrameEnum,AgentState,AgentEnum,AgentActivityEnum
from ..interface.gem import GEMInterface
from ..component import Component
#from ultralytics import YOLO
#import cv2
from typing import Dict

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
        #self.detector = YOLO('../../knowledge/detection/yolov8n.pt')
        self.last_person_boxes = []

    def rate(self):
        return 4.0
    
    def state_inputs(self):
        return ['vehicle']
    
    def state_outputs(self):
        return ['agents']
    
    def initialize(self):
        #tell the vehicle to use image_callback whenever 'front_camera' gets a reading, and it expects images of type cv2.Mat
        #self.vehicle_interface.subscribe_sensor('front_camera',self.image_callback,cv2.Mat)
        pass
    
    #def image_callback(self, image : cv2.Mat):
    #    detection_result = self.detector(image)
    #    self.last_person_boxes = []
    #    #uncomment if you want to debug the detector...
    #    #for bb in self.last_person_boxes:
    #    #    x,y,w,h = bb
    #    #    cv2.rectangle(image, (int(x-w/2), int(y-h/2)), (int(x+w/2), int(y+h/2)), (255, 0, 255), 3)
    #    #cv2.imwrite("pedestrian_detections.png",image)
    
    def update(self, vehicle : VehicleState) -> Dict[str,AgentState]:
        res = {}
        for i,b in enumerate(self.last_person_boxes):
            x,y,w,h = b
            res['pedestrian'+str(i)] = box_to_fake_agent(b)
        if len(res) > 0:
            print("Detected",len(res),"pedestrians")
        return res


class FakePedestrianDetector2D(Component):
    """Triggers a pedestrian detection at some random time ranges"""
    # def __init__(self,vehicle_interface : GEMInterface):
    #     self.vehicle_interface = vehicle_interface
    #     self.times = [(5.0,20.0),(30.0,35.0)]
    #     self.t_start = None
    def __init__(self, vehicle_interface):
        self.vehicle_interface = vehicle_interface
        self.t_start = None
        self.last_spawn_time = 0
        self.spawn_interval = 3  # Spawn a pedestrian every 3 seconds
        self.max_pedestrians = 10  # Maximum pedestrians at a time
        self.pedestrians = {}
        self.despawn_time = 30  # Despawn after 30 seconds
        self.direction_change_interval = 10  # Change direction every 10 seconds

    def rate(self):
        # return 4.0
        return 10.0
    
    def state_inputs(self):
        return ['vehicle']
    
    def state_outputs(self):
        return ['agents']
    
    def update(self, vehicle: VehicleState) -> Dict[str, AgentState]:
        """Simulates pedestrians moving freely in all directions with varied speeds."""

        if self.t_start is None:
            self.t_start = self.vehicle_interface.time()

        current_time = self.vehicle_interface.time() - self.t_start
        res = {}

        # Spawn new pedestrians
        if current_time - self.last_spawn_time > self.spawn_interval:
            if len(self.pedestrians) < self.max_pedestrians:
                agent_id = f"pedestrian{len(self.pedestrians)}"
                
                # Spawn at random locations in a 20x20 area
                x_start = random.uniform(-10, 10)
                y_start = random.uniform(-10, 10)

                # Random walking direction (angle in radians)
                direction = random.uniform(0, 2 * math.pi)

                # Assign a random speed (between 0.5 and 1.5 m/s)
                speed = random.uniform(0.5, 1.5)

                self.pedestrians[agent_id] = {
                    "x": x_start,
                    "y": y_start,
                    "speed": speed,
                    "direction": direction,
                    "last_direction_change": current_time,
                    "spawn_time": current_time
                }
                print(f"Spawned {agent_id} at ({x_start}, {y_start}) moving at {speed:.2f} m/s")

            self.last_spawn_time = current_time

        # Move pedestrians
        for agent_id, state in list(self.pedestrians.items()):
            t = current_time - state["spawn_time"]
            speed = state["speed"]

            # Change direction periodically
            if current_time - state["last_direction_change"] > self.direction_change_interval:
                state["direction"] = random.uniform(0, 2 * math.pi)
                state["last_direction_change"] = current_time
                print(f"{agent_id} changed direction")

            # Compute new position based on speed and direction
            x_new = state["x"] + speed * 0.1 * math.cos(state["direction"])
            y_new = state["y"] + speed * 0.1 * math.sin(state["direction"])

            # Remove pedestrian after 30 seconds
            if t > self.despawn_time:
                print(f"{agent_id} despawns")
                del self.pedestrians[agent_id]
            else:
                state["x"], state["y"] = x_new, y_new
                res[agent_id] = box_to_fake_agent((x_new, y_new, 0, 0))

        print(f"Currently tracking {len(self.pedestrians)} pedestrians")
        return res