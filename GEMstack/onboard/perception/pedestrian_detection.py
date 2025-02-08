from ...state import AllState,VehicleState,ObjectPose,ObjectFrameEnum,AgentState,AgentEnum,AgentActivityEnum
from ..interface.gem import GEMInterface
from ..component import Component
from ultralytics import YOLO
import cv2
from typing import Dict
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import rospy
import os
from typing import Union


class PedestrianDetector2DShared(Component):
    """
    Detects pedestrians.
    self.vehicle_interface -> GEM car info
    """
    def __init__(self,vehicle_interface : GEMInterface):
        self.vehicle_interface = vehicle_interface

    def rate(self):
        return 4.0
    
    def state_inputs(self):
        return ['vehicle']
    
    def state_outputs(self):
        return ['agents']
    
    def box_to_fake_agent(box) -> AgentState:
        """Creates a fake agent state from an (x,fy,w,h) bounding box.
        
        The location and size are pretty much meaningless since this is just giving a 2D location.
        """
        x,y,w,h = box
        pose = ObjectPose(t=0,x=x+w/2,y=y+h/2,z=0,yaw=0,pitch=0,roll=0,frame=ObjectFrameEnum.CURRENT)
        dims = (w,h,0)
        return AgentState(pose=pose,dimensions=dims,outline=None,type=AgentEnum.PEDESTRIAN,activity=AgentActivityEnum.MOVING,velocity=(0,0,0),yaw_rate=0)


class PedestrianDetector2D(PedestrianDetector2DShared):
    """Detects pedestrians.
    
    self.vehicle -> GEM car info
    self.detector -> pedestrian detection model
    self.last_person_boxes -> used to store boxes detected in every frame of the video
    self.pedestrians -> stores the agentstate (pedestrian) found during the video in a dict, not sure if required can be removed
    self.visualization -> enable this to view pedestrian detected
    self.confidence -> min model confidence level
    self.classes_to_detect -> Classes for the model to detect
    
    """
    def __init__(self,vehicle_interface : GEMInterface):
        self.detector = YOLO(os.getcwd()+'/GEMstack/knowledge/detection/yolov8n.pt') # change to get model value from sys arg
        self.last_person_boxes = [] 
        self.pedestrians = {}
        self.confidence = 0.7
        self.classes_to_detect = 0
        
        # Setup visualization variables
        self.visualization = True # Set this to true for visualization, later change to get value from sys arg
        self.label_text = "Pedestrian "
        self.font = cv2.FONT_HERSHEY_SIMPLEX
        self.font_scale = 0.5
        self.font_color = (255, 255, 255)  # White text
        self.outline_color = (0, 0, 0)  # Black outline
        self.line_type = 1
        self.text_thickness = 2 # Text thickness
        self.outline_thickness = 1  # Thickness of the text outline
    
    def initialize(self):

        """Initializes subscribers and publishers
        
        self.vehicle_interface.subscribe_sensor -> GEM Car camera subscription
        self.rosbag_test -> ROS Bag topic subscription
        self.pub_image -> Publishes Image with detection for visualization

        """
        #tell the vehicle to use image_callback whenever 'front_camera' gets a reading, and it expects images of type cv2.Mat

        # GEM Car subscriber
        # self.vehicle_interface.subscribe_sensor('front_camera',self.image_callback,cv2.Mat)

        # Webcam
        # rospy.Subscriber('/webcam', Image, self.image_callback)

        # Testing with rosbag
        self.rosbag_test = rospy.Subscriber('/oak/rgb/image_raw',Image, self.image_callback,queue_size=1)
        if(self.visualization):
            self.pub_image = rospy.Publisher("/camera/image_detection", Image, queue_size=1)
    
    # Use cv2.Mat for GEM Car, Image for RosBag
    def image_callback(self, image : Union[cv2.Mat, Image]):

        """Detects pedestrians using the model provided when new image is passed.

        Converts Image.msg to cv2 format (Might need to change this to use cv2.Mat) and uses the model to detect pedestrian
        IF visualization is true, will publish an image with pedestrians detected.

        Hardcoded values for now:
            Detected only pedestrians -> Class = 0
            Confidence level -> 0.7
        
        """

        # Use Image directly for GEM Car convert to cv2.Mat for rosbag:
        if type(image) == Image:
            bridge = CvBridge() 
            image = bridge.imgmsg_to_cv2(image, "bgr8") 
        track_result = self.detector.track(source=image, classes=self.classes_to_detect, persist=True, conf=self.confidence)

        self.last_person_boxes = []
        boxes = track_result[0].boxes

        # Unpacking box dimentions detected into x,y,w,h
        for box in boxes:

            xywh = box.xywh[0].tolist()
            self.last_person_boxes.append(xywh)
            x, y, w, h = xywh
            id = box.id.item()

            # Stores AgentState in a dict, can be removed if not required
            pose = ObjectPose(t=0,x=x,y=y,z=0,yaw=0,pitch=0,roll=0,frame=ObjectFrameEnum.CURRENT)
            dims = (w,h,0)
            if(id not in self.pedestrians.keys()):
                self.pedestrians[id] = AgentState(pose=pose,dimensions=dims,outline=None,type=AgentEnum.PEDESTRIAN,activity=AgentActivityEnum.MOVING,velocity=(0,0,0),yaw_rate=0)
            else:
                self.pedestrians[id].pose = pose
                self.pedestrians[id].dims = dims

            # Used for visualization
            if(self.visualization):
                self.__visualize_labeled_image()
        
        # Used for visualization
        if(self.visualization):
            ros_img = bridge.cv2_to_imgmsg(image, 'bgr8')
            self.pub_image.publish(ros_img)

        #uncomment if you want to debug the detector...
        # print(self.last_person_boxes)
        # print(self.pedestrians.keys())
        #for bb in self.last_person_boxes:
        #    x,y,w,h = bb
        #    cv2.rectangle(image, (int(x-w/2), int(y-h/2)), (int(x+w/2), int(y+h/2)), (255, 0, 255), 3)
        #cv2.imwrite("pedestrian_detections.png",image)

    def __visualize_labeled_image(self, image: cv2.Mat):
        # Draw bounding box
        cv2.rectangle(image, (int(x - w / 2), int(y - h / 2)), (int(x + w / 2), int(y + h / 2)), (255, 0, 255), 3)

        # Define text label
        x = int(x - w / 2)
        y = int(y - h / 2)
        label = self.label_text + str(id) + " : " + str(round(box.conf.item(), 2))

        # Get text size
        text_size, baseline = cv2.getTextSize(label, self.font, self.font_scale, self.line_type)
        text_w, text_h = text_size

        # Position text above the bounding box
        text_x = x
        text_y = y - 10 if y - 10 > 10 else y + h + text_h

        # Draw text outline for better visibility, uncomment for outline
        # for dx, dy in [(-1, -1), (-1, 1), (1, -1), (1, 1)]:  
        #     cv2.putText(image, label, (text_x + dx, text_y - baseline + dy), self.font, self.font_scale, self.outline_color, self.outline_thickness)

        # Draw main text on top of the outline
        cv2.putText(image, label, (text_x, text_y - baseline), self.font, self.font_scale, self.font_color, self.text_thickness)
    
    def update(self, vehicle: VehicleState) -> Dict[str, AgentState]:
        res = {}
        for i,b in enumerate(self.last_person_boxes):
            x,y,w,h = b
            res['pedestrian'+str(i)] = box_to_fake_agent(b)
        if len(res) > 0:
            print("Detected",len(res),"pedestrians")
        return res


class FakePedestrianDetector2D(PedestrianDetector2DShared):
    """Triggers a pedestrian detection at some random time ranges"""
    def __init__(self, vehicle_interface: GEMInterface):
        self.times = [(5.0,20.0),(30.0,35.0)]
        self.t_start = None
    
    def update(self, vehicle: VehicleState) -> Dict[str, AgentState]:
        if self.t_start is None:
            self.t_start = self.vehicle_interface.time()
        t = self.vehicle_interface.time() - self.t_start
        res = {}
        for times in self.times:
            if t >= times[0] and t <= times[1]:
                res['pedestrian0'] = box_to_fake_agent((0,0,0,0))
                print("Detected a pedestrian")
        return res
