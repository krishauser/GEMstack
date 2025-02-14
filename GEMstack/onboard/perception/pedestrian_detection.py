import time
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


def box_to_fake_agent(box):
    """Creates a fake agent state from an (x,y,w,h) bounding box.
    
    The location and size are pretty much meaningless since this is just giving a 2D location.
    """
    x,y,w,h = box
    pose = ObjectPose(t=0,x=x+w/2,y=y+h/2,z=0,yaw=0,pitch=0,roll=0,frame=ObjectFrameEnum.CURRENT)
    dims = (w,h,0)
    return AgentState(pose=pose,dimensions=dims,outline=None,type=AgentEnum.PEDESTRIAN,activity=AgentActivityEnum.MOVING,velocity=(0,0,0),yaw_rate=0)


class PedestrianDetector2D(Component):
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
        self.vehicle_interface = vehicle_interface
        self.detector = YOLO(os.getcwd()+'/GEMstack/knowledge/detection/yolov8n.pt') # change to get model value from sys arg
        self.last_person_boxes = [] 

        self.disappeared_timers = {}
        self.last_frame_time = None
        self.max_disappear_time = 1.0

        self.pedestrians = {}
        self.visualization = True # Set this to true for visualization, later change to get value from sys arg
        self.confidence = 0.7
        self.classes_to_detect = 0

    def rate(self):
        return 4.0
    
    def state_inputs(self):
        return ['vehicle']
    
    def state_outputs(self):
        return ['agents']
    
    def initialize(self):

        """Initializes subscribers and publishers
        
        self.vehicle_interface.subscribe_sensor -> GEM Car camera subscription
        self.rosbag_test -> ROS Bag topic subscription
        self.pub_image -> Publishes Image with detection for visualization

        """
        #tell the vehicle to use image_callback whenever 'front_camera' gets a reading, and it expects images of type cv2.Mat

        # GEM Car subacriber
        # self.vehicle_interface.subscribe_sensor('front_camera',self.image_callback,cv2.Mat)

        # Webcam
        # rospy.Subscriber('/webcam', Image, self.image_callback)

        # Testing with rosbag
        self.rosbag_test = rospy.Subscriber('/oak/rgb/image_raw',Image, self.image_callback,queue_size=1)
        if(self.visualization):
            self.pub_image = rospy.Publisher("/camera/image_detection", Image, queue_size=1)
        pass
    
    # Use cv2.Mat for GEM Car, Image for RosBag
    def image_callback(self, image : Image): #image : cv2.Mat):

        """Detects pedestrians using the model provided when new image is passed.

        Converts Image.msg to cv2 format (Might need to change this to use cv2.Mat) and uses the model to detect pedestrian
        IF visualization is true, will publish an image with pedestrians detected.

        Hardcoded values for now:
            Detected only pedestrians -> Class = 0
            Confidence level -> 0.7
        
        """

        # Use Image directly for GEM Car
        # track_result = self.detector.track(source=image, classes=self.classes_to_detect, persist=True, conf=self.confidence)

        # Compute time delta (dt) since last frame
        current_time = time.time()
        if self.last_frame_time is None:
            self.last_frame_time = current_time
        dt = current_time - self.last_frame_time
        self.last_frame_time = current_time

        # Convert to CV2 format for RosBag
        bridge = CvBridge() 
        cv_image = bridge.imgmsg_to_cv2(image, "bgr8") 
        # track_result = self.detector.track(source=image, classes=self.classes_to_detect, persist=True, conf=self.confidence)

        # Run YOLO tracking
        track_result = self.detector.track(
            source=cv_image,
            classes=self.classes_to_detect,
            persist=True,
            conf=self.confidence
        )

        # YOLOv8 returns a 'Boxes' object
        # We'll gather the IDs we see in this frame:
        current_ids = set()

        if len(track_result) > 0:
            boxes = track_result[0].boxes
        else:
            boxes = []

        # self.last_person_boxes = []
        # boxes = track_result[0].boxes

        # # Used for visualization
        # if(self.visualization):
        #     label_text = "Pedestrian "
        #     font = cv2.FONT_HERSHEY_SIMPLEX
        #     font_scale = 0.5
        #     font_color = (255, 255, 255)  # White text
        #     outline_color = (0, 0, 0)  # Black outline
        #     line_type = 1
        #     text_thickness = 2 # Text thickness
        #     outline_thickness = 1  # Thickness of the text outline

        # Unpacking box dimentions detected into x,y,w,h
        for box in boxes:

            xywh = box.xywh[0].tolist()
            self.last_person_boxes.append(xywh)
            x, y, w, h = xywh
            # id = box.id.item()

            # YOLO assigned ID
            track_id = box.id.item()
            current_ids.add(track_id)

            # Check if we have seen this ID before
            if track_id not in self.pedestrians:
                # It's a new ID, create a new AgentState
                self.pedestrians[track_id] = box_to_fake_agent(xywh, velocity=(0,0,0))
            else:
                # Update existing
                old_pose = self.pedestrians[track_id].pose

                # Calculate new center
                new_center_x = x + w/2
                new_center_y = y + h/2

                # Compute velocity from old pose -> new pose (pixels/sec or similar)
                if dt > 0:
                    vx = (new_center_x - old_pose.x) / dt
                    vy = (new_center_y - old_pose.y) / dt
                else:
                    vx = 0
                    vy = 0

                # Update the AgentState
                self.pedestrians[track_id].pose.x = new_center_x
                self.pedestrians[track_id].pose.y = new_center_y
                self.pedestrians[track_id].dimensions = (w, h, 0)
                self.pedestrians[track_id].velocity = (vx, vy, 0)

            # Reset the disappeared timer for this ID
            self.disappeared_timers[track_id] = current_time

            # Stores AgentState in a dict, can be removed if not required
            # pose = ObjectPose(t=0,x=x,y=y,z=0,yaw=0,pitch=0,roll=0,frame=ObjectFrameEnum.CURRENT)
            # dims = (w,h,0)
            # if(id not in self.pedestrians.keys()):
            #     self.pedestrians[id] = AgentState(pose=pose,dimensions=dims,outline=None,type=AgentEnum.PEDESTRIAN,activity=AgentActivityEnum.MOVING,velocity=(0,0,0),yaw_rate=0)
            # else:
            #     self.pedestrians[id].pose = pose
            #     self.pedestrians[id].dims = dims

            # Used for visualization
            # if(self.visualization):
            #     # Draw bounding box
            #     cv2.rectangle(image, (int(x - w / 2), int(y - h / 2)), (int(x + w / 2), int(y + h / 2)), (255, 0, 255), 3)

            #     # Define text label
            #     x = int(x - w / 2)
            #     y = int(y - h / 2)
            #     label = label_text + str(id) + " : " + str(round(box.conf.item(), 2))

            #     # Get text size
            #     text_size, baseline = cv2.getTextSize(label, font, font_scale, line_type)
            #     text_w, text_h = text_size

            #     # Position text above the bounding box
            #     text_x = x
            #     text_y = y - 10 if y - 10 > 10 else y + h + text_h

            #     # Draw text outline for better visibility, uncomment for outline
            #     # for dx, dy in [(-1, -1), (-1, 1), (1, -1), (1, 1)]:  
            #     #     cv2.putText(image, label, (text_x + dx, text_y - baseline + dy), font, font_scale, outline_color, outline_thickness)

            #     # Draw main text on top of the outline
            #     cv2.putText(image, label, (text_x, text_y - baseline), font, font_scale, font_color, text_thickness)

        
        # Used for visualization
        if(self.visualization):
            self._visualize(cv_image, boxes)

            # Publish the annotated image
            ros_img = bridge.cv2_to_imgmsg(cv_image, 'bgr8')
            self.pub_image.publish(ros_img)

        # Now handle "disappeared" IDs:
        # If an ID hasn't appeared for > max_disappear_time, remove it.
        to_remove = []
        for pid, last_seen_time in self.disappeared_timers.items():
            if (current_time - last_seen_time) > self.max_disappear_time:
                to_remove.append(pid)

        for pid in to_remove:
            self.disappeared_timers.pop(pid, None)
            self.pedestrians.pop(pid, None)

        #uncomment if you want to debug the detector...
        # print(self.last_person_boxes)
        # print(self.pedestrians.keys())
        #for bb in self.last_person_boxes:
        #    x,y,w,h = bb
        #    cv2.rectangle(image, (int(x-w/2), int(y-h/2)), (int(x+w/2), int(y+h/2)), (255, 0, 255), 3)
        #cv2.imwrite("pedestrian_detections.png",image)

    def _visualize(self, image, boxes):
        """
        Overlays bounding boxes and labels on the image for debugging/visualization.
        """
        label_text = "Pedestrian "
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.5
        font_color = (255, 255, 255)  # White text
        line_type = 1
        text_thickness = 2

        for box in boxes:
            xywh = box.xywh[0].tolist()
            x, y, w, h = xywh
            track_id = box.id.item()
            conf = box.conf.item()

            # Draw bounding box
            x1, y1 = int(x - w/2), int(y - h/2)
            x2, y2 = int(x + w/2), int(y + h/2)
            cv2.rectangle(image, (x1, y1), (x2, y2), (255, 0, 255), 3)

            # Construct label
            label = f"{label_text}{track_id} : {conf:.2f}"

            # Position text above the bounding box
            text_size, baseline = cv2.getTextSize(label, font, font_scale, line_type)
            text_w, text_h = text_size
            text_x = x1
            text_y = y1 - 10 if y1 - 10 > 10 else y2 + text_h

            # Draw text
            cv2.putText(
                image,
                label,
                (text_x, text_y - baseline),
                font,
                font_scale,
                font_color,
                text_thickness
            )
    
    def update(self, vehicle: VehicleState) -> Dict[str, AgentState]:
        """
        Called at the rate specified by self.rate().
        Returns a dictionary of {agent_name: AgentState} for all currently tracked pedestrians.
        """
        # You can name them 'pedestrian0', 'pedestrian1', etc. based on their YOLO ID
        agents = {}
        for pid, agent_state in self.pedestrians.items():
            agents[f"pedestrian_{pid}"] = agent_state
        
        # If you want to see the console output:
        if len(agents) > 0:
            print(f"Currently tracking {len(agents)} pedestrians.")
        return agents

        
    # def update(self, vehicle : VehicleState) -> Dict[str,AgentState]:
    #     res = {}
    #     for i,b in enumerate(self.last_person_boxes):
    #         x,y,w,h = b
    #         res['pedestrian'+str(i)] = box_to_fake_agent(b)
    #     if len(res) > 0:
    #         print("Detected",len(res),"pedestrians")
    #     return res


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
