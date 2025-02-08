from ...state import AllState,VehicleState,ObjectPose,ObjectFrameEnum,AgentState,AgentEnum,AgentActivityEnum
from ..interface.gem import GEMInterface
from ..component import Component
from ultralytics import YOLO
import os
import cv2
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
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
		self.detector = YOLO(f"{os.getcwd()}/knowledge/detection/yolo11n.pt")
		self.last_person_boxes = []

	def rate(self):
		return 4.0
    
	def state_inputs(self):
		return ['vehicle']
    
	def state_outputs(self):
		return ['agents']
    
	def initialize(self):
        # tell the vehicle to use image_callback whenever 'front_camera' gets a reading, and it expects images of type cv2.Mat
        # self.vehicle_interface.subscribe_sensor('front_camera',self.image_callback,cv2.Mat)

		# Initialize cv bridge
		self.bridge = CvBridge()

		# Subscribe to webcam
		# rospy.Subscriber('/webcam', Image, self.image_callback)

		# Subscribe to GEM4 front camera
		rospy.Subscriber('/oak/rgb/image_raw', Image, self.image_callback)

		# Publish visualization
		self.vis = rospy.Publisher("pedestrain_detection/annotate", Image, queue_size=1)

		# Publish AgentState data TODO: Need to implement a custom data type for AgentState msg
		# self.agent_state = rospy.Publisher("pedestrain_detection/agent_state", AgentState, queue_size=1)
         
	def update(self, vehicle: VehicleState=None) -> Dict[str,AgentState]:
		res = {}
		for i,b in enumerate(self.last_person_boxes):
			x,y,w,h = b
			res['pedestrian'+str(i)] = box_to_fake_agent(b)
		if len(res) > 0:
			print("Detected",len(res),"pedestrians")
		return res
    
	def image_callback(self, image: Image):
		# Convert image to cv2 format
		try:
			cv_image = self.bridge.imgmsg_to_cv2(image, "bgr8")
		except CvBridgeError as e:
			print(e)

		detection_result = self.detector(cv_image)
		self.last_person_boxes = []

		# Extract bounding boxes (x,y,w,h)
		data = detection_result[0].boxes
		cls = data.cls.tolist()
		conf = data.conf.tolist()
		bboxes = data.xywh.tolist()
		
		for k in range(len(cls)):
			category = cls[k]
			confidence = conf[k]

			b = bboxes[k]
			bb = (b[0], b[1], b[2], b[3])
			if category == 0 and confidence > 0.5:
				self.last_person_boxes.append(bb)
       
	   	# Insert bounding boxes into AgentState object
		agent_state = self.update()
		print(f"agent_state: {agent_state}")

		# Publish AgentState object TODO: Need to implement a custom data type for AgentState msg
		# self.agent_state.publish(agent_state)

        # Add annotated visualization
		font = cv2.FONT_HERSHEY_SIMPLEX
		for id, bb in enumerate(self.last_person_boxes):
			x,y,w,h = bb
			cv2.rectangle(cv_image, (int(x-w/2), int(y-h/2)), (int(x+w/2), int(y+h/2)), (255, 0, 255), 3)
			cv2.putText(cv_image, f'Person: {id + 1}', (int(x-w/2), int(y-h/2) - 15), font, 0.7, (255,0,255), 2, cv2.LINE_AA)
		cv2.putText(cv_image, f'Detected {len(agent_state)} pedestrians', (50,40), font, 0.8, (255,0,0), 2, cv2.LINE_AA)

		# Publish visualization topic
		if cv_image is not None:
			out_vis_msg = self.bridge.cv2_to_imgmsg(cv_image, 'bgr8')
			self.vis.publish(out_vis_msg)


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
