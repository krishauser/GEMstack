from ...state import AllState,VehicleState,ObjectPose,ObjectFrameEnum,AgentState,AgentEnum,AgentActivityEnum
from ...utils import settings
from ..interface.gem import GEMInterface
from ..component import Component
from typing import Dict, Tuple
import threading
import copy
import cv2
import mmcv
from mmseg.apis import MMSegInferencer
import numpy as np
from skimage import morphology
import cv2
try:
    from sensor_msgs.msg import CameraInfo
    from image_geometry import PinholeCameraModel
    import rospy
except ImportError:
    pass

class CurbSegmentor(Component):
    def __init__(self,vehicle_interface : GEMInterface, extrinsic= None):
        self.print_image = True
        self.vehicle_interface = vehicle_interface
        #self.agents = {}
        #self.lock = threading.Lock()
        # Open the video file for reading
        #self.video_reader = mmcv.VideoReader('curb.mp4')
        self.models = MMSegInferencer.list_models('mmseg')
        self.camera_info_sub = None
        self.camera_info = None
        self.zed_image = None
        self.point_cloud = None
        self.point_cloud_zed = None
        assert(settings.get('vehicle.calibration.top_lidar.reference') == 'rear_axle_center')
        assert(settings.get('vehicle.calibration.front_camera.reference') == 'rear_axle_center')

        
        if extrinsic is None:
            extrinsic = np.loadtxt("GEMstack/knowledge/calibration/gem_e4_lidar2oak.txt")
        self.extrinsic = np.array(extrinsic)
            
        intrinsic = np.loadtxt("GEMstack/knowledge/calibration/gem_e4_intrinsic.txt")
        self.intrinsic = np.concatenate([intrinsic, np.zeros((3, 1))], axis=1)

        T_lidar2_Gem = np.loadtxt("GEMstack/knowledge/calibration/gem_e4_lidar2vehicle.txt")
        self.T_lidar2_Gem = np.asarray(T_lidar2_Gem)

        #self.fps = video_reader.fps
        self.width, self.height = 1024,512

        # Initialize frame counter
        self.frame_number = 0
        self.inferencer = MMSegInferencer(model='cgnet_fcn_4xb8-60k_cityscapes-512x1024')#30 fps

    def rate(self):
        return 15
    
    def state_inputs(self):
        return ['vehicle']
    
    def state_outputs(self):
        return ['roadgraph']

    def initialize(self):
        # tell the vehicle to use image_callback whenever 'front_camera' gets a reading, and it expects images of type cv2.Mat
        self.vehicle_interface.subscribe_sensor('front_camera',self.image_callback,cv2.Mat)
        # tell the vehicle to use lidar_callback whenever 'top_lidar' gets a reading, and it expects numpy arrays
        self.vehicle_interface.subscribe_sensor('top_lidar',self.lidar_callback,np.ndarray)
        # subscribe to the Zed CameraInfo topic
        self.camera_info_sub = rospy.Subscriber("/oak/rgb/camera_info", CameraInfo, self.camera_info_callback, queue_size=5000)

    def image_callback(self, image : cv2.Mat):
        self.zed_image = image

    def camera_info_callback(self, info : CameraInfo):
        self.camera_info = info

    def lidar_callback(self, point_cloud: np.ndarray):
        self.point_cloud = point_cloud

    def update(self, vehicle : VehicleState) -> Dict[str,AgentState]:

        #print(frame_number)
        frame = self.zed_image
        downsampled = cv2.resize(frame, (self.width, self.height))
    
        # Perform inference on the frame
        result = self.inferencer(downsampled, return_datasamples=True)
        seg = result.pred_sem_seg.data.squeeze().cpu().numpy()
        seg = cv2.convertScaleAbs(seg)
        seg_curb = (seg==9).astype(np.uint8)
        if self.print_image:
            downsampled = self.print_curb(seg_curb,downsampled)
            # Display the original and processed images
            cv2.imshow('Curb detector', downsampled)
            cv2.waitKey(1)
            print("reached----------------------")

        #TODO Use coordinates of corners to grab 3D Bounding boxes 
        #TODO Incorperate Roadgraph creation here
        #TODO return Roadgraphs

    def print_curb(self, curb, downsampled):
        curb = cv2.GaussianBlur(curb, (5,5), 0)
        kernel = np.ones((9, 9 ), np.uint8)  # Define a 3x3 kernel for erosion and dilation
        eroded_image = cv2.erode(curb*255, kernel, iterations=1)
        contours, hierarchy = cv2.findContours(eroded_image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        for c in contours:
            area = cv2.contourArea(c)
            if area > 500:
                epsilon = 0.02 * cv2.arcLength(c, True)  # Adjust epsilon as needed
                approx = cv2.approxPolyDP(c, epsilon, True)
                polygon_points = approx.squeeze() 
                print("no. of points in polygon", len(polygon_points), polygon_points)
                # Draw a polygon on the original image
                #cv2.drawContours(downsampled, [approx], 0, (0, 255, 0), 2)

                #OR Draw a filled polygon:
                cv2.fillPoly(downsampled, [approx], (0, 255, 0))
            
        return downsampled
