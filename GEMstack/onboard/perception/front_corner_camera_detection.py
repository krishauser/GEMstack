# Front corner camera fl and fr are intrgrated here
# Future work
# Add rear corner camera in intergrate_corner_camera() function to build bird eye view after getting calibration matrix for those camera

from ...state import AllState,VehicleState,ObjectPose,ObjectFrameEnum,AgentState,AgentEnum,AgentActivityEnum,SignEnum, SignState, Sign
from ..interface.gem import GEMInterface
from ..component import Component
from typing import Dict
import threading
from sklearn.cluster import DBSCAN
import copy
import cv2
import struct
import sensor_msgs.point_cloud2 as pc2
import numpy as np
import ctypes
from ultralytics import YOLO
from typing import Dict, Tuple, List
from GEMstack.onboard.perception.pixelwise_3D_lidar_coord_handler import PixelWise3DLidarCoordHandler

try:
    from sensor_msgs.msg import CameraInfo, PointCloud2
    from image_geometry import PinholeCameraModel
    import rospy
except ImportError:
    pass

def ros_PointCloud2_to_numpy(pc2_msg, want_rgb=False):
    if pc2 is None:
        raise ImportError("ROS is not installed")
    # gen = pc2.read_points(pc2_msg, skip_nans=True)
    gen = pc2.read_points(pc2_msg, skip_nans=True, field_names=['x', 'y', 'z'])

    if want_rgb:
        xyzpack = np.array(list(gen), dtype=np.float32)
        if xyzpack.shape[1] != 4:
            raise ValueError("PointCloud2 does not have points with color data.")
        xyzrgb = np.empty((xyzpack.shape[0], 6))
        xyzrgb[:,:3] = xyzpack[:,:3]
        for i, x in enumerate(xyzpack):
            rgb = x[3]
            # cast float32 to int so that bitwise operations are possible
            s = struct.pack('>f', rgb)
            i = struct.unpack('>l', s)[0]
            # you can get back the float value by the inverse operations
            pack = ctypes.c_uint32(i).value
            r = (pack & 0x00FF0000) >> 16
            g = (pack & 0x0000FF00) >> 8
            b = (pack & 0x000000FF)
            # r,g,b values in the 0-255 range
            xyzrgb[i,3:] = (r, g, b)
        return xyzrgb
    else:
        return np.array(list(gen), dtype=np.float32)[:,:3]


class FrontCornerCameraDetection(Component):
    """Utilize YOLOv8 to detect multi-objects in each frame"""
    def __init__(self,vehicle_interface : GEMInterface):
        self.vehicle_interface = vehicle_interface
        self.detector = YOLO('GEMstack/knowledge/detection/yolov8s.pt')
        self.pedestrian_counter = 0
        self.last_agent_states = {}
        self.previous_agents = {}
        self.coord_3d_handler = PixelWise3DLidarCoordHandler()
        # Hardcode the roi area for agents
        self.xrange = (0, 20)
        self.yrange = (-10, 10)
        self.zrange = (-3, 1)
        self.visulization = False
    def rate(self):
        return 4.0

    def state_inputs(self):
        return ['vehicle']

    def state_outputs(self):
        return ['agents']

    def initialize(self):
        #tell the vehicle to use image_callback whenever 'front_camera' gets a reading, and it expects images of type cv2.Mat
        self.corner_fr = rospy.Subscriber("/camera_fr/arena_camera_node/image_raw", self.corner_fr_callback, cv2.Mat)
        self.corner_fl = rospy.Subscriber("/camera_fl/arena_camera_node/image_raw", self.corner_fl_callback, cv2.Mat)
        #tell the vehicle to use lidar_callback whenever 'top_lidar' gets a reading, and it expects numpy arrays
        self.vehicle_interface.subscribe_sensor('top_lidar',self.lidar_callback,np.ndarray)
        #subscribe to the Zed CameraInfo topic
        self.point_cloud_sub = rospy.Subscriber("/ouster/points", PointCloud2, self.lidar_callback)

    def corner_fr_callback(self, image : cv2.Mat):
        self.fr_image = image

    def corner_fl_callback(self, image : cv2.Mat):
        self.fl_image = image
    
    def lidar_callback(self, point_cloud: np.ndarray):
        self.point_cloud = point_cloud

    def update(self, vehicle : VehicleState) -> Dict[str,AgentState]:
        if self.fr_image or self.fl_image is None:
            #no image data yet
            return {}
        if self.point_cloud is None:
            #no lidar data yet
            return {}

        corner_camera_detected_agents = self.integrate_corner_camera()

        return corner_camera_detected_agents 
    
    def box_to_agent(self, box, cls,x_3d, y_3d, z_3d):
        """Creates a 3D agent state from an (x,y,w,h) bounding box.

        TODO: you need to use the image, the camera intrinsics, the lidar
        point cloud, and the calibrated camera / lidar poses to get a good
        estimate of the pedestrian's pose and dimensions.
        """
        # get the idxs of point cloud that belongs to the agent
        b_x, b_y, w, h = box

        # Specify ObjectPose. Note that The pose's yaw, pitch, and roll are assumed to be 0 for simplicity.
        pose = ObjectPose(t=0, x=x_3d, y=y_3d, z=z_3d, yaw=0, pitch=0, roll=0, frame=ObjectFrameEnum.CURRENT)

        l = 1
        dims = (l, w, h) 

        if cls == SignEnum.STOP_SIGN:
            state=SignState(signal_state=None, left_turn_signal_state = None, right_turn_signal_state = None,
                            crossing_gate_state = None)
            return Sign(pose=pose, dimensions=dims, outline=None, type=cls, entities=["intersection"], speed=0, state=state)
        else:
            return AgentState(pose=pose,dimensions=dims,outline=None,type=cls,activity=AgentActivityEnum.MOVING,velocity=(0,0,0),yaw_rate=0, attributes=0)

    def corner_camera_detect_agents(self, img, pc_3D,which_camera):
        print("Start Front Corener Camera Detecting...")
        if self.fr_image or self.fl_image is None:
            return
        rospy.loginfo("Detecting agents from Corner Camera...")
        detection_result = self.detector(img,verbose=False)
        height, width, channels = img.shape
        colors = []
        colors += [(0, 255, i) for i in range(100, 256, 25)]
     
        #TODO: create boxes from detection result
        boxes = []
        target_ids = [0] # 0 for pedestrian
        self.class_names = {0: "Pedestrian"}
        bbox_ids = []
        for box in detection_result[0].boxes: # only one image, so use index 0 of result
            class_id = int(box.cls[0].item())
            if class_id in target_ids: # class 0 stands for pedestrian
                bbox_ids.append(class_id)
                bbox = box.xywh[0].tolist()
                boxes.append(bbox)
        vis = img.copy()
        detected_agents = []

        for i,b in enumerate(boxes):
            box = boxes[i]
            id = bbox_ids[i]

            x, y, w, h = box        
            color = colors(int(cls), True)
           
            label = f"{self.class_names} "
            
            x1,y1,x2,y2 = box            
            width = x2 - x1
            height = y2 - y1
            x1s = int(x1 + width // 4)
            x2s = int(x1 + width // 4 * 3)
            y1s = int(y1 + height // 4)
            y2s = int(y1 + height // 4 * 3)
            
            coord_3d_map = self.coord_3d_handler.get3DCoord(img, pc_3D, which_camera)
            cv2.rectangle(vis, (x1s, y1s), (x2s, y2s), (0, 255, 0), thickness=2)
            
            small_coord_3d_map = coord_3d_map[y1s:y2s, x1s:x2s]
            coord_3d = [0, 0, 0]
            for i in range(3): # channels
                coord_3d[i] = np.median(small_coord_3d_map[:, :, i])
            
            x_3d, y_3d, z_3d = coord_3d
            depth = np.mean( (x_3d ** 2 + y_3d[:, 1] ** 2) ** 0.5 )
            # x_3d, y_3d, z_3d = coord_3d_map[int(center_y)][int(center_x)]
            label = label + f' x={x_3d:.2f}, y={y_3d:.2f}, z={z_3d:.2f}'
           
       
            
            if id == 0:
                color = (255, 0, 255)
                self.pedestrian_counter += 1
                cls = AgentEnum.PEDESTRIAN
                print("Pedestrian Depth:", depth)
                agent = self.box_to_agent(b, cls, x_3d,y_3d,z_3d, depth)
                if agent is not None:
                    detected_agents.append(agent)
            

            # draw bbox
            left_up = (int(x-w/2), int(y-h/2))
            right_bottom = (int(x+w/2), int(y+h/2))
            cv2.rectangle(vis, left_up, right_bottom, color, thickness=2)
            # draw
            text = str(target_ids[id])
            text += f" | depth: {depth:.2f}"
            text_size, baseline = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.4, 1)
            p1 = (left_up[0], left_up[1] - text_size[1])
            cv2.rectangle(vis, (p1[0] - 2 // 2, p1[1] - 2 - baseline), (p1[0] + text_size[0], p1[1] + text_size[1]),
                        color, -1)
            cv2.putText(vis, text, (p1[0], p1[1] + baseline), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1, 8)

        
        return detected_agents
    
    def integrate_corner_camera(self):
        if self.fr_image is None or self.fl_image is None or self.point_cloud is None:
            return

        fl_vis = self.bridge.imgmsg_to_cv2(self.fl_image, "bgr8")
        fr_vis = self.bridge.imgmsg_to_cv2(self.fr_image, "bgr8")
        
        raw_point_cloud = ros_PointCloud2_to_numpy(self.point_cloud, want_rgb=False)
    
        fl_detection = self.corner_camera_detect_agents(fl_vis, raw_point_cloud,'front_left')
        fr_detection = self.corner_camera_detect_agents(fr_vis, raw_point_cloud,'front_center')
                
        if self.visulization:
            combined_frame = cv2.hconcat([fl_detection,fr_detection])

            # Display the combined frame
            cv2.imshow('Front Corner Cameras', combined_frame)

            # cv2.imshow('frame', vis)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                exit(1)


        



