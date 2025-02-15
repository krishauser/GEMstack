from ...state import AllState,VehicleState,ObjectPose,ObjectFrameEnum,AgentState,AgentEnum,AgentActivityEnum
from ..interface.gem import GEMInterface
from ..component import Component
from ultralytics import YOLO
import cv2
from typing import Dict
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
from geometry_msgs.msg import TransformStamped
import rospy
import os
import numpy as np
import tf2_ros
import json
import open3d as o3d

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
        self.pedestrians = {}
        self.visualization = True # Set this to true for visualization, later change to get value from sys arg
        self.confidence = 0.7
        self.classes_to_detect = 0
        self.rosbag_image = None

    def rate(self):
        return 4.0
    
    def state_inputs(self):
        return ['vehicle']
    
    def state_outputs(self):
        return ['agents']
    
    def initialize(self):

        """Initializes subscribers and publishers
        
        self.vehicle_interface.subscribe_sensor -> GEM Car camera subscription
        self.rosbag_cam_sub -> ROS Bag topic subscription
        self.rosbag_cam_pub -> Publishes Image with detection for visualization

        """
        #tell the vehicle to use image_callback whenever 'front_camera' gets a reading, and it expects images of type cv2.Mat

        # GEM Car subacriber
        self.vehicle_interface.subscribe_sensor('front_camera',self.image_callback,cv2.Mat)

        # Webcam
        # rospy.Subscriber('/webcam', Image, self.image_callback)

        # Conversion tools
        self.cv_bridge = CvBridge()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Testing with rosbag
        if(self.visualization):
            self.rosbag_cam_pub = rospy.Publisher("/camera/image_detection", Image, queue_size=1)
            self.rosbag_cam_lidar_pub = rospy.Publisher("/camera/sensor_fusion", Image, queue_size=1)

        # Lidar Tranform topic publisher
        self.rosbag_lidar_livox_pub = rospy.Publisher("/livox/transformed_lidar", PointCloud2, queue_size=1)
        self.rosbag_lidar_ouster_pub = rospy.Publisher("/ouster/transformed_lidar", PointCloud2, queue_size=1)

        self.rosbag_cam_sub = rospy.Subscriber('/oak/rgb/image_raw',Image, self.image_callback,queue_size=1)
        # self.rosbag_lidar_livox_sub = rospy.Subscriber('/livox/lidar',PointCloud2, self.lidar_livox_callback,queue_size=1)
        self.rosbag_lidar_ouster_sub = rospy.Subscriber('/ouster/points',PointCloud2, self.lidar_ouster_callback,queue_size=1)
        
        pass
    

    # Publishes baselink to map transformation for visualization
    def publish_base_link_to_map(self):

        br = tf2_ros.TransformBroadcaster()
        transform = TransformStamped()
        
        transform.header.stamp = rospy.Time.now()
        transform.header.frame_id = "map"  # Parent frame
        transform.child_frame_id = "base_link"  # Child frame

        # Set translation (change as per actual vehicle position)
        transform.transform.translation.x = 0.0
        transform.transform.translation.y = 0.0
        transform.transform.translation.z = 0.0

        # Set rotation (quaternion)
        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = 0.0
        transform.transform.rotation.w = 1.0  # No rotation
        br.sendTransform(transform)


    # Livox Lidar data
    def lidar_livox_callback(self,pointcloud : PointCloud2):

        points = np.array(list(pc2.read_points(pointcloud, skip_nans=True)), dtype=np.float32)[:, :3]
        intrinsic_file_path = os.getcwd()+'/GEMstack/knowledge/calibration/intrinsic.json'
        extrinsics_file_path = os.getcwd()+'/GEMstack/knowledge/calibration/extrinsics.npz'
        R, t = self.load_extrinsics(extrinsics_file_path)
        K = self.load_intrinsics(intrinsic_file_path)


        # if(self.visualization):
        #     transform = self.tf_buffer.lookup_transform('base_link', pointcloud.header.frame_id, rospy.Time(0), rospy.Duration(1.0))
        #     # Transform the point cloud
        #     transformed_cloud = do_transform_cloud(pointcloud, transform)
        #     # Publish transformed point cloud
        #     self.rosbag_lidar_livox_pub.publish(transformed_cloud)
        #     self.publish_base_link_to_map()

    # Ouster Lidar data
    def lidar_ouster_callback(self,pointcloud : PointCloud2):

        points = np.array(list(pc2.read_points(pointcloud, skip_nans=True)), dtype=np.float32)[:, :3]
        intrinsic_file_path = os.getcwd()+'/GEMstack/knowledge/calibration/intrinsic.json'
        extrinsics_file_path = os.getcwd()+'/GEMstack/knowledge/calibration/extrinsics.npz'
        R, t = self.load_extrinsics(extrinsics_file_path)
        K = self.load_intrinsics(intrinsic_file_path)
        
        transformed_points = (R @ points.T + t.reshape(3,1)).T

        print(f"before :{transformed_points.shape}")
        # Convert numpy array to Open3D point cloud
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(transformed_points)

        # Apply voxel grid downsampling
        voxel_size = 0.1  # Adjust for desired resolution
        downsampled_pcd = pcd.voxel_down_sample(voxel_size=voxel_size)

        # Convert back to numpy array
        transformed_points = np.asarray(downsampled_pcd.points)
        print(f"after :{transformed_points.shape}")

        for p in transformed_points:
            if p[2] > 0:  # only project points in front of the camera
                u = K[0, 0] * (p[0] / p[2]) + K[0, 2]
                v = K[1, 1] * (p[1] / p[2]) + K[1, 2]
                cv2.circle(self.rosbag_image, (int(u),int(v)), 2, (0, 0, 255), -1)

        ros_img = self.cv_bridge.cv2_to_imgmsg(self.rosbag_image, 'bgr8')
        self.rosbag_cam_lidar_pub.publish(ros_img)
        
        

        # if(self.visualization):
        #     transform = self.tf_buffer.lookup_transform('base_link', pointcloud.header.frame_id, rospy.Time(0), rospy.Duration(1.0))
        #     # Transform the point cloud
        #     transformed_cloud = do_transform_cloud(pointcloud, transform)
        #     # Publish transformed point cloud
        #     self.rosbag_lidar_ouster_pub.publish(transformed_cloud)
        #     self.publish_base_link_to_map()


    def load_extrinsics(self,extrinsics_file):
        """
        Load calibrated extrinsics from a .npz file.
        Assumes the file contains keys 'R' and 't'.
        """
        data = np.load(extrinsics_file)
        R = data['R']
        t = data['t']
        return R, t

    def load_intrinsics(self,intrinsics_file):
        """
        Load camera intrinsics from a JSON file.
        Expects keys: 'fx', 'fy', 'cx', and 'cy'.
        """
        with open(intrinsics_file, 'r') as f:
            intrinsics = json.load(f)
        fx = intrinsics["fx"]
        fy = intrinsics["fy"]
        cx = intrinsics["cx"]
        cy = intrinsics["cy"]
        K = np.array([[fx, 0, cx],
                    [0, fy, cy],
                    [0,  0,  1]], dtype=np.float32)
        return K

    
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

        # Convert to CV2 format for RosBag
        image = self.cv_bridge.imgmsg_to_cv2(image, "bgr8") 
        track_result = self.detector.track(source=image, classes=self.classes_to_detect, persist=True, conf=self.confidence)

        self.last_person_boxes = []
        boxes = track_result[0].boxes

        # Used for visualization
        if(self.visualization):
            label_text = "Pedestrian "
            font = cv2.FONT_HERSHEY_SIMPLEX
            font_scale = 0.5
            font_color = (255, 255, 255)  # White text
            outline_color = (0, 0, 0)  # Black outline
            line_type = 1
            text_thickness = 2 # Text thickness
            outline_thickness = 1  # Thickness of the text outline

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
                # Draw bounding box
                cv2.rectangle(image, (int(x - w / 2), int(y - h / 2)), (int(x + w / 2), int(y + h / 2)), (255, 0, 255), 3)

                # Define text label
                x = int(x - w / 2)
                y = int(y - h / 2)
                label = label_text + str(id) + " : " + str(round(box.conf.item(), 2))

                # Get text size
                text_size, baseline = cv2.getTextSize(label, font, font_scale, line_type)
                text_w, text_h = text_size

                # Position text above the bounding box
                text_x = x
                text_y = y - 10 if y - 10 > 10 else y + h + text_h

                # Draw text outline for better visibility, uncomment for outline
                # for dx, dy in [(-1, -1), (-1, 1), (1, -1), (1, 1)]:  
                #     cv2.putText(image, label, (text_x + dx, text_y - baseline + dy), font, font_scale, outline_color, outline_thickness)

                # Draw main text on top of the outline
                cv2.putText(image, label, (text_x, text_y - baseline), font, font_scale, font_color, text_thickness)
                self.rosbag_image = image

        
        # Used for visualization
        if(self.visualization):
            ros_img = self.cv_bridge.cv2_to_imgmsg(image, 'bgr8')
            self.rosbag_cam_pub.publish(ros_img)

        #uncomment if you want to debug the detector...
        # print(self.last_person_boxes)
        # print(self.pedestrians.keys())
        #for bb in self.last_person_boxes:
        #    x,y,w,h = bb
        #    cv2.rectangle(image, (int(x-w/2), int(y-h/2)), (int(x+w/2), int(y+h/2)), (255, 0, 255), 3)
        #cv2.imwrite("pedestrian_detections.png",image)
    
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
