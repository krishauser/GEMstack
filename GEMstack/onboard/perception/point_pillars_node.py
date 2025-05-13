# from ...state import AllState, VehicleState, ObjectPose, ObjectFrameEnum, AgentState, AgentEnum, AgentActivityEnum
# from ..interface.gem import GEMInterface
# from ..component import Component
# from perception_utils import *
from combined_detection_utils import *
import numpy as np
from scipy.spatial.transform import Rotation as R
import rospy
from sensor_msgs.msg import PointCloud2, Image
from message_filters import Subscriber, ApproximateTimeSynchronizer
from cv_bridge import CvBridge
import time
import os
import ros_numpy

# PointPillars imports:
import torch
from pointpillars.model import PointPillars

# Publisher imports:
from jsk_recognition_msgs.msg import BoundingBox, BoundingBoxArray
from geometry_msgs.msg import Pose, Vector3


def pc2_to_numpy_with_intensity(pc2_msg, want_rgb=False):
    """
    Convert a ROS PointCloud2 message into a numpy array quickly using ros_numpy.
    This function extracts the x, y, z coordinates and reflecivity from the point cloud.
    """
    # Convert the ROS message to a numpy structured array
    pc = ros_numpy.point_cloud2.pointcloud2_to_array(pc2_msg)
    reflectivity = np.array(pc['reflectivity']).ravel()
    intensity = np.array(pc['intensity']).ravel()

    # Normalize reflectivity to 0-1 range
    normalized_reflectivity = reflectivity / 255.0

    # Stack x,y,z, r fields to a (N,4) array
    pts = np.stack((np.array(pc['x']).ravel(),
                    np.array(pc['y']).ravel(),
                    np.array(pc['z']).ravel(),
                    normalized_reflectivity), 
                    axis=1)

    # Restrict points to the model's default range:
    x_min, y_min, z_min = 0, -39.68, -3
    x_max, y_max, z_max = 69.12, 39.68, 1

    mask = (
        (pts[:, 0] >= x_min) & (pts[:, 0] <= x_max) &
        (pts[:, 1] >= y_min) & (pts[:, 1] <= y_max) &
        (pts[:, 2] >= z_min) & (pts[:, 2] <= z_max)
    )
    return pts[mask]


class PointPillarsNode():
    """
    Detects Pedestrians with PointPillars and publishes the results in vehicle frame.
    """

    def __init__(
        self,
    ):
        self.latest_image        = None
        self.latest_lidar        = None
        self.bridge              = CvBridge()
        self.camera_name = 'front'
        self.camera_front = (self.camera_name=='front')
        self.score_threshold = 0.4
        self.debug = True
        self.initialize()

    def initialize(self):
        # --- Determine the correct RGB topic for this camera ---
        rgb_topic_map = {
            'front': '/oak/rgb/image_raw',
            'front_right': '/camera_fr/arena_camera_node/image_raw',
            # add additional camera mappings here if needed
        }
        rgb_topic = rgb_topic_map.get(
            self.camera_name,
            f'/{self.camera_name}/rgb/image_raw'
        )

        # Initialize PointPillars node
        rospy.init_node('pointpillars_box_publisher')
        # Create bounding box publisher
        self.pub = rospy.Publisher('/pointpillars_boxes', BoundingBoxArray, queue_size=10)
        rospy.loginfo("PointPillars node initialized and waiting for messages.")

        # Initialize PointPillars
        device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self.pointpillars = PointPillars(
                nclasses=3,
                voxel_size=[0.16, 0.16, 4],
                point_cloud_range=[0, -39.68, -3, 69.12, 39.68, 1],
                max_num_points=32,
                max_voxels=(16000, 40000)
            )
        self.pointpillars.to(device)

        model_path = 'epoch_160.pth'
        checkpoint = torch.load(model_path) #, map_location='cuda' if torch.cuda.is_available() else 'cpu')
        self.pointpillars.load_state_dict(checkpoint)

        self.pointpillars.eval()
        rospy.loginfo("PointPillars model loaded successfully")

        self.T_l2v = np.array([[0.99939639, 0.02547917, 0.023615, 1.1],
                               [-0.02530848, 0.99965156, -0.00749882, 0.03773583],
                               [-0.02379784, 0.00689664, 0.999693, 1.95320223],
                               [0., 0., 0., 1.]])

        # Subscribe to the RGB and LiDAR streams
        self.rgb_sub = Subscriber(rgb_topic, Image)
        self.lidar_sub = Subscriber('/ouster/points', PointCloud2)
        self.sync = ApproximateTimeSynchronizer([
            self.rgb_sub, self.lidar_sub
        ], queue_size=10, slop=0.1)
        self.sync.registerCallback(self.synchronized_callback)

    def synchronized_callback(self, image_msg, lidar_msg):
        rospy.loginfo("Received synchronized RGB and LiDAR messages")
        self.latest_lidar = pc2_to_numpy_with_intensity(lidar_msg, want_rgb=False)

        downsample = False

        if downsample:
            lidar_down = downsample_points(self.latest_lidar, voxel_size=0.1)
        else:
            lidar_down = self.latest_lidar.copy()

        boxes = BoundingBoxArray()
        boxes.header.frame_id = 'currentVehicleFrame'
        boxes.header.stamp = lidar_msg.header.stamp

        with torch.no_grad():
            # Convert to tensor and format for PointPillars
            lidar_tensor = torch.from_numpy(lidar_down).float()
            if torch.cuda.is_available():
                lidar_tensor = lidar_tensor.cuda()
            
            # Format as batch with a single point cloud
            batched_pts = [lidar_tensor]
            
            # Get PointPillars predictions
            results = self.pointpillars.forward(batched_pts, mode='test')
            
            if results and len(results) > 0 and 'lidar_bboxes' in results[0]:
                # Process PointPillars results
                pp_bboxes = results[0]['lidar_bboxes']
                pp_labels = results[0]['labels'] 
                pp_scores = results[0]['scores']

                for i, (bbox, label, score) in enumerate(zip(pp_bboxes, pp_labels, pp_scores)):
                    # Skip low confidence detections and non-pedestrians (assuming class is 0)
                    if (score < self.score_threshold) or (label != 0):
                        continue
                        
                    # Extract center position and dimensions
                    x, y, z, l, w, h, yaw = bbox

                    # Transform from LiDAR to vehicle coordinates
                    center_lidar = np.array([x, y, z, 1.0])
                    center_vehicle = self.T_l2v @ center_lidar
                    x_vehicle, y_vehicle, z_vehicle = center_vehicle[:3]
                    
                    # Transform rotation from LiDAR to vehicle frame
                    R_lidar = R.from_euler('z', yaw).as_matrix()
                    R_vehicle = self.T_l2v[:3, :3] @ R_lidar
                    vehicle_yaw, vehicle_pitch, vehicle_roll = R.from_matrix(R_vehicle).as_euler('zyx', degrees=False)
                    
                    boxes = add_bounding_box(boxes=boxes, 
                        frame_id='currentVehicleFrame', 
                        stamp=lidar_msg.header.stamp, 
                        x=x_vehicle, 
                        y=y_vehicle, 
                        z=z_vehicle, 
                        l=l, # length 
                        w=w, # width 
                        h=h, # height 
                        yaw=vehicle_yaw,
                        conf_score=score,
                        label=label # person/pedestrian class
                    )
                    
                    rospy.loginfo(f"Pedestrian detected at ({x:.2f}, {y:.2f}, {z:.2f}) with score {score:.2f}")

            # Publish the bounding boxes
            rospy.loginfo(f"Publishing {len(boxes.boxes)} pedestrian bounding boxes")
            self.pub.publish(boxes)

if __name__ == '__main__':
    try:
        node = PointPillarsNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass