# from ...state import AllState, VehicleState, ObjectPose, ObjectFrameEnum, AgentState, AgentEnum, AgentActivityEnum
# from ..interface.gem import GEMInterface
# from ..component import Component
# from perception_utils import *
import numpy as np
from scipy.spatial.transform import Rotation as R
import rospy
from sensor_msgs.msg import PointCloud2, Image
from message_filters import Subscriber, ApproximateTimeSynchronizer
from cv_bridge import CvBridge
import time
import os

# PointPillars imports:
import torch
from pointpillars.model import PointPillars

# Publisher imports:
from jsk_recognition_msgs.msg import BoundingBox, BoundingBoxArray
from geometry_msgs.msg import Pose, Vector3


import numpy as np
import ros_numpy
def pc2_to_numpy(pc2_msg, want_rgb=False):
    """
    Convert a ROS PointCloud2 message into a numpy array quickly using ros_numpy.
    This function extracts the x, y, z coordinates from the point cloud.
    """
    print(pc2_msg.fields)
    # Convert the ROS message to a numpy structured array
    pc = ros_numpy.point_cloud2.pointcloud2_to_array(pc2_msg)
    # Stack x,y,z fields to a (N,3) array
    pts = np.stack((np.array(pc['x']).ravel(),
                    np.array(pc['y']).ravel(),
                    np.array(pc['z']).ravel(),
                    np.array(pc['intensity']).ravel()), axis=1)
    # Apply filtering (for example, x > 0 and z in a specified range)
    mask = (pts[:, 0] > -0.5) & (pts[:, 2] < -1) & (pts[:, 2] > -2.7)
    return pts[mask]

class PointPillarsNode():
    """
    Detects Pedestrians by fusing YOLO 2D detections with LiDAR point cloud 
    data by painting the points. Pedestrians are also detected with PointPillars
    on the point cloud. The resulting 3D bounding boxes of each are fused together
    with late sensor fusion.

    Tracking is optional: set `enable_tracking=False` to disable persistent tracking
    and return only detections from the current frame.
    """

    def __init__(
        self,
    ):
        self.latest_image        = None
        self.latest_lidar        = None
        self.bridge              = CvBridge()
        self.camera_name = 'front'
        self.camera_front = (self.camera_name=='front')
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

        # Subscribe to the RGB and LiDAR streams
        self.rgb_sub = Subscriber(rgb_topic, Image)
        self.lidar_sub = Subscriber('/ouster/points', PointCloud2)
        self.sync = ApproximateTimeSynchronizer([
            self.rgb_sub, self.lidar_sub
        ], queue_size=500, slop=0.05)
        self.sync.registerCallback(self.synchronized_callback)
        
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

        # if torch.cuda.is_available():
        #     self.pointpillars = self.pointpillars.cuda()

        self.pointpillars.eval()
        rospy.loginfo("PointPillars model loaded successfully")

        # Create bounding box publisher
        self.pub = rospy.Publisher('/pointpillars_boxes', BoundingBoxArray, queue_size=10)

        rospy.loginfo("PointPillars node initialized and waiting for messages.")

    def synchronized_callback(self, image_msg, lidar_msg):
        rospy.loginfo("Received synchronized RGB and LiDAR messages")
        self.latest_lidar = pc2_to_numpy(lidar_msg, want_rgb=False)

        downsample = False

        if downsample:
            lidar_down = downsample_points(self.latest_lidar, voxel_size=0.1)
        else:
            lidar_down = self.latest_lidar.copy()

        boxes = BoundingBoxArray()
        boxes.header.frame_id = 'velodyne'
        boxes.header.stamp = lidar_msg.header.stamp

        pointpillars_detections = []
        with torch.no_grad():
            # Convert to tensor and format for PointPillars
            lidar_tensor = torch.from_numpy(lidar_down).float()
            if torch.cuda.is_available():
                lidar_tensor = lidar_tensor.cuda()
            
            # Format as batch with a single point cloud
            batched_pts = [lidar_tensor]
            
            # Get PointPillars predictions
            results = self.pointpillars(batched_pts, mode='test')
            
            if results and len(results) > 0 and 'lidar_bboxes' in results[0]:
                # Process PointPillars results
                pp_bboxes = results[0]['lidar_bboxes']
                pp_labels = results[0]['labels'] 
                pp_scores = results[0]['scores']

                for i, (bbox, label, score) in enumerate(zip(pp_bboxes, pp_labels, pp_scores)):
                    # Skip low confidence detections
                    if score < 0.5:  # Adjust threshold as needed
                        continue

                    rospy.loginfo(f"PointPillars detected {bbox} objects")
                        
                    # Extract center position and dimensions
                    x, y, z, l, w, h, yaw = bbox
                    center = np.array([x, y, z])
                    dims = (l, w, h)
                    
                    pointpillars_detections.append({
                        'pose': pp_pose,
                        'dimensions': dims,
                        'activity': activity,
                        'score': score
                    })
                    
        rospy.loginfo(f"PointPillars detected {len(pointpillars_detections)} objects")

if __name__ == '__main__':
    try:
        node = PointPillarsNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass