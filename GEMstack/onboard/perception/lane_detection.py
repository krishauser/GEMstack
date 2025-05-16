
from CLRerNet.libs.datasets.pipelines import Compose
from CLRerNet.libs.api.inference import get_prediction
from CS588AD.GEMstack.GEMstack.state.roadgraph import RoadgraphSimpleLane
import cv2
import torch

from mmdet.apis import init_detector

from ...utils import settings
from ...state import AllState, VehicleState, ObjectPose, ObjectFrameEnum, Obstacle, ObstacleMaterialEnum, \
    ObstacleStateEnum
from ..interface.gem import GEMInterface
from ..component import Component
from .perception_utils import *
from ultralytics import YOLO
import cv2
from typing import Dict
import open3d as o3d
import numpy as np
from scipy.spatial.transform import Rotation as R
import rospy
from sensor_msgs.msg import PointCloud2, Image
from message_filters import Subscriber, ApproximateTimeSynchronizer
from cv_bridge import CvBridge
import time
import os
import yaml


class LaneDetector3D(Component):
    """
    Detects lanes by using CLRerNet and do ICM.

    Tracking is optional: set `enable_tracking=False` to disable persistent tracking
    and return only detections from the current frame.

    Supports multiple cameras; each camera’s intrinsics and extrinsics are
    loaded from a single YAML calibration file via plain PyYAML and global settings.
    """

    def __init__(
            self,
            vehicle_interface: GEMInterface,
            camera_name: str,
            camera_calib_file: str,
            enable_tracking: bool = True,
            visualize_2d: bool = False,
            use_cyl_roi: bool = False,
            save_data: bool = True,
            orientation: bool = True,
            use_start_frame: bool = True,
            **kwargs
    ):
        # Core interfaces and state
        self.vehicle_interface = vehicle_interface
        self.current_lanes = {}
        # self.tracked_lanes = {}
        self.lane_counter = 0
        self.latest_image = None
        self.bridge = CvBridge()
        self.start_pose_abs = None
        self.start_time = None

        # Config flags
        self.camera_name = camera_name
        self.enable_tracking = enable_tracking
        self.visualize_2d = visualize_2d
        self.use_cyl_roi = use_cyl_roi
        self.save_data = save_data
        self.orientation = orientation
        self.use_start_frame = use_start_frame

        # 2) Load camera intrinsics/extrinsics from the supplied YAML
        with open(camera_calib_file, 'r') as f:
            calib = yaml.safe_load(f)

        # Expect structure:
        # cameras:
        #   front:
        #     K:   [[...], [...], [...]]
        #     D:   [...]
        #     T_l2c: [[...], ..., [...]]
        cam_cfg = calib['cameras'][camera_name]
        self.K = np.array(cam_cfg['K'])
        self.D = np.array(cam_cfg['D'])
        self.T_l2c = np.array(cam_cfg['T_l2c'])
        self.T_l2v = np.array(cam_cfg['T_l2v'])

        # Expect structure:
        #front_camera:
        #  extrinsics:
        #    rotation: [[...], [...], [...]]
        #    position: [[...], [...], [...]]
        self.R = np.array(settings.get(f'calibration.{camera_name}_camera.extrinsics.rotation'))
        self.T = np.array(settings.get(f'calibration.{camera_name}_camera.extrinsics.position'))

        # Derived transforms

        self.undistort_map1 = None
        self.undistort_map2 = None
        self.camera_front = (camera_name == 'front')

    def rate(self) -> float:
        return 8

    def state_inputs(self) -> list:
        return ['vehicle']

    def state_outputs(self) -> list:
        return ['lanes']

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

        # Subscribe to the RGB and LiDAR streams
        self.rgb_sub = Subscriber(rgb_topic, Image)
        self.sync = ApproximateTimeSynchronizer([
            self.rgb_sub, self.lidar_sub
        ], queue_size=500, slop=0.03)
        self.sync.registerCallback(self.synchronized_callback)

        # Initialize the YOLO detector

        self.detector = init_detector('CLRerNet/configs/lane_detection.py', 'CLRerNet/checkpoints/lane_detection.pth', device='cuda')
        self.detector.to('cuda')
        self.T_c2l = np.linalg.inv(self.T_l2c)
        self.R_c2l = self.T_c2l[:3, :3]

    def synchronized_callback(self, image_msg, lidar_msg):
        '''
        Get the latest image.
        '''
        step1 = time.time()
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
        except Exception as e:
            rospy.logerr("Failed to convert image: {}".format(e))
            self.latest_image = None
        step2 = time.time()
        # print('image callback: ', step2 - step1, 'lidar callback ', step3 - step2)

    def undistort_image(self, image, K, D):
        '''
        Undistort the image using the camera intrinsic matrix and distortion coefficients.
        '''
        h, w = image.shape[:2]
        newK, _ = cv2.getOptimalNewCameraMatrix(K, D, (w, h), 1, (w, h))
        if self.undistort_map1 is None or self.undistort_map2 is None:
            self.undistort_map1, self.undistort_map2 = cv2.initUndistortRectifyMap(K, D, R=None,
                                                                                   newCameraMatrix=newK, size=(w, h),
                                                                                   m1type=cv2.CV_32FC1)

        start = time.time()
        undistorted = cv2.remap(image, self.undistort_map1, self.undistort_map2, interpolation=cv2.INTER_NEAREST)
        end = time.time()
        # print('--------undistort', end-start)
        return undistorted, newK

    def inference_one_imave(self, model, img):
        """Inference on an image with the detector.
        Args:
            model (nn.Module): The loaded detector.
            img_path (str): Image path.
        Returns:
            img (np.ndarray): Image data with shape (width, height, channel).
            preds (List[np.ndarray]): Detected lanes.
        """
        ori_shape = img.shape
        data = dict(
            sub_img_name=None,
            img=img,
            gt_points=[],
            id_classes=[],
            id_instances=[],
            img_shape=ori_shape,
            ori_shape=ori_shape,
        )

        cfg = model.cfg
        model.bbox_head.test_cfg.as_lanes = False

        test_pipeline = Compose(cfg.test_dataloader.dataset.pipeline)

        data = test_pipeline(data)
        data_ = dict(
            inputs=[data["inputs"]],
            data_samples=[data["data_samples"]],
        )

        # forward the model
        with torch.no_grad():
            results = model.test_step(data_)

        lanes = results[0]['lanes']
        preds = get_prediction(lanes, ori_shape[0], ori_shape[1])

        return img, preds
    

    def ipm_2d_to_3d(self, points_2d, K, R, T):
        """
        Projects 2D image lane points to 3D ground-plane coordinates using IPM.
        
        Args:
            points_2d: (N, 2) array of 2D points in image space (u, v)
            K: (3, 3) camera intrinsic matrix
            R: (3, 3) camera rotation matrix (world to camera)
            T: (3, 1) camera translation vector (world to camera)

        Returns:
            points_3d: (N, 3) array of 3D ground-plane points in world coordinates
        """
        # Inverse camera extrinsics: camera-to-world transformation
        Rt = np.hstack((R, T))  # (3,4)
        cam_to_world = np.linalg.inv(np.vstack((Rt, [0, 0, 0, 1])))  # (4,4)

        # Inverse of camera intrinsics
        K_inv = np.linalg.inv(K)

        points_3d = []
        for (u, v) in points_2d:
            uv1 = np.array([u, v, 1.0])
            ray_cam = K_inv @ uv1  # normalized camera coords
            ray_cam = ray_cam / np.linalg.norm(ray_cam)

            # Extend to homogeneous 4D ray direction
            ray_dir_homo = np.append(ray_cam, [0.0])  # direction (no translation)
            cam_origin_homo = np.array([0.0, 0.0, 0.0, 1.0])  # camera center

            # Transform ray to world coordinates
            ray_origin_world = cam_to_world @ cam_origin_homo
            ray_dir_world = cam_to_world @ ray_dir_homo
            ray_dir_world = ray_dir_world[:3]
            ray_origin_world = ray_origin_world[:3]

            # Intersect with ground plane Z=0: solve for t in O + tD → Z=0
            t = -ray_origin_world[2] / ray_dir_world[2]
            point_ground = ray_origin_world + t * ray_dir_world
            points_3d.append(point_ground)

        return np.array(points_3d)


    def update(self, vehicle: VehicleState) -> Dict[str, RoadgraphSimpleLane]:
        downsample = False
        # Gate guards against data not being present for both sensors:
        if self.latest_image is None or self.latest_lidar is None:
            return {}
        latest_image = self.latest_image.copy()

        # Set up current time variables
        start = time.time()
        current_time = self.vehicle_interface.time()

        if downsample:
            lidar_down = downsample_points(self.latest_lidar, voxel_size=0.1)
        else:
            lidar_down = self.latest_lidar.copy()

        if self.start_time is None:
            self.start_time = current_time
        time_elapsed = current_time - self.start_time

        # Ensure data/ exists and build timestamp
        # if self.save_data:
        #     self.save_sensor_data(vehicle=vehicle, latest_image=latest_image)

        if self.camera_front == False:
            start = time.time()
            undistorted_img, current_K = self.undistort_image(latest_image, self.K, self.D)
            end = time.time()
            # print('-------processing time undistort_image---', end -start)
            self.current_K = current_K
            orig_H, orig_W = undistorted_img.shape[:2]

            # --- Begin modifications for three-angle detection ---
            img_normal = undistorted_img
        else:
            img_normal = latest_image.copy()
            undistorted_img = latest_image.copy()
            orig_H, orig_W = latest_image.shape[:2]
            self.current_K = self.K
        img, results_normal = self.inference_one_imave(self.detector, img_normal)
        combined_lanes = []
        combined_lanes_3d = []
        if not self.enable_tracking:
            self.lane_counter = 0

        for lane in results_normal:
            combined_lanes.append(lane)
            combined_lanes_3d.append(self.ipm_2d_to_3d(lane, self.current_K, self.R, self.T))

        # Visualize the received images in 2D with their corresponding labels
        # It draws rectangles and labels on the images:
        if getattr(self, 'visualize_2d', False):
            for (x, y) in combined_lanes:
                # Radius of circle
                radius = 2
                
                # Blue color in BGR
                color = (255, 0, 0)
                
                # Line thickness of 2 px
                thickness = 2
                
                # Using cv2.circle() method
                # Draw a circle with blue line borders of thickness of 2 px
                cv2.circle(undistorted_img, (x, y), radius, color, thickness)
            cv2.imshow("Detection - Lane 2D", undistorted_img)

        # print('-------processing time1---', end -start)

        lanes = {f"Lane{index}": RoadgraphSimpleLane(points=lane) for index, lane in enumerate(combined_lanes_3d)}
        self.current_lanes = lanes

        # If tracking not enabled, return only current frame detections
        if not self.enable_tracking:
            for lane_id, lane in self.current_lanes.items():
                p = lane
                rospy.loginfo(
                    f"Lane ID: {lane_id}\n"
                    f"Points: {p}\n"
                )
            end = time.time()
            # print('-------processing time', end -start)
            
        return self.current_lanes
