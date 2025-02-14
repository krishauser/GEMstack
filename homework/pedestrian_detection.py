from ...state import AllState, VehicleState, ObjectPose, ObjectFrameEnum, AgentState, AgentEnum, AgentActivityEnum
from ..interface.gem import GEMInterface
from ..component import Component
from ultralytics import YOLO
import cv2
import numpy as np
from typing import Dict

# =============================================================================
# Global camera intrinsics (placeholder values – move these to your settings file later)
# The camera frame is assumed to be:
#    - x: right, y: down, z: forward
# These values should be obtained via calibration (e.g., from the /oak/rgb/camera_info topic)
# =============================================================================
CAMERA_INTRINSICS = {
    'fx': 500.0,  # focal length in pixels (replace with your calibrated value)
    'fy': 500.0,
    'cx': 320.0,  # principal point x (replace with your calibrated value)
    'cy': 240.0,  # principal point y (replace with your calibrated value)
}

# =============================================================================
# LIDAR to Camera Transform (placeholder, replace with your actual calibration matrix)
# This 4x4 homogeneous transformation converts points in the lidar frame to the camera frame.
# Replace the identity matrix below with your actual calibration data.
# =============================================================================
LIDAR_TO_CAMERA_TRANSFORM = np.eye(4)  # TODO: replace with your calibrated transform


def box_to_fake_agent(box):
    """Creates a fake agent state from an (x,y,w,h) bounding box.
    
    The location and size are placeholders, since this is just a 2D approximation.
    """
    x, y, w, h = box
    pose = ObjectPose(t=0, x=x + w / 2, y=y + h / 2, z=0,
                      yaw=0, pitch=0, roll=0, frame=ObjectFrameEnum.CURRENT)
    dims = (w, h, 0)
    return AgentState(pose=pose, dimensions=dims, outline=None, type=AgentEnum.PEDESTRIAN,
                      activity=AgentActivityEnum.MOVING, velocity=(0, 0, 0), yaw_rate=0)


def get_average_cam_point_from_lidar(bbox, point_cloud):
    """
    Given a bounding box (x, y, w, h) in image coordinates and a lidar point cloud (in the lidar frame),
    this function:
      1. Converts the point cloud to homogeneous coordinates.
      2. Transforms the points into the camera frame using LIDAR_TO_CAMERA_TRANSFORM.
      3. Projects the points into the image using the camera intrinsics.
      4. Selects points that fall inside the bounding box.
      5. Returns the average 3D point (in the camera frame) of those points.
    
    Returns:
        avg_point: A NumPy array [x_cam, y_cam, z_cam] in the camera frame,
                   or None if no points are found inside the bbox.
    """
    x, y, w, h = bbox
    if point_cloud is None or point_cloud.shape[0] == 0:
        return None

    # Convert the point cloud (assumed shape (N,3)) to homogeneous coordinates (N,4)
    ones = np.ones((point_cloud.shape[0], 1))
    points_homog = np.hstack((point_cloud, ones))  # shape (N,4)
    
    # Transform points from lidar frame to camera frame
    cam_points_homog = (LIDAR_TO_CAMERA_TRANSFORM @ points_homog.T).T  # shape (N,4)
    cam_points = cam_points_homog[:, :3]  # (x_cam, y_cam, z_cam)
    
    # Only consider points in front of the camera (z_cam > 0)
    valid_mask = cam_points[:, 2] > 0
    cam_points = cam_points[valid_mask]
    if cam_points.shape[0] == 0:
        return None

    # Project points into the image using the pinhole camera model:
    # u = fx * (x_cam / z_cam) + cx, v = fy * (y_cam / z_cam) + cy
    u = CAMERA_INTRINSICS['fx'] * (cam_points[:, 0] / cam_points[:, 2]) + CAMERA_INTRINSICS['cx']
    v = CAMERA_INTRINSICS['fy'] * (cam_points[:, 1] / cam_points[:, 2]) + CAMERA_INTRINSICS['cy']
    
    # Create a mask for points that fall within the bounding box.
    in_bbox_mask = (u >= x) & (u <= x + w) & (v >= y) & (v <= y + h)
    if np.sum(in_bbox_mask) == 0:
        return None
    
    selected_points = cam_points[in_bbox_mask]
    # Compute the average of the selected points (mean of x, y, and z)
    avg_point = np.mean(selected_points, axis=0)
    return avg_point


def camera_to_vehicle(cam_point):
    """
    Convert a point from the camera frame (x_cam, y_cam, z_cam) to the vehicle frame.
    Assumptions:
      - Camera frame: x to the right, y down, z forward.
      - Vehicle frame: x forward, y to the left, z up.
    
    The conversion is:
      x_vehicle = z_cam
      y_vehicle = -x_cam
      z_vehicle = -y_cam
    """
    x_cam, y_cam, z_cam = cam_point
    x_vehicle = z_cam
    y_vehicle = -x_cam
    z_vehicle = -y_cam
    return np.array([x_vehicle, y_vehicle, z_vehicle])


def box_to_agent_with_lidar(bbox, point_cloud):
    """
    Given a bounding box and a lidar point cloud, use the lidar data to estimate the 3D position
    of the detected pedestrian.
    """
    avg_cam_point = get_average_cam_point_from_lidar(bbox, point_cloud)
    if avg_cam_point is None:
        # If no lidar points fall within the bounding box, fall back to a 2D-only approximation.
        return box_to_fake_agent(bbox)
    
    # Convert the averaged 3D point (in the camera frame) to the vehicle frame.
    vehicle_point = camera_to_vehicle(avg_cam_point)
    
    # Create an ObjectPose using the vehicle coordinates.
    pose = ObjectPose(t=0, x=vehicle_point[0], y=vehicle_point[1], z=vehicle_point[2],
                      yaw=0, pitch=0, roll=0, frame=ObjectFrameEnum.CURRENT)
    
    # Estimate the dimensions in meters by projecting the bounding box dimensions using the average depth.
    z = avg_cam_point[2]
    width_m = (bbox[2] * z) / CAMERA_INTRINSICS['fx']
    height_m = (bbox[3] * z) / CAMERA_INTRINSICS['fy']
    dims = (width_m, height_m, 0.5)  # Assume a fixed thickness of 0.5 m for a pedestrian.
    
    return AgentState(pose=pose, dimensions=dims, outline=None, type=AgentEnum.PEDESTRIAN,
                      activity=AgentActivityEnum.MOVING, velocity=(0, 0, 0), yaw_rate=0)


class PedestrianDetector2D(Component):
    """Detects pedestrians by fusing 2D image detections with lidar point cloud data."""
    def __init__(self, vehicle_interface: GEMInterface):
        self.vehicle_interface = vehicle_interface
        self.detector = None             # YOLO detector for image processing
        self.last_person_boxes = []      # 2D bounding boxes from the image
        self.latest_point_cloud = None   # Latest lidar point cloud (in the lidar frame)

    def rate(self):
        return 4.0

    def state_inputs(self):
        return ['vehicle']

    def state_outputs(self):
        return ['agents']

    def initialize(self):
        # Subscribe to the front camera for 2D detection.
        self.detector = YOLO('../../knowledge/detection/yolov11n.pt')
        self.vehicle_interface.subscribe_sensor('front_camera', self.image_callback, cv2.Mat)
        # Subscribe to the lidar sensor (e.g., "top_lidar"). Assume it provides a NumPy array for the point cloud.
        self.vehicle_interface.subscribe_sensor('top_lidar', self.lidar_callback, np.ndarray)

    def image_callback(self, image: cv2.Mat):
        """Process the image to detect pedestrians using YOLO."""
        results = self.detector(image, conf=0.5)
        boxes = results[0].boxes
        if len(boxes) == 0:
            self.last_person_boxes = []
            return
        cls_ids = boxes.cls.cpu()
        xywh = boxes.xywh.cpu()
        person_mask = (cls_ids == 0)
        self.last_person_boxes = [tuple(map(float, box)) for box in xywh[person_mask]]

    def lidar_callback(self, point_cloud):
        """Receive and store the latest lidar point cloud (assumed to be a NumPy array of shape (N,3))."""
        self.latest_point_cloud = point_cloud

    def update(self, vehicle: VehicleState) -> Dict[str, AgentState]:
        res = {}
        for i, bbox in enumerate(self.last_person_boxes):
            # Fuse the 2D detection with lidar data to estimate a 3D location.
            agent_state = box_to_agent_with_lidar(bbox, self.latest_point_cloud)
            res['pedestrian' + str(i)] = agent_state
        if len(res) > 0:
            print("Detected", len(res), "pedestrians")
        return res


class FakePedestrianDetector2D(Component):
    """Triggers a pedestrian detection at some random time ranges."""
    def __init__(self, vehicle_interface: GEMInterface):
        self.vehicle_interface = vehicle_interface
        self.times = [(5.0, 20.0), (30.0, 35.0)]
        self.t_start = None

    def rate(self):
        return 4.0

    def state_inputs(self):
        return ['vehicle']

    def state_outputs(self):
        return ['agents']

    def update(self, vehicle: VehicleState) -> Dict[str, AgentState]:
        if self.t_start is None:
            self.t_start = self.vehicle_interface.time()
        t = self.vehicle_interface.time() - self.t_start
        res = {}
        for times in self.times:
            if t >= times[0] and t <= times[1]:
                res['pedestrian0'] = box_to_fake_agent((0, 0, 0, 0))
                print("Detected a pedestrian")
        return res
