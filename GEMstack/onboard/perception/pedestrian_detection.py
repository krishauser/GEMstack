from ...state import AllState, VehicleState, ObjectPose, ObjectFrameEnum, AgentState, AgentEnum, AgentActivityEnum
from ..interface.gem import GEMInterface
from ..component import Component
from ultralytics import YOLO
import cv2
from typing import Dict
import open3d as o3d
import numpy as np
from sklearn.cluster import DBSCAN
from scipy.spatial.transform import Rotation as R

def backproject_pixel(u, v, K):
    """Backprojects pixel (u,v) into a normalized 3D ray (camera coordinates)."""
    cx, cy = K[0, 2], K[1, 2]
    fx, fy = K[0, 0], K[1, 1]
    x = (u - cx) / fx
    y = (v - cy) / fy
    ray_dir = np.array([x, y, 1.0])
    return ray_dir / np.linalg.norm(ray_dir)

def find_human_center_on_ray(lidar_pc, ray_origin, ray_direction,
                             t_min, t_max, t_step,
                             distance_threshold, min_points, ransac_threshold):
    """
    Sweep along the ray and return the first candidate point where a sufficient number
    of LiDAR points are found. (For simplicity, plane fitting is omitted.)
    """
    t_values = np.arange(t_min, t_max, t_step)
    for t in t_values:
        candidate = ray_origin + t * ray_direction
        dists = np.linalg.norm(lidar_pc - candidate, axis=1)
        if np.sum(dists < distance_threshold) >= min_points:
            return candidate, None, None
    return None, None, None

def extract_roi(pc, center, roi_radius):
    """Extract points from pc that lie within roi_radius of center."""
    distances = np.linalg.norm(pc - center, axis=1)
    return pc[distances < roi_radius]

def refine_cluster(roi_points, center, eps=0.2, min_samples=10):
    """Refine a cluster using DBSCAN and return the cluster closest to center."""
    clustering = DBSCAN(eps=eps, min_samples=min_samples).fit(roi_points)
    labels = clustering.labels_
    valid_clusters = [roi_points[labels == l] for l in set(labels) if l != -1]
    if not valid_clusters:
        return roi_points
    best_cluster = min(valid_clusters, key=lambda c: np.linalg.norm(np.mean(c, axis=0) - center))
    return best_cluster

def remove_ground_by_min_range(cluster, z_range=0.05):
    """
    Remove ground points by computing the minimum z value in the cluster and eliminating
    all points with z within z_range of that minimum.
    """
    if cluster is None or cluster.shape[0] == 0:
        return cluster
    min_z = np.min(cluster[:, 2])
    filtered = cluster[cluster[:, 2] > (min_z + z_range)]
    return filtered

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
        # self.detector = YOLO('../../knowledge/detection/yolov11n.pt')
        self.last_person_boxes = []

    def rate(self):
        return 4.0
    
    def state_inputs(self):
        return ['vehicle']
    
    def state_outputs(self):
        return ['agents']
    
    def initialize(self):
        #tell the vehicle to use image_callback whenever 'front_camera' gets a reading, and it expects images of type cv2.Mat
        self.detector = YOLO('../../knowledge/detection/yolov8n.pt')
        self.vehicle_interface.subscribe_sensor('front_camera', self.image_callback, cv2.Mat)
        # Set up camera intrinsics and LiDAR-to-camera transformation.
        self.K = np.array([[684.83331299, 0., 573.37109375],
                           [0., 684.60968018, 363.70092773],
                           [0., 0., 1.]])
        self.T_l2c = np.array([[-0.01909581, -0.9997844, 0.0081547, 0.24521313],
                                [0.06526397, -0.00938524, -0.9978239, -0.80389025],
                                [0.9976853, -0.01852205, 0.06542912, -0.6605772],
                                [0., 0., 0., 1.]])
        self.T_c2l = np.linalg.inv(self.T_l2c)
        self.R_c2l = self.T_c2l[:3, :3]
        self.camera_origin_in_lidar = self.T_c2l[:3, 3]

    def image_callback(self, image: cv2.Mat):
        """Image processing callback with original detection logic"""
        results = self.detector(image, conf=0.5, classes = [0])
        boxes = results[0].boxes.xywh.cpu().tolist()  # Format: [center_x, center_y, w, h]
        self.last_person_boxes = boxes

    def update(self, vehicle: VehicleState) -> dict:
        agents = {}
        # Assume LiDAR point cloud is available in vehicle.lidar as a (N,3) np.ndarray.
        lidar_pc = vehicle.lidar
        for i, box in enumerate(self.last_person_boxes):
            cx, cy, w, h = box
            # Backproject the center pixel into a ray.
            ray_dir_cam = backproject_pixel(cx, cy, self.K)
            ray_dir_lidar = self.R_c2l @ ray_dir_cam
            ray_dir_lidar /= np.linalg.norm(ray_dir_lidar)
            # Sweep along the ray to get an initial 3D candidate.
            intersection, _, _ = find_human_center_on_ray(lidar_pc, self.camera_origin_in_lidar, ray_dir_lidar,
                                                          t_min=0.5, t_max=20.0, t_step=0.1,
                                                          distance_threshold=0.2, min_points=20, ransac_threshold=0.05)
            if intersection is None:
                continue
            # Extract an ROI around the candidate and refine it.
            roi_points = extract_roi(lidar_pc, intersection, roi_radius=1.0)
            if roi_points.shape[0] < 20:
                refined_cluster = roi_points
            else:
                refined_cluster = refine_cluster(roi_points, intersection, eps=0.15, min_samples=10)
            refined_cluster = remove_ground_by_min_range(refined_cluster, z_range=0.05)
            if refined_cluster is None or refined_cluster.shape[0] == 0:
                refined_center = intersection
                dims = (0, 0, 0)
                yaw, pitch, roll = 0, 0, 0
            else:
                # Compute oriented bounding box from the refined cluster.
                pcd = o3d.geometry.PointCloud()
                pcd.points = o3d.utility.Vector3dVector(refined_cluster)
                obb = pcd.get_oriented_bounding_box()
                refined_center = obb.center
                dims = tuple(obb.extent)
                euler_angles = R.from_matrix(obb.R.copy()).as_euler('zyx', degrees=True)
                yaw, pitch, roll = euler_angles[0], euler_angles[1], euler_angles[2]
            # Create the agent pose and state.
            pose = ObjectPose(t=vehicle.time, x=refined_center[0], y=refined_center[1],
                              z=refined_center[2], yaw=yaw, pitch=pitch, roll=roll,
                              frame=ObjectFrameEnum.CURRENT)
            agent_state = AgentState(pose=pose, dimensions=dims, outline=None,
                                     type=AgentEnum.PEDESTRIAN, activity=AgentActivityEnum.MOVING,
                                     velocity=(0, 0, 0), yaw_rate=0)
            agents[f'pedestrian{i}'] = agent_state
        return agents


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
