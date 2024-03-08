""" Instructions:
From the main GEMstack folder, run
python3 main.py --variant=detector_only launch/pedestrian_avoidance.yaml
"""

from ...state import AllState,VehicleState,ObjectPose,ObjectFrameEnum,AgentState,AgentEnum,AgentActivityEnum
from ...utils import settings
from ...mathutils import transforms
from ..interface.gem import GEMInterface
from ..component import Component
from ultralytics import YOLO
import cv2
try:
    from sensor_msgs.msg import CameraInfo
    from image_geometry import PinholeCameraModel
    import rospy
except ImportError:
    pass
import numpy as np
from typing import Dict,Tuple
import time
import yaml

def project_point_cloud(point_cloud : np.ndarray, P : np.ndarray, xrange : Tuple[int,int], yrange : Tuple[int,int]) -> Tuple[np.ndarray,np.ndarray]:
    """Projects a point cloud into a 2D image using a camera intrinsic projection matrix P.
    
    Returns:
        - point_cloud_image: an Nx2 array of (u,v) visible image coordinates
        - image_indices: an array of N indices of visible points into the original point cloud
    """
    #this is the easy but slow way
    # camera = PinholeCameraModel()
    # camera.fromCameraInfo(self.camera_info)
    # for i,p in enumerate(self.point_cloud_zed):
    #     if p[2] < 0:
    #         continue
    #     u,v = camera.project3dToPixel(p[:3])
    #     if u >= 0 and u < self.camera_info.width and v >= 0 and v < self.camera_info.height:
    #         point_cloud_image.append((u,v,i))
    # point_cloud_image = np.array(point_cloud_image)
    # image_indices = point_cloud_image[:,2].astype(int)
    
    #this is the hard but fast way

    pc_with_ids = np.hstack((point_cloud,np.arange(len(point_cloud)).reshape(-1,1)))
    pc_fwd = pc_with_ids[pc_with_ids[:,2] > 0]
    pxform = pc_fwd[:,:3].dot(P[:3,:3].T) + P[:3,3]
    uv = (pxform[:,0:2].T/pxform[:,2]).T
    inds = np.logical_and(np.logical_and(uv[:,0] >= xrange[0],uv[:,0] < xrange[1]),
                          np.logical_and(uv[:,1] >= yrange[0],uv[:,1] < yrange[1]))
    point_cloud_image = uv[inds]
    image_indices = pc_fwd[inds,3].astype(int)
    return point_cloud_image, image_indices


class PedestrianDetector(Component):
    """Detects and tracks pedestrians."""
    def __init__(self,vehicle_interface : GEMInterface):
        self.vehicle_interface = vehicle_interface
        self.detector = YOLO('GEMstack/knowledge/detection/yolov8n.pt')
        self.camera_info_sub = None
        self.camera_info = None
        self.zed_image = None
        self.last_person_boxes = []
        self.lidar_translation = np.array(settings.get('vehicle.calibration.top_lidar.position'))
        self.lidar_rotation = np.array(settings.get('vehicle.calibration.top_lidar.rotation'))
        self.zed_translation = np.array(settings.get('vehicle.calibration.front_camera.rgb_position'))
        self.zed_rotation = np.array(settings.get('vehicle.calibration.front_camera.rotation'))
        self.T_lidar = np.eye(4)
        self.T_lidar[:3,:3] = self.lidar_rotation
        self.T_lidar[:3,3] = self.lidar_translation
        self.T_zed = np.eye(4)
        self.T_zed[:3,:3] = self.zed_rotation
        self.T_zed[:3,3] = self.zed_translation
        self.T_lidar_to_zed = np.linalg.inv(self.T_zed) @ self.T_lidar
        self.point_cloud = None
        self.point_cloud_zed = None
        assert(settings.get('vehicle.calibration.top_lidar.reference') == 'rear_axle_center')
        assert(settings.get('vehicle.calibration.front_camera.reference') == 'rear_axle_center')
        self.pedestrian_counter = 0
        self.last_agent_states = {}

    def rate(self):
        return settings.get('perception.pedestrian_detection.rate')
    
    def state_inputs(self):
        return ['vehicle']
    
    def state_outputs(self):
        return ['agents']
    
    def initialize(self):
        #tell the vehicle to use image_callback whenever 'front_camera' gets a reading, and it expects images of type cv2.Mat
        self.vehicle_interface.subscribe_sensor('front_camera',self.image_callback,cv2.Mat)
        #tell the vehicle to use lidar_callback whenever 'top_lidar' gets a reading, and it expects numpy arrays
        self.vehicle_interface.subscribe_sensor('top_lidar',self.lidar_callback,np.ndarray)
        #subscribe to the Zed CameraInfo topic
        self.camera_info_sub = rospy.Subscriber("/zed2/zed_node/rgb/camera_info", CameraInfo, self.camera_info_callback)
    
    def image_callback(self, image : cv2.Mat):
        self.zed_image = image

    # TODO: uncomment before running on the vehicle
    # def camera_info_callback(self, info : CameraInfo):
    #     self.camera_info = info

    def lidar_callback(self, point_cloud: np.ndarray):
        self.point_cloud = point_cloud
    
    def update(self, vehicle : VehicleState) -> Dict[str,AgentState]:
        if self.zed_image is None:
            #no image data yet
            return {}
        if self.point_cloud is None:
            #no lidar data yet
            return {}
        if self.camera_info is None:
            #no camera info yet
            return {}
        
        #debugging
        #self.save_data()

        t1 = time.time()
        detected_agents = self.detect_agents()

        t2 = time.time()
        current_agent_states = self.track_agents(vehicle,detected_agents)
        t3 = time.time()
        print("Detection time",t2-t1,", shape estimation and tracking time",t3-t2)

        self.last_agent_states = current_agent_states
        return current_agent_states

    def box_to_agent(self, box, point_cloud_image, point_cloud_image_world):
        """Creates a 3D agent state from an (x,y,w,h) bounding box.
        
        Uses the image, the camera intrinsics, the lidar point cloud, 
        and the calibrated camera / lidar poses to get a good estimate 
        of the pedestrian's pose and dimensions.
        """
    
        # We first find the points in point_cloud_image that are within the bounding box
        x, y, w, h = box
        # print('Bbox: [{0:.2f}, {1:.2f}, {2:.2f}, {3:.2f}]'.format(x,y,w,h))

        points_in_box_idx = [i for i in range(len(point_cloud_image)) if 
                            point_cloud_image[i][0] >= x - w/2 and 
                            point_cloud_image[i][1] >= y - w/2 and 
                            point_cloud_image[i][0] <= x + w/2 and 
                            point_cloud_image[i][1] <= y + h/2]
        points_in_box = [point_cloud_image_world[idx] for idx in points_in_box_idx]
        # print("points_in_box", points_in_box)

        position = np.mean(points_in_box, axis=0)

        # Estimate dimensions - assuming the pedestrian is upright for simplicity
        # This could be refined with a more sophisticated model
        min_pt = np.min(points_in_box, axis=0)
        max_pt = np.max(points_in_box, axis=0)
        dimensions = max_pt - min_pt
        # print("dimensions", dimensions)

        # You might want to adjust the dimensions if they are too small or too large
        # This is a simple heuristic that may need refinement
        dims = [max(dimensions[0], 0.5), max(dimensions[1], 0.5), max(dimensions[2], 1.5)]

        # Create the agent state with the estimated position and dimensions
        pose = ObjectPose(t=0, x=position[0], y=position[1], z=position[2], 
                          yaw=0, pitch=0, roll=0, frame=ObjectFrameEnum.CURRENT)
        
        return AgentState(pose=pose, dimensions=dims, outline=None, 
                          type=AgentEnum.PEDESTRIAN, activity=AgentActivityEnum.MOVING, 
                          velocity=(0,0,0), yaw_rate=0)
    
    def detect_agents(self):
        detection_result = self.detector(self.zed_image, classes=0, verbose=False)
        # print(detection_result)

        # Create boxes from detection result
        bboxes = []
        for box in detection_result[0].boxes:
            xywh = box.xywh.cpu().clone().numpy().reshape(4, )
            bboxes.append(xywh)
        self.last_person_boxes = bboxes
        
        # Create point clouds in image frame and world frame
        point_cloud_image, point_cloud_image_world = self.point_cloud_image()

        detected_agents = []     
        for i,b in enumerate(self.last_person_boxes):
            agent = self.box_to_agent(b, point_cloud_image, point_cloud_image_world)
            detected_agents.append(agent)
            
        return detected_agents
    
    def track_agents(self, vehicle : VehicleState, detected_agents : list[AgentState]):
        """Given a list of detected agents, updates the state of the agents."""
        # TODO: keep track of which pedestrians were detected before using last_agent_states.
        # use these to assign their ids and estimate velocities.
        # return None
        results = {}
        for i,a in enumerate(detected_agents):
            results['pedestrian_'+str(self.pedestrian_counter)] = a
            self.pedestrian_counter += 1

            last = self.last_agent_states[i]
            current_pose = a.ObjectPose
            dimensions  = a.dimensions
            x_velocity = (current_pose.x - last.x) / (current_pose.t - last.t)
            y_velocity = (current_pose.y - last.y) / (current_pose.t - last.t)
            z_velocity = (current_pose.z - last.z) / (current_pose.t - last.t)
            yaw_rate   = (current_pose.yaw - last.yaw) / (current_pose.t - last.t)

            current_state = AgentState(pose=current_pose, dimensions=dimensions, outline=None, 
                                    type=AgentEnum.PEDESTRIAN, activity=AgentActivityEnum.MOVING, 
                                    velocity=(x_velocity,y_velocity,z_velocity), yaw_rate=yaw_rate)
            results.append(current_state)
        return results

    def save_data(self, loc=None):
        """This can be used for debugging.  See the provided test."""
        prefix = ''
        if loc is not None:
            prefix = loc + '/'
        cv2.imwrite(prefix+'zed_image.png',self.zed_image)
        np.savez(prefix+'velodyne_point_cloud.npz',self.point_cloud)
        import pickle
        with open(prefix+'zed_camera_info.pkl','wb') as f:
            pickle.dump(self.camera_info,f)

    def load_data(self, loc=None):
        prefix = ''
        if loc is not None:
            prefix = loc + '/'
        self.zed_image = cv2.imread(prefix+'zed_image.png')
        self.point_cloud = np.load(prefix+'velodyne_point_cloud.npz')['arr_0']
        try:
            import pickle
            with open(prefix+'zed_camera_info.pkl','rb') as f:
                self.camera_info = pickle.load(f)
        except ModuleNotFoundError:
            #ros not found?            
            with open(settings.get('calibration.zed_intrinsics'), 'r') as file:
                config = yaml.load(file, yaml.SafeLoader)
            
            P = np.zeros((3,4))
            P[:3,:3] = np.array(config['K']).reshape(3,3)
            
            from collections import namedtuple
            CameraInfo = namedtuple('CameraInfo',['width','height','P'])
            self.camera_info = CameraInfo(width=1280, height=720, P=P.flatten())

    def point_cloud_image(self):
        # Transform LiDAR points to the camera frame
        point_cloud_homogeneous = np.hstack((self.point_cloud, np.ones((self.point_cloud.shape[0], 1))))
        point_cloud_camera_frame = (self.T_lidar_to_zed @ point_cloud_homogeneous.T).T[:, :3]
        
        # Project the 3D points in the camera frame onto the 2D image
        projected_points, image_indices = project_point_cloud(
            point_cloud_camera_frame, np.array(self.camera_info.P).reshape(3,4), 
            [0, self.camera_info.width], [0, self.camera_info.height]
        )

        point_cloud_image = projected_points   # (u,v) visible image coordinates
        # print('Point cloud image:\n', point_cloud_image)

        point_cloud_camera_frame = point_cloud_camera_frame[image_indices] 

        # Convert the visible points from camera frame to vehicle frame
        pcd_temp = np.ones((4, len(point_cloud_camera_frame)))
        pcd_temp[:3, :] = point_cloud_camera_frame.transpose()
        point_cloud_image_world = (self.T_zed @ pcd_temp).T[:, :3]

        return point_cloud_image, point_cloud_image_world
