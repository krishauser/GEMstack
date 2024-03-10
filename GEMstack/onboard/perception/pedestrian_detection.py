from ...state import AllState,VehicleState,ObjectPose,ObjectFrameEnum,AgentState,AgentEnum,AgentActivityEnum
from ...utils import settings
from ...mathutils import transforms,collisions
from ..interface.gem import GEMInterface
from ..component import Component
from ultralytics import YOLO
import cv2
import copy

try:
    from sensor_msgs.msg import CameraInfo
    from image_geometry import PinholeCameraModel
    import rospy
except ImportError:
    pass
import numpy as np
from typing import Dict,Tuple
import time

def Pmatrix(fx,fy,cx,cy):
    """Returns a projection matrix for a given set of camera intrinsics."""
    return np.array([[fx,0,cx,0],
                     [0,fy,cy,0],
                     [0,0,1,0]])

# Values as provided in the image
focal_length_x = settings.get('vehicle.calibration.intrinsic_fx')
focal_length_y = settings.get('vehicle.calibration.intrinsic_fy')
principal_point_x = settings.get('vehicle.calibration.intrinsic_px')
principal_point_y = settings.get('vehicle.calibration.intrinsic_py')

# Creating the camera intrinsic matrix with an added column of zeros
intrinsic_matrix = Pmatrix(focal_length_x,focal_length_y,principal_point_x,principal_point_y)

extrinsic = settings.get('vehicle.calibration.extrinsic_matrix')
       


def project_point_cloud(point_cloud : np.ndarray, P : np.ndarray, xrange : Tuple[int,int], yrange : Tuple[int,int]) -> Tuple[np.ndarray,np.ndarray]:
    """Projects a point cloud into a 2D image using a camera intrinsic projection matrix P.
    
    Returns:
        - point_cloud_image: an Nx2 array of (u,v) visible image coordinates
        - image_indices: an array of N indices of visible points into the original point cloud
    """
    #this is the easy but slow way
    #camera = PinholeCameraModel()
    #camera.fromCameraInfo(self.camera_info)
    # for i,p in enumerate(self.point_cloud_zed):
    #     if p[2] < 0:
    #         continue
    #     u,v = camera.project3dToPixel(p[:3])
    #     if u >= 0 and u < self.camera_info.width and v >= 0 and v < self.camera_info.height:
    #         point_cloud_image.append((u,v,i))
    #point_cloud_image = np.array(point_cloud_image)
    #image_indices = point_cloud_image[:,2].astype(int)
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
        
        self.intrinsic = intrinsic_matrix

        self.extrinsic = np.asarray(extrinsic)

    def rate(self):
        return 4.0
    
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
        pass
    
    # image call back before
    # def image_callback(self, image : cv2.Mat):
    #     detection_result = self.detector(image)
    #     # cv2.imshow("pppp",image)
    #     results = detection_result[0]
    #     # print(results)
    #     self.last_person_boxes = []
    #     for result in results:
    #         for b in result:
    #             if b.boxes.cls.item()<0.1:
    #                 print(b.boxes.xywh.tolist()[0])
    #                 self.last_person_boxes.append(b.boxes.xywh.tolist()[0]) 
           
    #    #uncomment if you want to debug the detector...
    #     for bb in self.last_person_boxes:
    #         x,y,w,h = bb
    #         cv2.rectangle(image, (int(x-w/2), int(y-h/2)), (int(x+w/2), int(y+h/2)), (255, 0, 255), 3)
    #     cv2.imwrite("pedestrian_detections.png",image)


    def image_callback(self, image : cv2.Mat):
        self.zed_image = image

    def camera_info_callback(self, info : CameraInfo):
        self.camera_info = info

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
        # print("Detection time",t2-t1,", shape estimation and tracking time",t3-t2)
        
        self.last_agent_states = current_agent_states
        return current_agent_states

    def pointcloud2_zed(self,point_cloud:np.ndarray, intrinsic: np.ndarray):
        # Convert LiDAR points to homogeneous coordinates
        homo_point_cloud_lidar = np.hstack((point_cloud, np.ones((point_cloud.shape[0], 1))))  # (N, 4)

        # Transform LiDAR points to camera frame and then to image pixel coordinates
        pointcloud_pixel = intrinsic @ self.extrinsic  @ homo_point_cloud_lidar.T  # (3, N)

        # Convert to non-homogeneous coordinates and normalize
        point_cloud_image = pointcloud_pixel[:2] / pointcloud_pixel[2]  # (2, N)

        return point_cloud_image.T  # Return as (N, 2) array

    def pointcloud2_vehicle(self,point_cloud:np.ndarray):
        # This is done to apply a 4x4 transformation matrix to the points
        homo_point_cloud_lidar = np.hstack((point_cloud, np.ones((point_cloud.shape[0], 1))))  # (N, 4)

        # This matrix converts points from LiDAR frame to vehicle frame
        pointcloud_vehicle = self.T_lidar @ homo_point_cloud_lidar.T  # (4, N)

        # Transpose the result to get the point cloud back to shape (N, 4)
        pointcloud_vehicle = pointcloud_vehicle.T  # (N, 4)

        # Return only the first three columns (x, y, z) of the transformed point cloud, discarding the homogeneous coordinate
        return pointcloud_vehicle[:, :3]  # (N, 3)

# code used before
# def box_to_fake_agent(box):
#     """Creates a fake agent state from an (x,y,w,h) bounding box.
    
#     The location and size are pretty much meaningless since this is just giving a 2D location.
#     """
#     x,y,w,h = box
#     pose = ObjectPose(t=0,x=x+w/2,y=y+h/2,z=0,yaw=0,pitch=0,roll=0,frame=ObjectFrameEnum.CURRENT)
#     dims = (w,h,0)
#     return AgentState(pose=pose,dimensions=dims,outline=None,type=AgentEnum.PEDESTRIAN,activity=AgentActivityEnum.MOVING,velocity=(0,0,0),yaw_rate=0)

    def box_to_agent(self, box, point_cloud_image, point_cloud_image_world):
        """Creates a 3D agent state from an (x,y,w,h) bounding box.
        TODO: you need to use the image, the camera intrinsics, the lidar
        point cloud, and the calibrated camera / lidar poses to get a good
        estimate of the pedestrian's pose and dimensions.
        """
        #print("--------------------------------------------------------------------===============================================================")

        x, y, w, h = box
        xmin, xmax = x - w / 2, x + w / 2
        ymin, ymax = y - h / 2, y + h / 2

        # Filter points within the bounding box
        mask = (point_cloud_image[:, 0] >= xmin) & (point_cloud_image[:, 0] <= xmax) & \
                (point_cloud_image[:, 1] >= ymin) & (point_cloud_image[:, 1] <= ymax)

        if len(point_cloud_image_world) != len(point_cloud_image):
            pose = ObjectPose(t=0,x=x+w/2,y=y+h/2,z=0,yaw=0,pitch=0,roll=0,frame=ObjectFrameEnum.CURRENT)
            dims = (1,1,1.8)
            return AgentState(pose=pose,dimensions=dims,outline=None,type=AgentEnum.PEDESTRIAN,activity=AgentActivityEnum.MOVING,velocity=(0,0,0),yaw_rate=0)

        #print("=============len(point_cloud_image_world)",len(point_cloud_image_world))
        #print("=============len(point_cloud_image",len(point_cloud_image))

        points_in_box = point_cloud_image_world[mask]

        if len(points_in_box) == 0:
            pose = ObjectPose(t=0,x=x+w/2,y=y+h/2,z=0,yaw=0,pitch=0,roll=0,frame=ObjectFrameEnum.CURRENT)
            dims = (1,1,1.8)
            return AgentState(pose=pose,dimensions=dims,outline=None,type=AgentEnum.PEDESTRIAN,activity=AgentActivityEnum.MOVING,velocity=(0,0,0),yaw_rate=0)

        distances = np.linalg.norm(point_cloud_image - [x, y], axis=1)
        closest_point_idx = np.argmin(distances)
        closest_point= point_cloud_image_world[closest_point_idx]
        # TODO: this part may have problem
        # l = np.max(points_in_box[:, 0]) - np.min(points_in_box[:, 0])

        pose = ObjectPose(t=0,x=closest_point[0],y=closest_point[1],z=0,yaw=0,pitch=0,roll=0,frame=ObjectFrameEnum.CURRENT)

        dims = [1,1,1.8]
        return AgentState(pose=pose,dimensions=dims,outline=None,type=AgentEnum.PEDESTRIAN,activity=AgentActivityEnum.MOVING,velocity=(0,0,0),yaw_rate=0)


    def detect_agents(self):
        print("|||||||||||||||||||||||||||||||||===============================================================")

        yolo_result = self.detector(self.zed_image,verbose=False)
        self.last_person_boxes = []
        #TODO: create boxes from detection result
        #TODO: create point clouds in image frame and world frame
        for b in yolo_result[0].boxes:
            if b.cls.item()<0.1:
                self.last_person_boxes.append(b.xywh.tolist()[0])

        # lidar points are transformed into zed 
        point_cloud_image = self.pointcloud2_zed(self.point_cloud,  self.intrinsic)

        # Tansfer lidar point cloud to vehicle frame
        point_cloud_image_world = self.pointcloud2_vehicle(self.point_cloud)
        
        #conver point clouds in image frame
        detected_agents = []
        for i,b in enumerate(self.last_person_boxes):
            agent = self.box_to_agent(b, point_cloud_image, point_cloud_image_world)
            detected_agents.append(agent)
            
        return detected_agents
    
    def cal_velocity(self,cur_pedestrian,prev_pedestrian)->tuple:
        v_x = (cur_pedestrian.pose.x - prev_pedestrian.pose.x) *self.rate()
        v_y = (cur_pedestrian.pose.y - prev_pedestrian.pose.y) *self.rate()
        v_z = (cur_pedestrian.pose.z - prev_pedestrian.pose.z) *self.rate()
        return (v_x,v_y,v_z)



    def track_agents(self, vehicle : VehicleState, detected_agents):
        print("bhfhfhfhf===============================================================")
        """Given a list of detected agents, updates the state of the agents."""
        # TODO: keep track of which pedestrians were detected before using last_agent_states.
        # use these to assign their ids and estimate velocities.
        results = {}
        for cur_pedestrian in detected_agents:
            same = False
            
            for prev_pedestrian in self.last_agent_states:
                start_frame = self.last_agent_states[prev_pedestrian]
                current_frame = copy.deepcopy(cur_pedestrian)
                current_frame.pose.x += vehicle.v / self.rate()
                #if current and prev pedestrain overlaps
                if collisions.polygon_intersects_polygon_2d(current_frame.polygon_parent(), start_frame.polygon_parent()):
                    # what found in this step, they are the same person
                    results[prev_pedestrian] = cur_pedestrian
                    cur_pedestrian.velocity = self.cal_velocity(current_frame,start_frame)
                    same = True
                    break
            if not same:
                cur_pedestrian.velocity = (0,0,0)
                results['ped'+str(self.pedestrian_counter)] = cur_pedestrian
                self.pedestrian_counter += 1
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
            from collections import namedtuple
            CameraInfo = namedtuple('CameraInfo',['width','height','P'])
            #TODO: these are guessed parameters
            self.camera_info = CameraInfo(width=1280,height=720,P=[560.0,0,640.0,0,  0,560.0,360,0,  0,0,1,0])
