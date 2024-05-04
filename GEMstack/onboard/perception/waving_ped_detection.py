from ...state import AllState,VehicleState,ObjectPose,ObjectFrameEnum,AgentState,AgentEnum,AgentActivityEnum,AgentAttributesFlag
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
from typing import Dict,Tuple, List
import time
from numpy.linalg import inv


def Pmatrix(fx,fy,cx,cy):
    """Returns a projection matrix for a given set of camera intrinsics."""
    return np.array([[fx,0,cx,0],
                     [0,fy,cy,0],
                     [0,0,1,0]])

def project_point_cloud(point_cloud : np.ndarray, P : np.ndarray, xrange : Tuple[int,int], yrange : Tuple[int,int]) -> Tuple[np.ndarray,np.ndarray]:
    """Projects a point cloud into a 2D image using a camera intrinsic projection matrix P.
    
    Returns:
        - point_cloud_image: an Nx2 array of (u,v) visible image coordinates
        - image_indices: an array of N indices of visible points into the original point cloud
    """

    pc_with_ids = np.hstack((point_cloud,np.arange(len(point_cloud)).reshape(-1,1)))
    pc_fwd = pc_with_ids[pc_with_ids[:,2] > 0]
    pxform = pc_fwd[:,:3].dot(P[:3,:3].T) + P[:3,3]
    uv = (pxform[:,0:2].T/pxform[:,2]).T
    inds = np.logical_and(np.logical_and(uv[:,0] >= xrange[0],uv[:,0] < xrange[1]),
                    np.logical_and(uv[:,1] >= yrange[0],uv[:,1] < yrange[1]))
    point_cloud_image = uv[inds]
    image_indices = pc_fwd[inds,3].astype(int)
    return point_cloud_image, image_indices

def lidar_to_image(point_cloud_lidar: np.ndarray, extrinsic : np.ndarray, intrinsic : np.ndarray):
    
    homo_point_cloud_lidar = np.hstack((point_cloud_lidar, np.ones((point_cloud_lidar.shape[0], 1)))) # (N, 4)
    pointcloud_pixel = (intrinsic @ extrinsic @ (homo_point_cloud_lidar).T) # (3, N)
    pointcloud_pixel = pointcloud_pixel.T # (N, 3)

    # normalize
    pointcloud_pixel[:, 0] /= pointcloud_pixel[:, 2] # normalize
    pointcloud_pixel[:, 1] /= pointcloud_pixel[:, 2] # normalize
    point_cloud_image =  pointcloud_pixel[:,:2] # (N, 2)
    return point_cloud_image

def lidar_to_vehicle(point_cloud_lidar: np.ndarray, T_lidar2_Gem: np.ndarray):
    ones = np.ones((point_cloud_lidar.shape[0], 1))
    pcd_homogeneous = np.hstack((point_cloud_lidar, ones)) # (N, 4)
    pointcloud_trans = np.dot(T_lidar2_Gem, pcd_homogeneous.T) # (4, N)
    pointcloud_trans = pointcloud_trans.T # (N, 4)
    point_cloud_image_world = pointcloud_trans[:, :3] # (N, 3)
    return point_cloud_image_world

def filter_lidar_by_range(point_cloud, xrange: Tuple[float, float], yrange: Tuple[float, float]):
    xmin, xmax = xrange
    ymin, ymax = yrange
    idxs = np.where((point_cloud[:, 0] > xmin) & (point_cloud[:, 0] < xmax) &
                    (point_cloud[:, 1] > ymin) & (point_cloud[:, 1] < ymax) )
    return point_cloud[idxs]

class WavingDetector(Component):
    """Detects and tracks pedestrians."""
    def __init__(self,vehicle_interface : GEMInterface, extrinsic=None):
        # State Estimation
        self.vehicle_interface = vehicle_interface

        #self.detector = YOLO('GEMstack/knowledge/detection/yolov8n.pt')
        self.model = YOLO('GEMstack/knowledge/detection/yolov8m-pose.pt')

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

        # Hardcode the roi area for agents
        self.xrange = (1, 20)
        self.yrange = (-10, 10)

###
        self.no_for_discard=20
        self.no_for_confirm=20
        self.prev_ped={}
        self.count_frame={}

    def rate(self):
        return 2.5

    def state_inputs(self):
        return ['vehicle','detected_agents']

    def state_outputs(self):
        return ['detected_agents']


    # def initialize(self):
    #     print("INITIALIZE PED DETECT")
    #     # tell the vehicle to use image_callback whenever 'front_camera' gets a reading, and it expects images of type cv2.Mat
    #     self.vehicle_interface.subscribe_sensor('front_camera',self.image_callback,cv2.Mat)
    #     # tell the vehicle to use lidar_callback whenever 'top_lidar' gets a reading, and it expects numpy arrays
    #     self.vehicle_interface.subscribe_sensor('top_lidar',self.lidar_callback,np.ndarray)
    #     # subscribe to the Zed CameraInfo topic
    #     self.camera_info_sub = rospy.Subscriber("/oak/rgb/camera_info", CameraInfo, self.camera_info_callback)

    # def image_callback(self, image : cv2.Mat):
    #     self.zed_image = image

    # def camera_info_callback(self, info : CameraInfo):
    #     self.camera_info = info

    # def lidar_callback(self, point_cloud: np.ndarray):
    #     self.point_cloud = point_cloud

    def update(self, vehicle : VehicleState, detected_agents : List[AgentState]):
        print("0PED DETECT UPDATE", self.zed_image is None, self.point_cloud is None, self.camera_info is  None)
        if self.zed_image is None:
            # no image data yet
            print("zed")
            return {}
        if self.point_cloud is None:
            # no lidar data yet
            print("pc")
            return {}
        if self.camera_info is None:
            # no camera info yet
            print("camera")
            return {}
        pose_array, flag_array=self.form_agent_info(detected_agents)
        print("Updating with detected, ", detected_agents)
        #add flag to them
        detected_agents = self.update_waving_agents(detected_agents,pose_array, flag_array) 
        print("Pedestrian Detection detected agents: ", detected_agents)
        return detected_agents
###

    def form_agent_info(self, detected_agents):
        pose_array=[]
        flag_array=[]
        for agent in detected_agents:
            pose=[agent.pose.x,agent.pose.y,agent.pose.z]
            pose_array.append(pose)
            flag=agent.attributes.value
            flag_array.append(flag)
        pose_array=np.array(pose_array)
        flag_array=np.array(flag_array)
        return pose_array, flag_array

    def cal_angle(self,elbow,wrist):
        x1=elbow[0]
        y1=elbow[1]
        x2=wrist[0]
        y2=wrist[1]

        diff_x=x2-x1
        diff_y=-(y2-y1)

        angle_deg=180*np.arctan2(diff_y,diff_x)/np.pi
        if angle_deg<0:
            angle_deg=360+angle_deg 
        #print(angle_deg)
        return angle_deg
###    
    def waving_detection(self,boxes,kpt,img):
        curr_ped={}
        for i in range(len(boxes)):
            lh_waving_flag=False
            rh_waving_flag=False
            waving_flag=False

            lh_using_phone=False  
            rh_using_phone=False

            if boxes[i].id is None:
                break
            id=int(boxes[i].id.item())
            kpt_i=kpt[i].xy.cpu().numpy()[0].astype(int).tolist()
            #print('keypoints  ',kpt_i)
            nose=kpt_i[0]
            lh_eye=kpt_i[1]
            rh_eye=kpt_i[2]
            lh_ear=kpt_i[3]
            rh_ear=kpt_i[4]
            lh_shoulder=kpt_i[5]
            rh_shoulder=kpt_i[6]
            lh_elbow=kpt_i[7]
            lh_wrist=kpt_i[9]
            rh_elbow=kpt_i[8]
            rh_wrist=kpt_i[10]
            org = [100, 100]
            org1 = [50, 50]
            org2 = [400, 50]
            #print(lh_angle)

            
            font = cv2.FONT_HERSHEY_SIMPLEX
            fontScale = 1
            color = (255, 0, 0)
            thickness = 2
            lh_elbow_angle=None
            rh_elbow_angle=None
            lh_shoulder_angle=None
            rh_shoulder_angle=None
            #value=[0,0] if that keypoint is not detected       
            #point1 = np.array(point1)
            #point2 = np.array(point2)
            #return np.linalg.norm(point2 - point1)

            if not (nose==[0,0] and lh_eye==[0,0] and rh_eye==[0,0]):
                #ped turn backwards
                #check using phone
                if lh_wrist!=[0,0] and lh_ear!=[0,0] and lh_eye!=[0,0] and rh_eye!=[0,0]:
                    dist=np.linalg.norm(np.array(lh_wrist) - np.array(lh_ear))
                    eye_ref=np.linalg.norm(np.array(lh_eye) - np.array(rh_eye))
                    if dist<=eye_ref*2.5:
                        lh_using_phone=True  
                    # dd="dis "+str(int(dist))
                    # cv2.putText(img, dd, org1, font, fontScale, color, thickness)
                    # ph="eye_ref "+str(int(eye_ref))+str(lh_using_phone)
                    # cv2.putText(img, ph, org2, font, fontScale, color, thickness)
                
                if not lh_using_phone:
                    if lh_elbow!=[0,0] and lh_wrist!=[0,0]:
                        lh_elbow_angle=self.cal_angle(lh_elbow,lh_wrist)
                        lh_el="lh_el "+str(int(lh_elbow_angle))
                        cv2.putText(img, lh_el, org1, font, fontScale, color, thickness)
                        #print('lh_elbow_angle  ',lh_elbow_angle)
                    if lh_shoulder!=[0,0] and lh_elbow!=[0,0]:
                        lh_shoulder_angle=self.cal_angle(lh_shoulder,lh_elbow)
                        #print('lh_shoulder_angle  ',lh_shoulder_angle)

                        lh_sh="lh_sl "+str(int(lh_shoulder_angle))
                        cv2.putText(img, lh_sh, org2, font, fontScale, color, thickness)
                    if lh_elbow_angle is not None and lh_shoulder_angle is not None:
                        if lh_shoulder_angle<=90 or lh_shoulder_angle>=310:
                            if np.abs(lh_shoulder_angle-lh_elbow_angle)<=20 or (lh_elbow_angle>=0 and lh_elbow_angle<=145):
                                lh_waving_flag=True
            ####
                if rh_wrist!=[0,0] and rh_ear!=[0,0] and lh_eye!=[0,0] and rh_eye!=[0,0]:
                    dist=np.linalg.norm(np.array(rh_wrist) - np.array(rh_ear))
                    eye_ref=np.linalg.norm(np.array(lh_eye) - np.array(rh_eye))
                    if dist<=eye_ref*2.5:
                        rh_using_phone=True  
                    # dd="dis "+str(int(dist))
                    # cv2.putText(img, dd, org1, font, fontScale, color, thickness)
                    # ph="eye_ref "+str(int(eye_ref))+str(rh_using_phone)
                    # cv2.putText(img, ph, org2, font, fontScale, color, thickness)
                
                if not rh_using_phone:
                    if rh_elbow!=[0,0] and rh_wrist!=[0,0]:
                        rh_elbow_angle=self.cal_angle(rh_elbow,rh_wrist)
                        print('rh_elbow_angle  ',rh_elbow_angle)
                        rh_el="rh_el "+str(int(rh_elbow_angle))
                        #cv2.putText(img, rh_el, org1, font, fontScale, color, thickness)
                    if rh_shoulder!=[0,0] and rh_elbow!=[0,0]:
                        rh_shoulder_angle=self.cal_angle(rh_shoulder,rh_elbow)
                        print('rh_shoulder_angle  ',rh_shoulder_angle)
                        rh_sh="rh_sh "+str(int(rh_shoulder_angle))
                        #cv2.putText(img, rh_sh, org2, font, fontScale, color, thickness)
                    if rh_elbow_angle is not None and rh_shoulder_angle is not None:
                        if rh_shoulder_angle>=90 and rh_shoulder_angle<=230:
                            if np.abs(rh_shoulder_angle-rh_elbow_angle)<=20 or (rh_elbow_angle>=35 and rh_elbow_angle<=180):
                                rh_waving_flag=True
                dd="lh "+str(lh_waving_flag)
                cv2.putText(img, dd, org2, font, fontScale, color, thickness)
                ph="rh "+str(rh_waving_flag)
                cv2.putText(img, ph, org, font, fontScale, color, thickness)
                
                if lh_elbow_angle is not None and rh_elbow_angle is not None and np.abs(lh_elbow_angle-rh_elbow_angle)<=20:
                    # ="lh "+str(int(np.abs(lh_elbow_angle-rh_elbow_angle)))dd
                    # cv2.putText(img, dd, org1, font, fontScale, color, thickness)
                    # ph="rh "+str(rh_waving_flag)
                    # cv2.putText(img, ph, org2, font, fontScale, color, thickness)
                    print('   case1   ')
                    waving_flag=False
                elif lh_waving_flag or rh_waving_flag:
                    waving_flag=True
                    print('   case2   ')
                    #no_of_waving+=1
                    print('---------waving--------  ')

                    curr_ped[id]=boxes[i].xyxy[0].numpy().tolist()
                else: 
                    print('   case3   ')    
                
            if kpt_i[5][0]!=0 and kpt_i[7][0]!=0:
                cv2.line(img, (kpt_i[5][0],kpt_i[5][1]), (kpt_i[7][0],kpt_i[7][1]), (0, 255, 0), 2)
            if kpt_i[6][0]!=0 and kpt_i[8][0]!=0:
                cv2.line(img, (kpt_i[6][0],kpt_i[6][1]), (kpt_i[8][0],kpt_i[8][1]), (0, 255, 0), 2)
            if kpt_i[7][0]!=0 and kpt_i[9][0]!=0:
                cv2.line(img, (kpt_i[7][0],kpt_i[7][1]), (kpt_i[9][0],kpt_i[9][1]), (0, 255, 0), 2)
            if kpt_i[8][0]!=0 and kpt_i[10][0]!=0:
                cv2.line(img, (kpt_i[8][0],kpt_i[8][1]), (kpt_i[10][0],kpt_i[10][1]), (0, 255, 0), 2)

            cv2.circle(img, (kpt_i[0][0], kpt_i[0][1]), radius=3, color=(0, 0, 255), thickness=-1)
            cv2.circle(img, (kpt_i[1][0], kpt_i[1][1]), radius=3, color=(0, 255, 0), thickness=-1)
            cv2.circle(img, (kpt_i[2][0], kpt_i[2][1]), radius=3, color=(0, 255, 0), thickness=-1)
            cv2.circle(img, (kpt_i[3][0], kpt_i[3][1]), radius=3, color=(255, 0, 0), thickness=-1)
            cv2.circle(img, (kpt_i[4][0], kpt_i[4][1]), radius=3, color=(255, 0, 0), thickness=-1)
        return curr_ped,img

    def box_to_pose(self, box, point_cloud_image, point_cloud_image_world):
        """Creates a 3D agent state from an (x,y,w,h) bounding box.
        """

        # print ('Detect a pedestrian!')

        # get the idxs of point cloud that belongs to the agent
        x,y,w,h = box
        xmin, xmax = x - w/2, x + w/2
        ymin, ymax = y - h/2, y + h/2


        # enlarge bbox in case inaccuracy calibration
        enlarge_factor = 3
        xmin *= enlarge_factor
        xmax *= enlarge_factor
        ymin *= enlarge_factor
        ymax *= enlarge_factor

        agent_world_pc = point_cloud_image_world


        # Find the point_cloud that is closest to the center of our bounding box
        center_x = x + w / 2
        center_y = y + h / 2
        distances = np.linalg.norm(point_cloud_image - [center_x, center_y], axis=1)
        closest_point_cloud_idx = np.argmin(distances)
        closest_point_cloud = point_cloud_image_world[closest_point_cloud_idx]

        x, y, _ = closest_point_cloud
        #pose = ObjectPose(t=0, x=x, y=y, z=0, yaw=0, pitch=0, roll=0, frame=ObjectFrameEnum.CURRENT)

        # Specify AgentState.
        # l = np.max(agent_world_pc[:, 0]) - np.min(agent_world_pc[:, 0])
        # w = np.max(agent_world_pc[:, 1]) - np.min(agent_world_pc[:, 1])
        # h = np.max(agent_world_pc[:, 2]) - np.min(agent_world_pc[:, 2])
        
        # dims = (w, h, l) 
        pose=np.array([x,y,0])
        return pose
        #return AgentState(pose=pose,dimensions=dims,outline=None,type=AgentEnum.PEDESTRIAN,activity=AgentActivityEnum.MOVING,velocity=(0,0,0),yaw_rate=0,attributes=AgentAttributesFlag.WAVING)
    def closest_point(self, points, target):
        distances = np.linalg.norm(points - target, axis=1)
        closest_index = np.argmin(distances)
        return closest_index

    # Behavior Prediction added test argument to return matchings
    def update_waving_agents(self, detected_agents : List[AgentState], pose_array, flag_array):
        results = self.model.track(self.zed_image,verbose=False)
        boxes = results[0].boxes
        #print('boxes  ',len(boxes))
        kpt=results[0].keypoints
        curr_ped={}

        # TODO: create boxes from detection result

        if len(boxes)!=0:
            curr_ped,img=self.waving_detection(boxes,kpt,self.zed_image)

        dict_no_keys = set(self.count_frame.keys()) - set(curr_ped.keys())    
        for idx in dict_no_keys:
            self.count_frame[idx][1]-=1

        for ped in curr_ped:
            if ped in self.prev_ped:
                self.count_frame[ped][0]+=1
                self.count_frame[ped][1]=self.no_for_discard
                self.count_frame[ped][2]=curr_ped[ped]
            else:
                self.count_frame[ped]=[1,self.no_for_discard,curr_ped[ped]]
        print('curr_ped  ',curr_ped)
        print('count_frame  ',self.count_frame)


        self.count_frame = {key: value for key, value in self.count_frame.items() if value[1] != 0}
        waving_dict = {key: value for key, value in self.count_frame.items() if value[0] > self.no_for_confirm}
        waving_ped_key = list(waving_dict.keys())
        waving_ped_bb = [value[2] for value in waving_dict.values()]

        self.prev_ped=curr_ped

#if in the frame but not wave??

        # Only keep lidar point cloud that lies in roi area for agents
        point_cloud_lidar = filter_lidar_by_range(self.point_cloud, self.xrange, self.yrange)

        # Tansfer lidar point cloud to camera frame
        point_cloud_image = lidar_to_image(point_cloud_lidar, self.extrinsic, self.intrinsic)

        # Tansfer lidar point cloud to vehicle frame
        point_cloud_image_world = lidar_to_vehicle(point_cloud_lidar, self.T_lidar2_Gem)
        
        pred_waving_indices = np.nonzero(flag_array)[0]
        idx_array=[]
        #change to waving
        for i,b in enumerate(waving_ped_bb):
            pose = self.box_to_pose(b, point_cloud_image, point_cloud_image_world)
            idx=self.closest_point(pose_array, pose)
            detected_agents[idx].attributes=AgentAttributesFlag.WAVING
            idx_array.append(idx)
        #change from waving in the previous frame to not waving
        idx_array=np.array(idx_array)
        mask = np.isin(pred_waving_indices, idx_array)
        not_waving_indices = pred_waving_indices[~mask]
        for ind in not_waving_indices:
            detected_agents[ind].attributes=AgentAttributesFlag.DEFAULT

        return detected_agents



