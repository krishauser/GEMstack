from ...state import AllState,VehicleState,ObjectPose,ObjectFrameEnum,AgentState,AgentEnum,AgentActivityEnum,AgentAttributesFlag
from ...utils import settings
from ...mathutils import transforms
from ..interface.gem import GEMInterface
from ..component import Component
from ultralytics import YOLO
import cv2
try:
    from sensor_msgs.msg import CameraInfo, PointCloud2

    from image_geometry import PinholeCameraModel
    import rospy
except ImportError:
    pass
import numpy as np
from typing import Dict,Tuple, List
import time
from numpy.linalg import inv
from sklearn.cluster import DBSCAN


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

def filter_lidar_by_range(point_cloud, xrange: Tuple[float, float], yrange: Tuple[float, float], zrange: Tuple[float, float]):
    xmin, xmax = xrange
    ymin, ymax = yrange
    zmin, zmax = zrange
    idxs = np.where((point_cloud[:, 0] > xmin) & (point_cloud[:, 0] < xmax) &
                    (point_cloud[:, 1] > ymin) & (point_cloud[:, 1] < ymax) &
                    (point_cloud[:, 2] > zmin) & (point_cloud[:, 2] < zmax) )
    return point_cloud[idxs]

class WavingDetector(Component):
    """Detects and tracks pedestrians."""
    def __init__(self,vehicle_interface : GEMInterface, extrinsic=None):
        self.vehicle_interface = vehicle_interface
        self.model = YOLO('GEMstack/knowledge/detection/yolov8m-pose.pt')
        self.camera_info_sub = None
        self.camera_info = None
        self.zed_image = None
        self.point_cloud = None
        self.point_cloud_zed = None

        if extrinsic is None:
            extrinsic = np.loadtxt("GEMstack/knowledge/calibration/gem_e4_lidar2oak.txt")
        self.extrinsic = np.array(extrinsic)
        intrinsic = np.loadtxt("GEMstack/knowledge/calibration/gem_e4_intrinsic.txt")
        self.intrinsic = np.concatenate([intrinsic, np.zeros((3, 1))], axis=1)

        T_lidar2_Gem = np.loadtxt("GEMstack/knowledge/calibration/gem_e4_lidar2vehicle.txt")
        self.T_lidar2_Gem = np.asarray(T_lidar2_Gem)

        # Hardcode the roi area for agents
        self.xrange = (0, 20)
        self.yrange = (-10, 10)
        self.zrange = (-3, 1)

        # Parameters for waving detection
        self.no_for_discard=20
        self.no_for_confirm=20
        self.parallel_range=20
        self.lh_elbow_range_1=0
        self.lh_elbow_range_2=145
        self.rh_elbow_range_1=35
        self.rh_elbow_range_2=180
        self.lh_shoulder_range_1=90
        self.lh_shoulder_range_2=310
        self.rh_shoulder_range_1=90
        self.rh_shoulder_range_2=230
        self.prev_ped={}
        self.count_frame={}

    def rate(self):
        return 4.0

    def state_inputs(self):
        return ['detected_agents']

    def state_outputs(self):
        return ['detected_agents']


    def initialize(self):
        #tell the vehicle to use image_callback whenever 'front_camera' gets a reading, and it expects images of type cv2.Mat
        self.vehicle_interface.subscribe_sensor('front_camera',self.image_callback,cv2.Mat)
        #tell the vehicle to use lidar_callback whenever 'top_lidar' gets a reading, and it expects numpy arrays
        self.vehicle_interface.subscribe_sensor('top_lidar',self.lidar_callback,np.ndarray)
        #subscribe to the Zed CameraInfo topic
        self.camera_info_sub = rospy.Subscriber("/oak/rgb/camera_info", CameraInfo, self.camera_info_callback)
        self.point_cloud_sub = rospy.Subscriber("/ouster/points", PointCloud2, self.lidar_callback)

    def image_callback(self, image : cv2.Mat):
        self.zed_image = image

    def camera_info_callback(self, info : CameraInfo):
        self.camera_info = info
    
    def lidar_callback(self, point_cloud: np.ndarray):
        self.point_cloud = point_cloud

    def update(self, detected_agents : List[AgentState]):
        print("--Waving Detection", self.zed_image is None, self.point_cloud is None, self.camera_info is  None)
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
        #add flag to them
        detected_agents = self.update_waving_agents(detected_agents,pose_array, flag_array) 
        print("Update waving flag: ", detected_agents)
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
    
    def check_parallel(self, lh_elbow_angle,rh_elbow_angle):
        if lh_elbow_angle is not None and rh_elbow_angle is not None:
            neg_lh_elbow_angle=float('inf')
            neg_rh_elbow_angle=float('inf')

            if lh_elbow_angle>=360-self.parallel_range:
                neg_lh_elbow_angle=-360+lh_elbow_angle
            if rh_elbow_angle>=360-self.parallel_range:
                neg_rh_elbow_angle=-360+rh_elbow_angle
            if np.abs(lh_elbow_angle-rh_elbow_angle)<=self.parallel_range or np.abs(lh_elbow_angle-neg_rh_elbow_angle)<=self.parallel_range or np.abs(neg_lh_elbow_angle-rh_elbow_angle)<=self.parallel_range or np.abs(neg_lh_elbow_angle-neg_rh_elbow_angle)<=self.parallel_range:
                return True
        return False
###    
    def waving_detection(self,boxes,kpt,img):
        curr_ped={}
        for i in range(len(boxes)):
            lh_waving_flag=False
            rh_waving_flag=False

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
                        #cv2.putText(img, lh_el, org1, font, fontScale, color, thickness)
                        #print('lh_elbow_angle  ',lh_elbow_angle)
                    if lh_shoulder!=[0,0] and lh_elbow!=[0,0]:
                        lh_shoulder_angle=self.cal_angle(lh_shoulder,lh_elbow)
                        #print('lh_shoulder_angle  ',lh_shoulder_angle)

                        lh_sh="lh_sl "+str(int(lh_shoulder_angle))
                        #cv2.putText(img, lh_sh, org2, font, fontScale, color, thickness)
                    if lh_elbow_angle is not None and lh_shoulder_angle is not None:
                        if lh_shoulder_angle<=self.lh_shoulder_range_1 or lh_shoulder_angle>=self.lh_shoulder_range_2:
                            if self.check_parallel(lh_shoulder_angle, lh_elbow_angle) or (lh_elbow_angle>=self.lh_elbow_range_1 and lh_elbow_angle<=self.lh_elbow_range_2):
                                lh_waving_flag=True

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
                        #print('rh_elbow_angle  ',rh_elbow_angle)
                        rh_el="rh_el "+str(int(rh_elbow_angle))
                        #cv2.putText(img, rh_el, org1, font, fontScale, color, thickness)
                    if rh_shoulder!=[0,0] and rh_elbow!=[0,0]:
                        rh_shoulder_angle=self.cal_angle(rh_shoulder,rh_elbow)
                        #print('rh_shoulder_angle  ',rh_shoulder_angle)
                        rh_sh="rh_sh "+str(int(rh_shoulder_angle))
                        #cv2.putText(img, rh_sh, org2, font, fontScale, color, thickness)
                    if rh_elbow_angle is not None and rh_shoulder_angle is not None:
                        if rh_shoulder_angle>=self.rh_shoulder_range_1 and rh_shoulder_angle<=self.rh_shoulder_range_2:
                            if self.check_parallel(rh_shoulder_angle, rh_elbow_angle) or (rh_elbow_angle>=self.rh_elbow_range_1 and rh_elbow_angle<=self.rh_elbow_range_2):
                                rh_waving_flag=True
                #dd="lh "+str(lh_waving_flag)
                #cv2.putText(img, dd, org2, font, fontScale, color, thickness)
                #ph="rh "+str(rh_waving_flag)
                #cv2.putText(img, ph, org, font, fontScale, color, thickness)
                
                forearm_parallel_flag=self.check_parallel(lh_elbow_angle,rh_elbow_angle)

                if (lh_waving_flag or rh_waving_flag) and not forearm_parallel_flag:
                    #print('   case2   ')
                    #no_of_waving+=1
                    #print('---------waving--------  ')

                    curr_ped[id]=boxes[i].xyxy[0].numpy().tolist()

                    #show only waving ped
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
        
    def closest_point(self, points, target):
        distances = np.linalg.norm(points - target, axis=1)
        closest_index = np.argmin(distances)
        return closest_index

#based on agent_detection.py
    def update_waving_agents(self, detected_agents : List[AgentState], pose_array, flag_array):
        results = self.model.track(self.zed_image,verbose=False)
        boxes = results[0].boxes
        #print('boxes  ',len(boxes))
        kpt=results[0].keypoints
        curr_ped={}
        img=self.zed_image
        height, width, channels = img.shape

        # TODO: create boxes from detection result

        if len(boxes)!=0:
            curr_ped,img=self.waving_detection(boxes,kpt,img)
        cv2.imshow("Keypoints of waving ped", img)
        if cv2.waitKey(1) == ord('q'):
            break
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

        # Only keep lidar point cloud that lies in roi area for agents
        filtered_point_cloud = filter_lidar_by_range(self.point_cloud, 
                                                  self.xrange, 
                                                  self.yrange,
                                                  self.zrange)
        
        epsilon = 0.09  # Epsilon parameter for DBSCAN

        # Perform DBSCAN clustering
        min_samples = 5  # Minimum number of samples in a cluster
        dbscan = DBSCAN(eps=epsilon, min_samples=min_samples)
        clusters = dbscan.fit_predict(filtered_point_cloud)
        
        # Tansfer lidar point cloud to camera frame
        point_cloud_image = lidar_to_image(filtered_point_cloud, 
                                           self.extrinsic, 
                                           self.intrinsic)
        
        # Tansfer lidar point cloud to vehicle frame
        pc_3D = lidar_to_vehicle(filtered_point_cloud, self.T_lidar2_Gem)


        #change to waving
        pred_waving_indices = np.nonzero(flag_array)[0]
        cur_waving_indices=[]
        for i,b in enumerate(waving_ped_bb):
            box = boxes[i]
            #id = bbox_ids[i]

            x, y, w, h = box
            xmin, xmax = x - w/2, x + w/2
            ymin, ymax = y - h/2, y + h/2

            # Filter. Get the idxs of point cloud that belongs to the agent
            idxs = np.where((point_cloud_image[:, 0] > xmin) & (point_cloud_image[:, 0] < xmax) &
                            (point_cloud_image[:, 1] > ymin) & (point_cloud_image[:, 1] < ymax) )
            agent_image_pc = point_cloud_image[idxs]
            agent_pc_3D = pc_3D[idxs]
            agent_clusters = clusters[idxs]

            # Get unique elements and their counts
            unique_elements, counts = np.unique(agent_clusters, return_counts=True)
            if len(counts) == 0:
                continue
            max_freq = np.max(counts)
            label_cluster = unique_elements[counts == max_freq]
            
            # filter again
            idxs = agent_clusters == label_cluster
            agent_image_pc = agent_image_pc[idxs]
            agent_clusters = agent_clusters[idxs]
            agent_pc_3D = agent_pc_3D[idxs]

            # calulate depth average
            depth = np.mean( (agent_pc_3D[:, 0] ** 2 + agent_pc_3D[:, 1] ** 2) ** 0.5 ) # euclidean dist
            y_3d = y - (width/2)
            pose=np.array([depth,y_3d,0])
            idx=self.closest_point(pose_array, pose)
            detected_agents[idx].attributes=AgentAttributesFlag.WAVING
            cur_waving_indices.append(idx)
        
        #change from waving in the previous frame to not waving
        cur_waving_indices=np.array(cur_waving_indices)
        mask = np.isin(pred_waving_indices, cur_waving_indices)
        not_waving_indices = pred_waving_indices[~mask]
        for ind in not_waving_indices:
            detected_agents[ind].attributes=AgentAttributesFlag.DEFAULT

        return detected_agents



