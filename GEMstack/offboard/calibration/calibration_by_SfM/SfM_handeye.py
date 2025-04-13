#%%
import cv2
import open3d as o3d
import numpy as np
import pyvista as pv
from typing import Literal
from tqdm import tqdm
#%%
import os
os.chdir('/mnt/GEMstack/GEMstack/offboard/calibration')
from tools.save_cali import *
#%%
N_PAIRS = 20
STEP = 3
cam = 'rr'

#%%
def get_shape():
    return 1920,1200

def getK():
    K_path = f"/mnt/GEMstack/GEMstack/knowledge/calibration/gem_e4_{cam}_in.yaml" 
    K, distort = load_in(K_path,mode='matrix',return_distort=True)
    w,h = get_shape()
    newK, roi = cv2.getOptimalNewCameraMatrix(K, distort, (w,h), 1, (w,h))
    return newK

def get_img_np(img_path):
    K_path = f"/mnt/GEMstack/GEMstack/knowledge/calibration/gem_e4_{cam}_in.yaml" 
    K, distort = load_in(K_path,mode='matrix',return_distort=True)
    img = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)
    undistorted = cv2.undistort(img, K, distort)
    return np.asarray(undistorted)

def get_pc_np(pcd_path):
    pcd = o3d.io.read_point_cloud(pcd_path)
    pc = np.asarray(pcd.points)
    pc = pc[~np.all(pc == 0, axis=1)] # remove (0,0,0)'s
    return pc

def vis(pc):
    plotter = pv.Plotter(notebook=False)
    # plotter = pv.Plotter()
    plotter.add_mesh(
        pv.PolyData(pc),
        render_points_as_spheres=True,
        point_size=2,
        color='red'
    )
    plotter.show()
    
#%%
def pcd_paths():
    img_root = '/mnt/GEMstack/GEMstack/offboard/calibration/calibration_by_SfM/data/pc/'
    img_format = '.pcd'
    for id in range(0,N_PAIRS*STEP+1):
        path = img_root + str(id).zfill(6) + img_format
        print(path)
        yield path

def img_paths():
    img_root = f'/mnt/GEMstack/GEMstack/offboard/calibration/calibration_by_SfM/data/{cam}/images/'
    img_format = '.png'
    for id in range(0,N_PAIRS*STEP+1):
        path = img_root + str(id).zfill(6) + img_format
        print(path)
        yield path

def pcs():
    for path in pcd_paths():
        yield get_pc_np(path)

def imgs():
    for path in img_paths():
        yield get_img_np(path)

def iterpair(iter,step):
    pre = None
    for i in iter:
        pre = i
        break
    for i,v in enumerate(iter):
        if i%step == step-1:
            yield pre,v
            pre = v

#%%
def findE_img(img1,img2,K):
    sift = cv2.SIFT.create()
    kp1,des1 = sift.detectAndCompute(img1,None)
    kp2,des2 = sift.detectAndCompute(img2,None)

    if des1 is None or des2 is None or len(des1) < 5 or len(des2) < 5:
        return None #insufficient features

    FLANN_INDEX_KDTREE = 1
    index_params = dict(algorithm=FLANN_INDEX_KDTREE, trees=5)
    search_params = dict(checks=50)
    flann = cv2.FlannBasedMatcher(index_params, search_params)
    matches = flann.knnMatch(des1, des2, k=2)
    good_matches = [m for m,n in matches if m.distance < 0.8 * n.distance]

    pts1 = np.float32([kp1[m.queryIdx].pt for m in good_matches]).reshape(-1, 2)
    pts2 = np.float32([kp2[m.trainIdx].pt for m in good_matches]).reshape(-1, 2)

    E, mask = cv2.findEssentialMat(pts1, pts2, K, method=cv2.RANSAC, prob=0.999, threshold=1.0)
    if E is None or mask.sum() < 5:
        return None #Essential matrix computation failed
    _, R_rel, t_rel, mask = cv2.recoverPose(E, pts1, pts2, K, mask=mask)
    ret = np.eye(4)
    ret[:3, :3] = R_rel
    ret[:3, 3] = t_rel.flatten()
    return ret

def findE_pc(pc1,pc2,mode:Literal['icp','kp','svd']):
    if mode == 'icp':
        source = o3d.geometry.PointCloud()
        source.points = o3d.utility.Vector3dVector(pc1)
        
        target = o3d.geometry.PointCloud()
        target.points = o3d.utility.Vector3dVector(pc2)
        
        source.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30)
        )
        target.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30)
        )
        init_pose = np.eye(4) #assume continuous movement
        # Robust configuration
        reg = o3d.pipelines.registration.registration_icp(
            source, target,
            0.1,
            init_pose,
            o3d.pipelines.registration.TransformationEstimationPointToPoint(),
            o3d.pipelines.registration.ICPConvergenceCriteria(
                max_iteration=30,
                relative_rmse=1e-6
            )
        )
        return reg.transformation
    else:
        print(mode,'no imp')
        return None
        
#%%
rcam, tcam = [],[]
rlidar, tlidar = [],[]
#%%
for i,((img1,img2),(pc1,pc2)) in tqdm(enumerate(zip(
    iterpair(map(get_img_np,img_paths()),STEP), 
    iterpair(map(get_pc_np,pcd_paths()),STEP))),total=N_PAIRS):
    K = getK()
    E_cam = findE_img(img1,img2,K)
    print(E_cam)
    E_lidar = findE_pc(pc1,pc2,mode='icp')
    print(E_lidar)
    if E_cam is None or E_lidar is None:
        continue #unsolved
    if np.isnan(E_cam).any() or np.isnan(E_lidar).any():
        continue #has nan
    if np.linalg.norm(E_lidar[:3,3]) > 0.1*STEP:
        continue #too large to be possible
    rcam.append(E_cam[:3, :3])
    tcam.append(E_cam[:3, 3])
    rlidar.append(E_lidar[:3, :3])
    tlidar.append(E_lidar[:3, 3])

#%%
rt2cam = []
tt2cam = []
for r,t in zip(rlidar,tlidar):
    B = np.eye(4)
    B[:3,:3] = r
    B[:3,3] = t
    B = np.linalg.inv(B)
    rt2cam.append(B[:3,:3])
    tt2cam.append(B[:3,3])

X = cv2.calibrateHandEye(
    R_gripper2base=rcam, t_gripper2base=tcam,
    R_target2cam=rt2cam, t_target2cam=tt2cam,
    # R_target2cam=rlidar, t_target2cam=tlidar,
    # method=cv2.CALIB_HAND_EYE_TSAI  
    method=cv2.CALIB_HAND_EYE_PARK
)
#%%
# Result is 4x4 transformation matrix: T_cam_lidar
T_cam_lidar = np.eye(4)
T_cam_lidar[:3, :3] = X[0]
T_cam_lidar[:3, 3] = X[1].flatten() 
