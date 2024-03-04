#needed to import GEMstack from top level directory
import sys
import os
sys.path.append(os.getcwd())

from GEMstack.state import AgentState, Route, SceneState, Path, ObjectFrameEnum
from GEMstack.onboard.perception.pedestrian_detection import PedestrianDetector
#from GEMstack.onboard.perception.pedestrian_detection import AGENT_TYPE_TO_TAG
from klampt import vis
from klampt.math import vectorops,se3
from klampt import Geometry3D
from klampt.model import create
from klampt.vis import colorize
from klampt.io import numpy_convert
import numpy as np

data_folder = 'testing/pedestrian_detection/'

def box_to_outline(P,x,y,w,h):
    frame = ((x-P[0,2])/P[0,0],(y-P[1,2])/P[1,1],(x+w-P[0,2])/P[0,0],(y+h-P[1,2])/P[1,1])
    return [[frame[0],frame[1],1],[frame[2],frame[1],1],[frame[2],frame[3],1],[frame[0],frame[3],1],[frame[0],frame[1],1]]

def test_pedestrian_detection():
    detector = PedestrianDetector(None)
    detector.load_data(data_folder)
    pc_img,pc_img_world = detector.point_cloud_image()
    P = np.array(detector.camera_info.P).reshape(3,4)
    pc_img[:,0] -= P[0,2]
    pc_img[:,1] -= P[1,2]
    pc_img[:,0] /= P[0,0]
    pc_img[:,1] /= P[1,1]
    T_zed = se3.from_ndarray(detector.T_zed)
    T_velodyne = se3.from_ndarray(detector.T_lidar)
    pc_img = np.hstack((pc_img, np.full((pc_img.shape[0],1),1.0)))
    h,w = detector.zed_image.shape[:2]
    vis.add('zed_image_plane',[se3.apply(T_zed,x) for x in box_to_outline(P,0,0,w,h)],color=(0,0,0,1))
    pc_img_world = pc_img_world[:,:3]
    pc_img_klampt = Geometry3D(numpy_convert.from_numpy(pc_img,'PointCloud'))
    pc_img_world_klampt = Geometry3D(numpy_convert.from_numpy(pc_img_world,'PointCloud'))
    pc_img_klampt.setCurrentTransform(*T_zed)
    vis.add('zed_frame',T_zed,fancy=True)
    vis.add('velodyne_frame',T_velodyne,fancy=True)
    vis.add('pc_img',pc_img_klampt,color=(1,0,0,0.5))
    vis.add('pc_img_world',pc_img_world_klampt,color=(0,1,0,0.5))
    agents = detector.detect_agents()
    for i,a in enumerate(agents):
        center = a.pose.x,a.pose.y,a.pose.z + a.dimensions[2]/2
        dims = a.dimensions
        b = create.box(width=dims[0],depth=dims[1],height=dims[2],R=se3.identity()[0],t=center,name='agent'+str(i))
        vis.add('ped '+str(i),b,color=(0,1,5,0.25))
        #vis.add(AGENT_TYPE_TO_TAG[a.type]+' '+str(i),b,color=(0,1,5,0.25))
    for i,b in enumerate(detector.last_person_boxes):
        x,y,w,h = b[:4]
        vis.add('detection '+str(i),[se3.apply(T_zed,p) for p in box_to_outline(P,x-w/2,y-h/2,w,h)],color=(0,0,1,1))
        vis.add('detection. '+str(i),[se3.apply(T_zed,vectorops.mul(p,1.9)) for p in box_to_outline(P,x-w/2,y-h/2,w,h)],color=(0,0,1,1))
        vis.add('detection.. '+str(i),[se3.apply(T_zed,vectorops.mul(p,2.95)) for p in box_to_outline(P,x-w/2,y-h/2,w,h)],color=(0,0,1,1))
        vis.add('detection... '+str(i),[se3.apply(T_zed,vectorops.mul(p,3.8)) for p in box_to_outline(P,x-w/2,y-h/2,w,h)],color=(0,0,1,1))
    vis.autoFitCamera()
    vis.loop()
    pass

if __name__ == '__main__':
    test_pedestrian_detection()
