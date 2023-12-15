#needed to import GEMstack from top level directory
import sys
import os
sys.path.append(os.getcwd())

from GEMstack.state import PhysicalObject,ObjectPose,ObjectFrameEnum
from GEMstack.mathutils import transforms
import math
import time

def test_poses():
    print(math.degrees(transforms.heading_to_yaw(0.0)),'== 90')
    print(math.degrees(transforms.heading_to_yaw(90.0)),'== 0')
    print(transforms.yaw_to_heading(math.pi,degrees=True),'== 270')

    start_pose_abs = ObjectPose(frame=ObjectFrameEnum.ABSOLUTE_CARTESIAN,t=time.time(),x=30.0,y=20.0,yaw=0.5)
    current_pose_abs = ObjectPose(frame=ObjectFrameEnum.ABSOLUTE_CARTESIAN,t=time.time()+55.0,x=60.0,y=25.0,yaw=1.5)
    current_pose_start = ObjectPose(frame=ObjectFrameEnum.START,t=55.0,x=60.0,y=25.0,yaw=1.0)
    test_pose_start = ObjectPose(frame=ObjectFrameEnum.START,t=75.0,x=80.0,y=20.0,yaw=1.2)
    test_pose_current = ObjectPose(frame=ObjectFrameEnum.CURRENT,t=20.0,x=20.0,y=00.0,yaw=0.2)
    print(start_pose_abs.transform())
    print(current_pose_start.transform())
    print(start_pose_abs.apply((0.4,0.3)))
    print(start_pose_abs.apply((0.4,0.3,0.2)))
    test_pose_current.to_frame(ObjectFrameEnum.START,current_pose=current_pose_start,start_pose_abs=start_pose_abs)
    test_pose_current.to_frame(ObjectFrameEnum.START,current_pose=current_pose_abs,start_pose_abs=start_pose_abs)
    test_pose_current.to_frame(ObjectFrameEnum.ABSOLUTE_CARTESIAN,current_pose=current_pose_start,start_pose_abs=start_pose_abs)
    test_pose_current.to_frame(ObjectFrameEnum.ABSOLUTE_CARTESIAN,current_pose=current_pose_abs,start_pose_abs=start_pose_abs)
    test_pose_start.to_frame(ObjectFrameEnum.CURRENT,current_pose=current_pose_start,start_pose_abs=start_pose_abs)
    test_pose_start.to_frame(ObjectFrameEnum.CURRENT,current_pose=current_pose_abs,start_pose_abs=start_pose_abs)
    test_pose_start.to_frame(ObjectFrameEnum.ABSOLUTE_CARTESIAN,current_pose=current_pose_start,start_pose_abs=start_pose_abs)
    test_pose_start.to_frame(ObjectFrameEnum.ABSOLUTE_CARTESIAN,current_pose=current_pose_abs,start_pose_abs=start_pose_abs)
    print("TODO: make this more comprehensive")

if __name__=='__main__':
    test_poses()