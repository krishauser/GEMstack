#needed to import GEMstack from top level directory
import sys
import os
sys.path.append(os.getcwd())

from GEMstack.state import PhysicalObject,ObjectPose,ObjectFrameEnum
from GEMstack.state.physical_object import _get_frame_chain
from GEMstack.mathutils import transforms
import math
import time

def test_poses():
    print(math.degrees(transforms.heading_to_yaw(0.0)),'== 90')
    print(math.degrees(transforms.heading_to_yaw(90.0)),'== 0')
    print(transforms.yaw_to_heading(math.pi,degrees=True),'== 270')

    #google maps gives latitude,longitude, ugh...
    #ObjectPose yaw expects radians, heading CW from north
    start_pose_global = ObjectPose(frame=ObjectFrameEnum.GLOBAL,t=time.time(),y=40.09286250064475,x=-88.23565755734872,yaw=math.radians(90.0))
    current_pose_global = ObjectPose(frame=ObjectFrameEnum.GLOBAL,t=time.time()+55.0,y=40.09287891579668, x=-88.23588822731645,yaw=math.radians(90.0))
    current_pose_start = current_pose_global.to_frame(ObjectFrameEnum.START,start_pose_abs=start_pose_global)
    print(current_pose_start.x,current_pose_start.y,current_pose_start.yaw,"(should be <0, ~=0, ~=0)")

    start_pose_global = ObjectPose(frame=ObjectFrameEnum.GLOBAL,t=time.time(),y=40.09286250064475,x=-88.23565755734872,yaw=math.radians(270.0))
    current_pose_global = ObjectPose(frame=ObjectFrameEnum.GLOBAL,t=time.time()+55.0,y=40.09287891579668, x=-88.23588822731645,yaw=math.radians(270.0))
    current_pose_start = current_pose_global.to_frame(ObjectFrameEnum.START,start_pose_abs=start_pose_global)
    # frame_chain = _get_frame_chain(ObjectFrameEnum.GLOBAL,ObjectFrameEnum.START,current_pose_global,start_pose_abs=start_pose_global)
    # xyhead = (current_pose_global.x,current_pose_global.y,current_pose_global.yaw)
    # print(xyhead)
    # for (frame,pose,dir) in frame_chain[1:]:
    #     print("Frame change:",frame,pose,dir)
    #     if dir == 1:
    #         xyhead = pose.apply_xyhead(xyhead)
    #     else:
    #         xyhead = pose.apply_inv_xyhead(xyhead)
    #     print(xyhead)
    print(current_pose_start.x,current_pose_start.y,current_pose_start.yaw,"(should be >0, ~=0, ~=0)")

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