from dataclasses import replace
import math
from typing import List
from ...utils import settings
from ...mathutils import transforms
from ...state.vehicle import VehicleState,VehicleGearEnum
from ...state.physical_object import ObjectFrameEnum,ObjectPose,convert_xyhead
from ...knowledge.vehicle.geometry import front2steer,steer2front
from ...mathutils.signal import OnlineLowPassFilter
from ..interface.gem import GEMInterface
from ..component import Component
from ..interface.gem import GNSSReading

import numpy as np
import open3d as o3d
import copy
import utm
import time
import argparse
import os
import glob
from scipy.spatial.transform import Rotation as R
from .gnss_kalman_filter import GNSSKalmanFilter
import rospy
from septentrio_gnss_driver.msg import INSNavGeod

class KFStateEstimator(Component):
    def __init__(self,vehicle_interface : GEMInterface):
        self.vehicle_interface = vehicle_interface
        self.filter = GNSSKalmanFilter()
        vehicle_interface.subscribe_sensor('gnss',self.gnss_callback,GNSSReading)
        _ = rospy.Subscriber("/map_estimator/navsatfix", INSNavGeod, self.map_based_estimation_callback)
        
    # Get GNSS information
    def gnss_callback(self, reading : GNSSReading):
        self.filter.update(rospy.get_time(),reading.pose,reading.speed,0.98)

    def map_based_estimation_callback(self, msg : INSNavGeod):
        self.pose = ObjectPose(ObjectFrameEnum.GLOBAL,
                    t=rospy.get_time(),
                    x=math.degrees(msg.longitude),   #Septentrio GNSS uses radians rather than degrees
                    y=math.degrees(msg.latitude),
                    z=msg.height,
                    yaw=math.radians(msg.heading),  #heading from north in degrees (TODO: maybe?? check this)
                    roll=math.radians(msg.roll),
                    pitch=math.radians(msg.pitch),
                    )
        self.speed = np.sqrt(msg.ve**2 + msg.vn**2)
        self.new_update = True

    def rate(self):
        return 1.0
    
    def state_outputs(self) -> List[str]:
        return ['vehicle']

    def healthy(self):
        return self.filter.is_initialized

    def update(self) -> VehicleState:
        new_pose = None
        new_speed = None
        if self.new_update:
            new_pose, new_speed = self.filter.update(rospy.get_time(),self.pose,self.speed,0.93)
            self.new_update = False
        else:
            new_pose, new_speed = self.filter.update(rospy.get_time())
        readings = self.vehicle_interface.get_reading()
        raw = readings.to_state(new_pose)
        raw.v = new_speed

        print(new_pose.x, new_pose.y, new_pose.z, new_pose.yaw)
        return raw
            