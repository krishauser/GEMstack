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
from ..interface.gem_hardware import GNSSReading
import numpy as np
from ekf import ESKF 


class EKFStateEstimator(Component):
    """Use IMU and GNSS reading to jointly estimate the vehicle state"""
    def __init__(self, vehicle_interface : GEMInterface):
        self.vehicle_interface = vehicle_interface
        if 'gnss' not in vehicle_interface.sensors() or 'imu' not in vehicle_interface.sensors():
            raise RuntimeError("Septentrio INS receiver not available")
        vehicle_interface.subscribe_sensor('gnss',self.gnss_callback,GNSSReading)
        vehicle_interface.subscribe_sensor('imu',self.imu_callback)
        self.gnss_pose = None
        self.location = settings.get('vehicle.calibration.gnss_location')[:2]
        self.yaw_offset = settings.get('vehicle.calibration.gnss_yaw')
        self.speed_filter  = OnlineLowPassFilter(1.2, 30, 4)
        self.status = None
        self.long_ref, self.lat_ref = -1.540009687915658, 0.6997620706897224 #a location outside of highbay as reference point 


        #initialize ESKF estimator
        self.estimator = ESKF(initial_t = self.vehicle_interface.time()) #set initial time


    # Get GNSS information
    def gnss_callback(self, reading : GNSSReading):
        self.gnss_pose = reading.pose
        self.gnss_speed = reading.speed
        self.status = reading.status

    #IMU callback handler    
    def imu_callback(self, reading):
        self.imu_reading = reading
        self.ax = reading.linear_acceleration.x
        self.ay = reading.linear_acceleration.y
        self.az = reading.linear_acceleration.z
        self.wx = reading.angular_velocity.x
        self.wy = reading.angular_velocity.y
        self.wz = reading.angular_velocity.z
    
    def rate(self):
        return 10.0
    
    def state_outputs(self) -> List[str]:
        return ['vehicle']

    def healthy(self):
        return self.gnss_pose is not None

    def update(self) -> VehicleState:
        if self.gnss_pose is None:
            return

        # vehicle gnss heading not working on Gem e4 vehicle
        # vehicle x, y position in radians, global frame, see gem_hardware.py
        # callback_with_gnss_reading sets to global frame
        localxy = transforms.rotate2d(self.location,-self.yaw_offset)
        gnss_xyhead_inv = (-localxy[0],-localxy[1],-self.yaw_offset)
        center_xyhead = self.gnss_pose.apply_xyhead(gnss_xyhead_inv)
        vehicle_pose_global = replace(self.gnss_pose,
                                      t=self.vehicle_interface.time(),
                                      x=center_xyhead[0],
                                      y=center_xyhead[1],
                                      yaw=center_xyhead[2])

        readings = self.vehicle_interface.get_reading()
        
        #Transform from latitude/longitude to Cartesian coordinates in meters
        self.dx, self.dy = transforms.lat_lon_to_xy(vehicle_pose_global.x, vehicle_pose_global.y, self.lat_ref, self.long_ref)

        #propagate and update of the kalman filter
        self.estimator.propagate(self.imu_reading.linear_acceleration, self.imu_reading.angular_velocity, self.vehicle_interface.time())
        self.estimator.measurement_update(0.000025, np.array([self.dx, self.dy, vehicle_pose_global.z]))

        vehicle_pose_global.x, vehicle_pose_global.y = transforms.xy_to_lat_lon(self.estimator.p[0], self.estimator.p[1], self.lat_ref, self.long_ref)
        raw = readings.to_state(vehicle_pose_global)
        raw.v = self.gnss_speed
        return raw
        

class GNSSStateEstimator(Component):
    """Just looks at the GNSS reading to estimate the vehicle state"""
    def __init__(self, vehicle_interface : GEMInterface):
        self.vehicle_interface = vehicle_interface
        if 'gnss' not in vehicle_interface.sensors():
            raise RuntimeError("GNSS sensor not available")
        vehicle_interface.subscribe_sensor('gnss',self.gnss_callback,GNSSReading)
        self.gnss_pose = None
        self.location = settings.get('vehicle.calibration.gnss_location')[:2]
        self.yaw_offset = settings.get('vehicle.calibration.gnss_yaw')
        self.speed_filter  = OnlineLowPassFilter(1.2, 30, 4)
        self.status = None

    # Get GNSS information
    def gnss_callback(self, reading : GNSSReading):
        self.gnss_pose = reading.pose
        self.gnss_speed = reading.speed
        self.status = reading.status
    
    def rate(self):
        return 10.0
    
    def state_outputs(self) -> List[str]:
        return ['vehicle']

    def healthy(self):
        return self.gnss_pose is not None

    def update(self) -> VehicleState:
        if self.gnss_pose is None:
            return
        #TODO: figure out what this status means
        #print("INS status",self.status)

        # vehicle gnss heading (yaw) in radians
        # vehicle x, y position in fixed local frame, in meters
        # reference point is located at the center of GNSS antennas
        localxy = transforms.rotate2d(self.location,-self.yaw_offset)
        gnss_xyhead_inv = (-localxy[0],-localxy[1],-self.yaw_offset)
        center_xyhead = self.gnss_pose.apply_xyhead(gnss_xyhead_inv)
        vehicle_pose_global = replace(self.gnss_pose,
                                      t=self.vehicle_interface.time(),
                                      x=center_xyhead[0],
                                      y=center_xyhead[1],
                                      yaw=center_xyhead[2])

        readings = self.vehicle_interface.get_reading()
        raw = readings.to_state(vehicle_pose_global)

        #filtering speed
        raw.v = self.gnss_speed
        #filt_vel     = self.speed_filter(raw.v)
        #raw.v = filt_vel
        return raw
        


class OmniscientStateEstimator(Component):
    """A state estimator used for the simulator which provides perfect state information"""
    def __init__(self, vehicle_interface : GEMInterface):
        self.vehicle_interface = vehicle_interface
        if 'gnss' not in vehicle_interface.sensors():
            raise RuntimeError("GNSS sensor not available")
        vehicle_interface.subscribe_sensor('gnss',self.fake_gnss_callback)
        self.vehicle_state = None

    # Get GNSS information
    def fake_gnss_callback(self, vehicle_state):
        self.vehicle_state = vehicle_state
    
    def rate(self):
        return 50.0
    
    def state_outputs(self) -> List[str]:
        return ['vehicle']

    def healthy(self):
        return self.vehicle_state is not None

    def update(self) -> VehicleState:
        return self.vehicle_state
    

#alias, will be deprecated by end of February
FakeStateEstimator = OmniscientStateEstimator