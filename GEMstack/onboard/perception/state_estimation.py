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
import rospy
from std_msgs.msg import Float64
class IMUStateEstimator(Component):
    def __init__(self, vehicle_interface = GEMInterface):
        
        self.vehicle_interface = vehicle_interface
        if 'gnss' in vehicle_interface.sensors():
            vehicle_interface.subscribe_sensor('gnss',self.inspva_callback)

        if 'imu' in vehicle_interface.sensors():
            vehicle_interface.subscribe_sensor('imu',self.imu_callback)
        # self.imu_sub = rospy.Subscriber("/as_tx/vehicle_speed",Float64, self.vel_callback)
        self.gnss_pose = None
        self.imu_pose =  None
        self.location = settings.get('vehicle.calibration.gnss_location')[:2]
        self.yaw_offset = settings.get('vehicle.calibration.gnss_yaw')
        self.speed_filter  = OnlineLowPassFilter(1.2, 30, 4)
        self.status = None
        self.last_pose = ObjectPose(ObjectFrameEnum.GLOBAL,
                                    t=self.vehicle_interface.time(),
                                    x=0,
                                    y=0,
                                    z=0,
                                    yaw=0,  #heading from north in degrees
                                    roll=0,
                                    pitch=0,
                                    )
        self.linear_vx = 0
        self.linear_vy = 0
        self.linear_vz = 0
        # self.vehicle_interface = vehicle_interface
        # if 'gnss' not in vehicle_interface.sensors():
        #     raise RuntimeError("GNSS sensor not available")
        # vehicle_interface.subscribe_sensor('gnss',self.inspva_callback)
        # self.gnss_pose = None
        # self.location = settings.get('vehicle.calibration.gnss_location')[:2]
        # self.yaw_offset = settings.get('vehicle.calibration.gnss_yaw')
        # self.speed_filter  = OnlineLowPassFilter(1.2, 30, 4)
        # self.status = None
    
    # Get GNSS information
    def inspva_callback(self, inspva_msg):
        self.gnss_pose = ObjectPose(ObjectFrameEnum.GLOBAL,
                                    t=self.vehicle_interface.time(),
                                    x=inspva_msg.longitude,
                                    y=inspva_msg.latitude,
                                    z=inspva_msg.height,
                                    yaw=math.radians(inspva_msg.azimuth),  #heading from north in degrees
                                    roll=math.radians(inspva_msg.roll),
                                    pitch=math.radians(inspva_msg.pitch),
                                    )
        self.status = inspva_msg.status


    def imu_callback(self, msg):
        self.imu_ax = msg.linear_acceleration.x
        self.imu_ay = msg.linear_acceleration.y
        self.imu_az = msg.linear_acceleration.z
        self.ang_vx = msg.angular_velocity.x
        self.ang_vy = msg.angular_velocity.y
        self.ang_vz = msg.angular_velocity.z
    def rate(self):
        return 10.0
    
    def state_outputs(self) -> List[str]:
        return ['vehicle']

    def healthy(self):
        return self.gnss_pose is not None
    
    def garbage_value(self, pose):
        # if pose.x > 180 or pose.x < -180:
        #     return True
        # elif pose.y > 90 or pose.y < 0:
        #     return True
        # else:
        #     return False
        return True
        
    def create_imu_pose(self):
        time = self.vehicle_interface.time()
        past_time = self.last_pose.t
       
        self.imu_pose = self.last_pose
        self.imu_pose.x += 0.5* self.imu_ax* (time - past_time)**2 + self.linear_vx* (time - past_time)
        self.imu_pose.y += 0.5* self.imu_ay* (time - past_time)**2 + self.linear_vy* (time - past_time)
        self.imu_pose.z += 0.5* self.imu_az* (time - past_time)**2 + self.linear_vz* (time - past_time)
        self.imu_pose.yaw += self.ang_vx* (time - past_time)
        self.imu_pose.roll += self.ang_vy* (time - past_time)
        self.imu_pose.pitch += self.ang_vz* (time - past_time)
        # self.velocity = self.imu_ax* (time - past_time)
        self.linear_vx += self.imu_ax* (time - past_time)
        self.linear_vy += self.imu_ay* (time - past_time)
        self.linear_vz += self.imu_az* (time - past_time) 
        self.imu_pose.t = time
        print(self.imu_pose)
        print(self.linear_vx, self.linear_vy, self.linear_vz)
    def update(self) -> VehicleState:
        if self.garbage_value(self.gnss_pose):
            self.create_imu_pose()

            readings = self.vehicle_interface.get_reading()
            # print('pose', self.imu_pose.x)
            raw = readings.to_state(self.imu_pose)

            #filtering speed
            filt_vel     = self.speed_filter(raw.v)
            raw.v = filt_vel
            self.last_pose = self.imu_pose

        else:
            print('GNSS', self.location,-self.yaw_offset)
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
            filt_vel     = self.speed_filter(raw.v)
            raw.v = filt_vel
            self.last_pose = self.gnss_pose
            return raw


class GNSSStateEstimator(Component):
    """Just looks at the GNSS reading to estimate the vehicle state"""
    def __init__(self, vehicle_interface : GEMInterface):
        self.vehicle_interface = vehicle_interface
        if 'gnss' not in vehicle_interface.sensors():
            raise RuntimeError("GNSS sensor not available")
        vehicle_interface.subscribe_sensor('gnss',self.inspva_callback)
        self.gnss_pose = None
        self.location = settings.get('vehicle.calibration.gnss_location')[:2]
        self.yaw_offset = settings.get('vehicle.calibration.gnss_yaw')
        self.speed_filter  = OnlineLowPassFilter(1.2, 30, 4)
        self.status = None

    # Get GNSS information
    def inspva_callback(self, inspva_msg):
        self.gnss_pose = ObjectPose(ObjectFrameEnum.GLOBAL,
                                    t=self.vehicle_interface.time(),
                                    x=inspva_msg.longitude,
                                    y=inspva_msg.latitude,
                                    z=inspva_msg.height,
                                    yaw=math.radians(inspva_msg.azimuth),  #heading from north in degrees
                                    roll=math.radians(inspva_msg.roll),
                                    pitch=math.radians(inspva_msg.pitch),
                                    )
        self.status = inspva_msg.status
    
    def rate(self):
        return 10.0
    
    def state_outputs(self) -> List[str]:
        return ['vehicle']

    def healthy(self):
        return self.gnss_pose is not None

    def update(self) -> VehicleState:
        if self.gnss_pose is None:
            print('None')
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
        filt_vel     = self.speed_filter(raw.v)
        raw.v = filt_vel
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