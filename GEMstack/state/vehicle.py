from __future__ import annotations
from dataclasses import dataclass,replace
from ..utils.serialization import register
from .physical_object import ObjectFrameEnum, ObjectPose, PhysicalObject
from enum import Enum


class VehicleGearEnum(Enum):
    PARK = -2
    REVERSE = -1
    NEUTRAL = 0
    FIRST = 1
    SECOND = 2
    THIRD = 3
    FOURTH = 4
    FIFTH = 5
    SIXTH = 6
    

@dataclass
@register
class VehicleState:
    """Represents the state of the ego-vehicle."""
    pose : ObjectPose                       #pose of the vehicle with origin at rear axle center. Includes time
    v : float                               #forward velocity in m/s
    accelerator_pedal_position : float      #current accelerator pedal position, in [0,1]
    brake_pedal_position : float            #current brake pedal position, in [0,1]
    acceleration : float                    #current estimated acceleration / deceleration in m/s^2
    steering_wheel_angle : float            #angle of the steering wheel, in radians
    front_wheel_angle : float               #angle of the front wheels, in radians.   Related to steering_wheel_angle by a fixed transform
    heading_rate : float                    #the rate at which the vehicle is turning, in radians/s.  Related to v and front_wheel_angle by a fixed transform
    gear : int                  #the current gear
    left_turn_indicator : bool = False      #whether left turn indicator is on
    right_turn_indicator : bool = False     #whether right turn indicator is on
    headlights_on : bool = False            #whether headlights are on
    horn_on : bool = False                  #whether horn is on
    wiper_level : int = 0                   #whether wipers are on

    @staticmethod
    def zero():
        return VehicleState(ObjectPose(ObjectFrameEnum.START,0,0,0),0,0,0,0,0,0,0,VehicleGearEnum.PARK,False,False,False,0)
    
    def to_object(self) -> PhysicalObject:
        """Extracts out the geometry of the object using the vehicle's
        current geometry in settings.  Note that the object's origin will be in the
        middle of the vehicle, NOT the true vehicle reference point (rear axle center).
        """
        from ..utils import settings
        xbounds,ybounds,zbounds = settings.get('vehicle.geometry.bounds')
        height = settings.get('vehicle.geometry.height')
        center = [0.5*(xbounds[0]+xbounds[1]),0.5*(ybounds[0]+ybounds[1]),0.0]  #z needs to be on base
        dims = [xbounds[1]-xbounds[0],ybounds[1]-ybounds[0],zbounds[1]-zbounds[0]]
        center_new = self.pose.apply(center)
        c_x = center_new[0]
        c_y = center_new[1]
        if self.pose.z is not None:
            c_z = center_new[2]
        else:
            c_z = None
        center_pose = replace(self.pose,x=c_x,y=c_y,z=c_z)
        return PhysicalObject(pose = center_pose,
                              dimensions = dims,
                              outline = None)

    def to_frame(self, frame : ObjectFrameEnum, current_pose = None, start_pose_abs = None) -> VehicleState:
        return replace(self,pose=self.pose.to_frame(frame,current_pose,start_pose_abs))
