from __future__ import annotations
from dataclasses import dataclass, replace, field
from ..mathutils import transforms
from ..utils.serialization import register
from typing import Tuple,List,Optional
from enum import Enum
import numpy as np
from klampt.math import so2,so3,se3

class ObjectFrameEnum(Enum):
    START = 0                  #position / yaw in m / radians relative to starting pose of vehicle 
    CURRENT = 1                #position / yaw in m / radians relative to current pose of vehicle
    GLOBAL = 2                 #position in longitude / latitude, yaw=heading in radians with respect to true north (used in GNSS)
    ABSOLUTE_CARTESIAN = 3     #position / yaw  in m / radians relative to a global cartesian reference frame (used in simulation)


@dataclass
@register
class ObjectPose:
    """
    Represents a hypothetical object position / orientation.
    
    Attributes:
        frame: the frame of reference for the pose. 
        t: if frame=GLOBAL or ABSOLUTE_CARTESIAN, the time in s since the
            epoch, i.e., time.time()  Otherwise, the time since start / current
            in the future, in s
        x: the x position, in the object's frame.  If frame=GLOBAL, this is
            longitude, otherwise forward in m.
        y: the y position, in the object's frame.  If frame=GLOBAL, this is
            latitude, otherwise left in m.
        z: the optional z position, in m and in the object's frame.
        yaw: the optional yaw, in radians and in the object's frame. If
            frame=GLOBAL, this is heading CW from north.  Otherwise, it is
            CCW yaw.
        pitch: the optional pitch, in radians and around left direction in the object's frame
        roll: the optional roll, in radians and around forward direction in the object's frame

    """
    frame : ObjectFrameEnum
    t : float
    x : float
    y : float
    z : Optional[float] = None
    yaw : Optional[float] = None
    pitch : Optional[float] = None
    roll : Optional[float] = None

    def rotation2d(self) -> np.ndarray:
        """Returns the 2x2 rotation matrix of this pose's yaw relative to the specified frame."""
        yaw = self.yaw if self.yaw is not None else 0.0
        if self.frame == ObjectFrameEnum.GLOBAL:
            yaw = transforms.heading_to_yaw(yaw,False)
        return so2.ndarray(yaw)

    def rotation(self) -> np.ndarray:
        """Returns the 3x3 rotation matrix of this pose relative to the specified frame."""
        rpy = [(self.roll if self.roll is not None else 0.0),
               (self.pitch if self.pitch is not None else 0.0),
               (self.yaw if self.yaw is not None else 0.0)]
        if self.frame == ObjectFrameEnum.GLOBAL:
            rpy[2] = transforms.heading_to_yaw(rpy[2],False)
        return so3.ndarray(so3.from_rpy(rpy))

    def translation(self) -> np.ndarray:
        """Returns the 3D translation of this pose relative to the specified frame."""
        if self.frame == ObjectFrameEnum.GLOBAL:
            raise ValueError("Cannot get translation of GLOBAL frame")
        return np.array([self.x,self.y,(self.z if self.z is not None else 0.0)])
    
    def transform(self) -> np.ndarray:
        """Returns the 4x4 homogeneous transform matrix of this pose."""
        return se3.ndarray((so3.from_ndarray(self.rotation()),self.translation()))

    def apply(self,point):
        """Applies this pose to a local (x,y) or (x,y,z) coordinate.
        
        If point is 2D, then the pitch and roll are ignored.
        """
        assert len(point) in [2,3],"Must provide a 2D or 3D point"
        oz = self.z if self.z is not None else 0.0
        if self.frame == ObjectFrameEnum.GLOBAL:
            east_m,north_m = self.rotation2d().dot(point[:2])
            olon,olat = self.x,self.y
            lat,lon = transforms.xy_to_lat_lon(east_m,north_m,olat,olon)
            if len(point) == 2:
                return (lon,lat)
            else:
                return (lon,lat, point[2] + oz)
        if len(point) == 2:
            return tuple(self.rotation2d().dot(point) + self.translation()[:2])
        else:
            return tuple(self.rotation().dot(point) + self.translation())

    def apply_inv(self,point):
        """Applies the inverse of this pose to an (x,y) or (x,y,z) coordinate
        specified in the same frame as this.
        
        If point is 2D, then the pitch and roll are ignored.  Otherwise, they
        are taken into account.
        """
        assert len(point) in [2,3],"Must provide a 2D or 3D point"
        oz = self.z if self.z is not None else 0.0
        if self.frame == ObjectFrameEnum.GLOBAL:
            lon,lat = point[:2]
            olon,olat = self.x,self.y
            east_m, north_m = transforms.lat_lon_to_xy(lat,lon,olat,olon)
            px,py = self.rotation2d().T.dot((east_m,north_m))
            if len(point) == 2:
                return (px,py)
            else:
                return (px,py, point[2] - oz)
        if len(point) == 2:
            return tuple(self.rotation2d().T.dot(np.array(point)-self.translation()[:2]))
        else:
            return tuple(self.rotation().T.dot(np.array(point)-self.translation()))

    def apply_dir(self,vec):
        """Applies this pose to a local (x,y) or (x,y,z) directional quantity.
        
        If direction is 2D, then the pitch and roll are ignored.
        """
        assert len(vec) in [2,3],"Must provide a 2D or 3D direction"
        oz = self.z if self.z is not None else 0.0
        if self.frame == ObjectFrameEnum.GLOBAL:
            east_m,north_m = self.rotation2d().dot(vec[:2])
            if len(vec) == 2:
                return (east_m,north_m)
            else:
                return (east_m,north_m, vec[2] + oz)
        if len(vec) == 2:
            return tuple(self.rotation2d().dot(vec))
        else:
            return tuple(self.rotation().dot(vec))

    def apply_dir_inv(self,vec):
        """Applies the inverse of this pose to an (x,y) or (x,y,z) directional
        quantity specified in the same frame as this.
        
        If direction is 2D, then the pitch and roll are ignored.  Otherwise, they
        are taken into account.
        """
        assert len(vec) in [2,3],"Must provide a 2D or 3D direction"
        oz = self.z if self.z is not None else 0.0
        if self.frame == ObjectFrameEnum.GLOBAL:
            east_m, north_m = vec[:2]
            px,py = self.rotation2d().T.dot((east_m,north_m))
            if len(vec) == 2:
                return (px,py)
            else:
                return (px,py, vec[2] - oz)
        if len(vec) == 2:
            return tuple(self.rotation2d().T.dot(vec))
        else:
            return tuple(self.rotation().T.dot(vec))

    def apply_xyhead(self,xyhead):
        """Applies this pose to a local (x,y,yaw) coordinate.  yaw is always
        specified in CCW radians.  Pitch and roll are ignored.
        """
        newxy = self.apply(xyhead[:2])
        yaw = self.yaw if self.yaw is not None else 0.0
        if self.frame == ObjectFrameEnum.GLOBAL:
            return newxy + (transforms.normalize_angle(yaw - xyhead[2]),)
        else:
            return newxy + (transforms.normalize_angle(yaw + xyhead[2]),)
    
    def apply_inv_xyhead(self,xyhead):
        """Applies this pose to a (x,y,yaw) coordinate expressed in `frame`.
        yaw is specified in CCW radians except for GLOBAL, in which case
        yaw is CW heading.  Pitch and roll are ignored."""
        newxy = self.apply_inv(xyhead[:2])
        yaw = self.yaw if self.yaw is not None else 0.0
        if self.frame == ObjectFrameEnum.GLOBAL:
            return newxy + (transforms.normalize_angle(yaw - xyhead[2]),)
        else:
            return newxy + (transforms.normalize_angle(xyhead[2] - yaw),)

    def to_frame(self, new_frame : ObjectFrameEnum, 
                 current_pose : ObjectPose = None, start_pose_abs : ObjectPose = None) -> ObjectPose:
        """Returns a new ObjectPose representing the same pose, but with
        coordinates expressed in a different frame. 
        
        Note that pitch and roll will be preserved!
        """
        if self.frame == new_frame:
            return replace(self)
        frame_chain = _get_frame_chain(self.frame,new_frame,current_pose,start_pose_abs)
        if self.yaw is None:
            pt = (self.x,self.y)
            for (frame,pose,dir) in frame_chain[1:]:
                if dir == 1:
                    pt = pose.apply(pt)
                else:
                    pt = pose.apply_inv(pt)
            new_x,new_y = pt
            new_yaw = None
        else:
            xyhead = (self.x,self.y,self.yaw)
            for (frame,pose,dir) in frame_chain[1:]:
                if dir == 1:
                    xyhead = pose.apply_xyhead(xyhead)
                else:
                    xyhead = pose.apply_inv_xyhead(xyhead)
            new_x,new_y,new_yaw = xyhead
        new_z = self.z
        if new_z is not None:
            for (frame,pose,dir) in frame_chain[1:]:
                if pose.z is not None:
                    new_z += pose.z*dir
        new_t = self.t
        for (frame,pose,dir) in frame_chain[1:]:
            new_t += pose.t*dir
        return replace(self, frame=new_frame, t=new_t,x=new_x, y=new_y, z=new_z, yaw=new_yaw)


@dataclass
@register
class PhysicalObject:
    """Base class for some physical possibly movable object.
    
    The origin is at the object's center in the x-y plane but at the bottom
    in the z axis.  I.e., if l,w,h are the dimensions, then the object is
    contained in a bounding box [-l/2,l/2] x [-w/2,w/2] x [0,h].
    
    Attributes:
        pose: the position / rotation coordinates of the object.
        dimensions: the length (forward), width (sideways), and height (up)
            of the object, in the object's local frame.
        outline: an optional list of vertices in CCW order denoting the
            object's outline polygon in its local frame (x:forward, y:left).

    """
    pose : ObjectPose
    dimensions : Tuple[float,float,float] 
    outline : Optional[List[Tuple[float,float]]]

    def bounds(self) -> Tuple[Tuple[float,float],Tuple[float,float],Tuple[float,float]]:
        """Returns the bounding box of the object in its local frame."""
        l,w,h = self.dimensions
        return [[-l/2,l/2],[-w/2,w/2],[0,h]]

    def to_frame(self, frame : ObjectFrameEnum, current_pose = None, start_pose_abs = None):
        newpose = self.pose.to_frame(frame,current_pose,start_pose_abs)
        return replace(self,pose=newpose)



def _get_frame_chain(source_frame : ObjectFrameEnum, target_frame : ObjectFrameEnum,
                     current_pose : ObjectPose = None, start_pose_abs : ObjectPose = None) -> List[Tuple[ObjectFrameEnum,ObjectPose,int]]:
    frame_chain = [(source_frame,None,0)]
    if source_frame == target_frame:
        return frame_chain
    
    if source_frame == ObjectFrameEnum.GLOBAL and target_frame == ObjectFrameEnum.ABSOLUTE_CARTESIAN:
        raise ValueError("Cannot mix GLOBAL and ABSOLUTE_CARTESIAN frames")
    elif target_frame == ObjectFrameEnum.GLOBAL and source_frame == ObjectFrameEnum.ABSOLUTE_CARTESIAN:
        raise ValueError("Cannot mix GLOBAL and ABSOLUTE_CARTESIAN frames")
    if current_pose is not None and current_pose.frame == ObjectFrameEnum.CURRENT:
        raise ValueError("Cannot accept current_pose in CURRENT frame")
    if start_pose_abs is not None and start_pose_abs.frame not in [ObjectFrameEnum.GLOBAL,ObjectFrameEnum.ABSOLUTE_CARTESIAN]:
        raise ValueError("start_pose_abs must be in GLOBAL or ABSOLUTE_CARTESIAN frame")
    if current_pose is not None and current_pose.frame in [ObjectFrameEnum.GLOBAL,ObjectFrameEnum.ABSOLUTE_CARTESIAN]:
        if current_pose.frame != start_pose_abs.frame:
            raise ValueError("Cannot mix GLOBAL and ABSOLUTE_CARTESIAN frames")
    if frame_chain[-1][0] == ObjectFrameEnum.CURRENT:
        if current_pose is None:
            raise ValueError("current_pose must be specified when converting from CURRENT")
        frame_chain.append((current_pose.frame,current_pose,1))
    if frame_chain[-1][0] == target_frame:
        return frame_chain
    if frame_chain[-1][0] == ObjectFrameEnum.START:
        if start_pose_abs is None:
            raise ValueError("start_pose_abs must be specified when converting from START")
        frame_chain.append((start_pose_abs.frame,start_pose_abs,1))
    if frame_chain[-1][0] == target_frame:
        return frame_chain
    if target_frame == ObjectFrameEnum.START:    
        if start_pose_abs is None:
            raise ValueError("start_pose_abs must be specified when converting to START")
        frame_chain.append((ObjectFrameEnum.START,start_pose_abs,-1))
    elif target_frame == ObjectFrameEnum.CURRENT:
        if current_pose.frame != frame_chain[-1][0]:  #global to start
            assert start_pose_abs.frame == frame_chain[-1][0]
            assert current_pose.frame == ObjectFrameEnum.START
            if start_pose_abs is None:
                raise ValueError("start_pose_abs must be specified when converting to CURRENT and current_pose is in START frame")
            frame_chain.append((ObjectFrameEnum.START,start_pose_abs,-1))
        if current_pose is None:
            raise ValueError("current_pose must be specified when converting to CURRENT")
        frame_chain.append((ObjectFrameEnum.CURRENT,current_pose,-1))
    return frame_chain


def convert_point(source_pt : tuple, source_frame : ObjectFrameEnum, target_frame : ObjectFrameEnum,
                  current_pose : ObjectPose = None, start_pose_abs : ObjectPose = None) -> tuple:
    """Converts an (x,y) or (x,y,z) point from one frame to another. 

    start_pose_abs must be in GLOBAL or ABSOLUTE_CARTESIAN frame.

    current_pose may be in START, GLOBAL, or ABSOLUTE_CARTESIAN frame.

    GLOBAL and ABSOLUTE_CARTESIAN are incompatible.
    """
    frame_chain = _get_frame_chain(source_frame,target_frame,current_pose,start_pose_abs)
    pt = source_pt
    for (frame,pose,dir) in frame_chain[1:]:
        if dir == 1:
            pt = pose.apply(pt)
        else:
            pt = pose.apply_inv(pt)
    return pt

def convert_vector(source_vec : tuple, source_frame : ObjectFrameEnum, target_frame : ObjectFrameEnum,
                  current_pose : ObjectPose = None, start_pose_abs : ObjectPose = None) -> tuple:
    """Converts an (x,y) or (x,y,z) vector from one frame to another. 

    start_pose_abs must be in GLOBAL or ABSOLUTE_CARTESIAN frame.

    current_pose may be in START, GLOBAL, or ABSOLUTE_CARTESIAN frame.

    GLOBAL and ABSOLUTE_CARTESIAN are incompatible.
    """
    frame_chain = _get_frame_chain(source_frame,target_frame,current_pose,start_pose_abs)
    vec = source_vec
    for (frame,pose,dir) in frame_chain[1:]:
        if len(vec) == 2:
            R = pose.rotation2d()
        else:
            R = pose.rotation()
        if dir == 1:
            vec = R.dot(vec)
        else:
            vec = R.T.dot(vec)
    return tuple(vec)

def convert_xyhead(source_state : Tuple[float,float,float], source_frame : ObjectFrameEnum, target_frame : ObjectFrameEnum,
                   current_pose : ObjectPose = None, start_pose_abs : ObjectPose = None) -> Tuple[float,float,float]:
    """Converts an (x,y,heading) state from one frame to another.
    
    start_pose_abs must be in GLOBAL or ABSOLUTE_CARTESIAN frame.

    current_pose may be in START, GLOBAL, or ABSOLUTE_CARTESIAN frame.

    GLOBAL and ABSOLUTE_CARTESIAN are incompatible.
    """
    frame_chain = _get_frame_chain(source_frame,target_frame,current_pose,start_pose_abs)

    xyhead = source_state
    for (frame,pose,dir) in frame_chain[1:]:
        if dir == 1:
            xyhead = pose.apply_xyhead(xyhead)
        else:
            xyhead = pose.apply_inv_xyhead(xyhead)
    return xyhead


def convert_points(source_pts : List[tuple], source_frame : ObjectFrameEnum, target_frame : ObjectFrameEnum,
                  current_pose : ObjectPose = None, start_pose_abs : ObjectPose = None) -> Tuple[float,float]:
    """Converts a list of (x,y) or (x,y,z) points from one frame to
    another.  Faster than repeated calls to convert_point.
    """
    frame_chain = _get_frame_chain(source_frame,target_frame,current_pose,start_pose_abs)
    res = []
    for pt in source_pts:
        for (frame,pose,dir) in frame_chain[1:]:
            if dir == 1:
                pt = pose.apply(pt)
            else:
                pt = pose.apply_inv(pt)
        res.append(pt)
    return res


def convert_xyheads(source_states : List[tuple], source_frame : ObjectFrameEnum, target_frame : ObjectFrameEnum,
                   current_pose : ObjectPose = None, start_pose_abs : ObjectPose = None) -> Tuple[float,float,float]:
    """Converts a list of (x,y,heading) states from one frame to another.
    Faster than repeated calls to convert_xyhead.
    """
    frame_chain = _get_frame_chain(source_frame,target_frame,current_pose,start_pose_abs)

    res = []
    for xyhead in source_states:
        for (frame,pose,dir) in frame_chain[1:]:
            if dir == 1:
                xyhead = pose.apply_xyhead(xyhead)
            else:
                xyhead = pose.apply_xyhead_inv(xyhead)
        res.append(xyhead)
    return res
