import numpy as np
from klampt.math import vectorops as vo
from klampt.math import so2
from typing import Tuple
import math

def normalize_angle(angle : float) -> float:
    """Normalizes an angle to be in the range [0,2pi]"""
    return so2.normalize(angle)

def normalize_vector(v):
    """Normalizes a vector"""
    return vo.unit(v)

def vector_add(v1, v2):
    """Adds v1 + v2 between two vectors"""
    return vo.add(v1,v2)

def vector_sub(v1, v2):
    """Subtracts v1 - v2 between two vectors"""
    return vo.sub(v1,v2)

def vector_mul(v, s:float):
    """Returns v*s"""
    return vo.mul(v,s)

def vector_madd(v1, v2, s:float):
    """Returns v1 + v2*s"""
    return vo.madd(v1,v2,s)

def vector_norm(v) -> float:
    """Norm of a vector"""
    return vo.norm(v)

def vector_dist(v1, v2) -> float:
    """Euclidean distance between two vectors"""
    return vo.distance(v1,v2)

def vector_dot(v1, v2) -> float:
    """Dot product between two vectors"""
    return vo.dot(v1,v2)

def vector_cross(v1, v2) -> float:
    """Cross product between two 2D vectors"""
    return vo.cross(v1,v2)

def vector2_angle(v1, v2 = None) -> float:
    """find the ccw angle bewtween two 2d vectors"""
    if v2 is None:
        cosang = v1[0]
        sinang = v1[1]
    else:
        cosang = np.dot(v1, v2)
        sinang = np.cross(v1, v2)
    return np.arctan2(sinang, cosang)

def vector2_dist(v1, v2) -> float:
    """Euclidean distance between two 2D vectors"""
    return np.linalg.norm([v1[0] - v2[0], v1[1] - v2[1]])

def point_segment_distance(x,a,b) -> Tuple[float,float]:
    """Computes the distance from point x to segment ab.  Returns
    a tuple (distance, parameter) where parameter is the parameter
    value on the segment in the range [0,1] indicating the closest
    point.
    """
    v = vector_sub(b,a)
    u = vector_sub(x,a)
    vnorm = vector_norm(v)
    if vnorm < 1e-6:
        return vector_norm(u),0
    udotv = np.dot(u,v)
    if udotv < 0:
        return vector_norm(u),0
    elif udotv > vnorm:
        return vector_norm(vector_sub(x,b)),1
    else:
        param = udotv/vnorm
        return vector_norm(vector_sub(u,vector_mul(v,param/vnorm))),param

def rotate2d(point, angle : float, origin=None):
    """Rotates a point about the origin by an angle"""
    if origin is None:
        return so2.apply(angle, point)
    else:
        return vo.add(origin,so2.apply(angle, vo.sub(point,origin)))

def heading_to_yaw(heading : float, degrees=True) -> float:
    """Conversion of GNSS heading (CW from north) to yaw (CCW w.r.t. east).

    If degrees is True, heading is in degrees, otherwise radians.
    """
    if not degrees:
        heading = np.degrees(heading)
    if heading >= 0 and heading < 90:
        return np.radians(-heading+90)
    else:
        return np.radians(-heading+450)

def yaw_to_heading(yaw : float, degrees=True) -> float:
    """Conversion of yaw (CCW w.r.t. east) to GNSS heading (CW from north).
    
    If degrees is True, heading will be reported in in degrees, otherwise
    radians.
    """
    yaw_degrees = np.degrees(yaw)
    heading_degrees = -yaw_degrees + 90
    if heading_degrees < 0:
        heading_degrees += 360
    if degrees:
        return heading_degrees
    else:
        return np.radians(heading_degrees)

def lat_lon_to_xy(lat: float, lon: float, lat_ref: float, lon_ref: float) -> Tuple[float, float]:
    """Convert geographic coordinates to local Cartesian coordinates.
    
    Args:
        lat, lon: Target point (degrees)
        lat_ref, lon_ref: Reference origin point (degrees)
        
    Returns:
        (east_x, north_y) in meters
    """
    # Earth radius in meters
    R = 6371000.0  
    
    # Convert reference latitude to radians
    lat_ref_rad = math.radians(lat_ref)
    
    # Calculate deltas in degrees
    delta_lat = lat - lat_ref
    delta_lon = lon - lon_ref
    
    # Convert to meters
    x = R * math.radians(delta_lon) * math.cos(lat_ref_rad)
    y = R * math.radians(delta_lat)
    
    return x, y

def xy_to_lat_lon(x: float, y: float, lat_ref: float, lon_ref: float) -> Tuple[float, float]:
    """Convert local Cartesian coordinates back to geographic coordinates.
    
    Args:
        x: Easting in meters
        y: Northing in meters
        lat_ref, lon_ref: Reference origin point (degrees)
        
    Returns:
        (latitude, longitude) in degrees
    """
    R = 6371000.0
    lat_ref_rad = math.radians(lat_ref)
    
    # Convert meters to degrees
    delta_lon = x / (R * math.cos(lat_ref_rad))
    delta_lat = y / R
    
    # Calculate final coordinates
    lat = lat_ref + math.degrees(delta_lat)
    lon = lon_ref + math.degrees(delta_lon)
    
    return lat, lon

def quaternion_to_euler(x : float, y : float, z : float, w : float):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = np.arctan2(t0, t1)
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = np.arcsin(t2)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = np.arctan2(t3, t4)
    return [roll, pitch, yaw]
