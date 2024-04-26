
import numpy as np
from klampt.math import vectorops as vo
from klampt.math import so2
from typing import Tuple
from . import alvinxy as axy 

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

def lat_lon_to_xy(lat : float, lon : float, lat_reference : float, lon_reference : float):
    """ Conversion of Lat & Lon to X & Y.

    Returns (x,y), where x is east in m, y is north in m.
    """
    # import alvinxy.alvinxy as axy 
    # convert GNSS waypoints into local fixed frame reprented in x and y
    east_x, north_y = axy.ll2xy(lat, lon, lat_reference, lon_reference)
    return east_x, north_y

def xy_to_lat_lon(x_east : float, y_north : float, lat_reference : float, lon_reference : float):
    """ Conversion of X & Y to Lat & Lon.

    Returns (lat,lon), where lat and lon are in degrees.
    """
    # import alvinxy.alvinxy as axy 
    # convert GNSS waypoints into local fixed frame reprented in x and y
    lat, lon = axy.xy2ll(x_east, y_north, lat_reference, lon_reference)
    return lat, lon
