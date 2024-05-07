
import numpy as np
from klampt.math import vectorops as vo
from klampt.math import so2
from typing import Tuple

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

def lat_lon_to_xy(lat : float, lon : float, lat_reference : float, lon_reference : float):
    """ Conversion of Lat & Lon to X & Y.

    Returns (x,y), where x is east in m, y is north in m.
    """
    import alvinxy.alvinxy as axy 
    # convert GNSS waypoints into local fixed frame reprented in x and y
    east_x, north_y = axy.ll2xy(lat, lon, lat_reference, lon_reference)
    return east_x, north_y

def xy_to_lat_lon(x_east : float, y_north : float, lat_reference : float, lon_reference : float):
    """ Conversion of X & Y to Lat & Lon.

    Returns (lat,lon), where lat and lon are in degrees.
    """
    import alvinxy.alvinxy as axy 
    # convert GNSS waypoints into local fixed frame reprented in x and y
    lat, lon = axy.xy2ll(x_east, y_north, lat_reference, lon_reference)
    return lat, lon

def skew_symmetric(v):
    """ Skew symmetric operator for a 3x1 vector. """
    return np.array(
        [[0, -v[2][0], v[1][0]],
         [v[2][0], 0, -v[0][0]],
         [-v[1][0], v[0][0], 0]]
    )

#Adapted from https://arxiv.org/pdf/1711.02508
class Quaternion():
    def __init__(self, w=1., x=0., y=0., z=0., axis_angle=None, euler=None):
        """
        Allow initialization either with explicit wxyz, axis angle, or euler XYZ (RPY) angles.

        :param w: w (real) of a quaternion.
        :param x: x (i) of a quaternion.
        :param y: y (j) of a quaternion.
        :param z: z (k) of a quaternion.
        :param axis_angle: Set of three values from axis-angle representation, as list or [3,] or [3,1] np.ndarray.
                           See C2M5L2 Slide 7 for details.
        :param euler: Set of three angles from euler XYZ rotating frame angles.
        """
        if axis_angle is None and euler is None:
            self.w = w
            self.x = x
            self.y = y
            self.z = z
        elif euler is not None and axis_angle is not None:
            raise AttributeError("Only one of axis_angle and euler can be specified.")
        elif axis_angle is not None:
            if not (type(axis_angle) == list or type(axis_angle) == np.ndarray) or len(axis_angle) != 3:
                raise ValueError("axis_angle must be list or np.ndarray with length 3.")
            axis_angle = np.array(axis_angle)
            norm = np.linalg.norm(axis_angle)
            self.w = np.cos(norm / 2)
            if norm < 1e-50:  # to avoid instabilities and nans
                self.x = 0
                self.y = 0
                self.z = 0
            else:
                imag = axis_angle / norm * np.sin(norm / 2)
                self.x = imag[0].item()
                self.y = imag[1].item()
                self.z = imag[2].item()
        else:
            roll = euler[0]
            pitch = euler[1]
            yaw = euler[2]

            cy = np.cos(yaw * 0.5)
            sy = np.sin(yaw * 0.5)
            cr = np.cos(roll * 0.5)
            sr = np.sin(roll * 0.5)
            cp = np.cos(pitch * 0.5)
            sp = np.sin(pitch * 0.5)

            # static frame
            self.w = cr * cp * cy + sr * sp * sy
            self.x = sr * cp * cy - cr * sp * sy
            self.y = cr * sp * cy + sr * cp * sy
            self.z = cr * cp * sy - sr * sp * cy


    def __repr__(self):
        return "Quaternion (wxyz): [%2.5f, %2.5f, %2.5f, %2.5f]" % (self.w, self.x, self.y, self.z)

    def to_mat(self):
        v = np.array([self.x, self.y, self.z]).reshape(3,1)
        return (self.w ** 2 - np.dot(v.T, v)) * np.eye(3) + \
               2 * np.dot(v, v.T) + 2 * self.w * skew_symmetric(v)

    def to_euler(self):
        """ Return as xyz (roll pitch yaw) Euler angles, with a static frame """
        roll = np.arctan2(2 * (self.w * self.x + self.y * self.z), 1 - 2 * (self.x**2 + self.y**2))
        pitch = np.arcsin(2 * (self.w * self.y - self.z * self.x))
        yaw = np.arctan2(2 * (self.w * self.z + self.x * self.y), 1 - 2 * (self.y**2 + self.z**2))
        return np.array([roll, pitch, yaw])

    def to_numpy(self):
        """ Return numpy wxyz representation. """
        return np.array([self.w, self.x, self.y, self.z])

    def normalize(self):
        """ Return a normalized version of this quaternion to ensure that it is valid. """
        norm = np.linalg.norm([self.w, self.x, self.y, self.z])
        return Quaternion(self.w / norm, self.x / norm, self.y / norm, self.z / norm)

    def quat_mult(self, q, out='np'):
        """
        Give output of multiplying this quaternion by another quaternion.

        :param q: The quaternion to multiply by, either a Quaternion or 4x1 ndarray.
        :param out: Output type, either np or Quaternion.
        :return: Returns quaternion of desired type
        """

        v = np.array([self.x, self.y, self.z]).reshape(3, 1)
        sum_term = np.zeros([4,4])
        sum_term[0,1:] = -v[:,0]
        sum_term[1:, 0] = v[:,0]
        sum_term[1:, 1:] = -skew_symmetric(v)
        sigma = self.w * np.eye(4) + sum_term

        if type(q).__name__ == "Quaternion":
            quat_np = np.dot(sigma, q.to_numpy())
        else:
            quat_np = np.dot(sigma, q)

        if out == 'np':
            return quat_np
        elif out == 'Quaternion':
            quat_obj = Quaternion(quat_np[0], quat_np[1], quat_np[2], quat_np[3])
            return quat_obj