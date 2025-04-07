from __future__ import annotations
from dataclasses import dataclass,replace
from ..utils.serialization import register
from ..mathutils import transforms,collisions
from .physical_object import ObjectFrameEnum, convert_point
import math
from typing import List,Tuple,Optional,Union

@dataclass
@register
class Path:
    """An untimed, piecewise linear path."""
    frame : ObjectFrameEnum
    points : List[List[float]]

    def to_frame(self, frame : ObjectFrameEnum, current_pose = None, start_pose_abs =  None) -> Path:
        """Converts the route to a different frame."""
        new_points = [convert_point(p,self.frame,frame,current_pose,start_pose_abs) for p in self.points]
        return replace(self,frame=frame,points=new_points)

    def domain(self) -> Tuple[float,float]:
        """Returns the parameter domain"""
        return (0.0,len(self.points)-1)

    def parameter_to_index(self, t : float) -> Tuple[int,float]:
        """Converts a path parameter to an (edge index, edge parameter) tuple."""
        if len(self.points) < 2:
            return 0,0.0
        ind = int(math.floor(t))  #truncate toward zero
        if ind < 0: ind = 0
        if ind >= len(self.points)-1: ind = len(self.points)-2
        u = t - ind
        if u > 1: u = 1
        if u < 0: u = 0
        return ind,u

    def eval(self, u : float) -> List[float]:
        """Evaluates the path at a given parameter.  The integer part of u
        indicates the segment, and the fractional part indicates the progress
        along the segment"""
        if len(self.points) < 2:
            return self.points[0]
        ind,u = self.parameter_to_index(u)
        p1 = self.points[ind]
        p2 = self.points[ind+1]
        return transforms.vector_madd(p1,transforms.vector_sub(p2,p1),u)

    def eval_derivative(self, u : float) -> List[float]:
        """Evaluates the derivative at a given parameter.  The integer part of 
        u indicates the segment, and the fractional part indicates the progress
        along the segment."""
        ind = int(u)
        if ind < 0: ind = 0
        if ind >= len(self.points)-1: ind = len(self.points)-2
        u = u - ind
        if u > 1: u = 1
        if u < 0: u = 0
        p1 = self.points[ind]
        p2 = self.points[ind+1]
        return transforms.vector_sub(p2,p1)

    def length(self):
        """Returns the length of the path."""
        l = 0.0
        for i in range(len(self.points)-1):
            l += transforms.vector_dist(self.points[i],self.points[i+1])
        return l

    def arc_length_parameterize(self, speed = 1.0) -> Trajectory:
        """Returns a new path that is parameterized by arc length."""
        times = [0.0]
        points = [self.points[0]]
        for i in range(len(self.points)-1):
            p1 = self.points[i]
            p2 = self.points[i+1]
            d = transforms.vector_dist(p1,p2)
            if d > 0:
                points.append(p2)
                times.append(times[-1] + d/speed)
        return Trajectory(frame=self.frame,points=points,times=times)

    def closest_point(self, x : List[float], edges = True) -> Tuple[float,float]:
        """Returns the closest point on the path to the given point.  If
        edges=False, only computes the distances to the vertices, not the
        edges.  This is slightly faster but less accurate.
        
        Returns (distance, closest_parameter)
        """
        best_dist = float('inf')
        best_point = None
        for i,p in enumerate(self.points):
            if edges and i > 0: 
                p1 = self.points[i-1]
                p2 = p
                dist,u = transforms.point_segment_distance(x,p1,p2)
                if dist < best_dist:
                    best_dist = dist
                    best_point = i-1+u
            else:
                dist = transforms.vector_dist(p,x)
                if dist < best_dist:
                    best_dist = dist
                    best_point = i
        return best_dist,best_point

    def closest_point_local(self, x : List[float], param_range=Tuple[float,float], edges = True) -> Tuple[float,float]:
        """Returns the closest point on the path to the given point within
        the given parameter range.
        
        If edges=False, only computes the distances to the vertices, not the
        edges.  This is slightly faster but less accurate.
        
        Returns (distance, closest_parameter)
        """
        best_dist = float('inf')
        param_range = [max(param_range[0],0),min(param_range[1],len(self.points))]
        imin = int(math.floor(param_range[0]))
        imax = int(math.floor(param_range[1]))
        if imax == len(self.points):
            imax -= 1
        
        umin = param_range[0] - imin
        umax = param_range[1] - imax
        best_point = None
        for i in range(imin,imax+1):
            p = self.points[i]
            if edges and i > 0: 
                p1 = self.points[i-1]
                p2 = p
                dist,u = transforms.point_segment_distance(x,p1,p2)
                if dist < best_dist:
                    best_dist = dist
                    best_point = i-1+u
            else:
                dist = transforms.vector_dist(p,x)
                if dist < best_dist:
                    best_dist = dist
                    best_point = i
        return best_dist,best_point

    def get_dims(self, dims : List[int]) -> Path:
        """Returns a new path with only the given dimensions."""
        return replace(self,points=[[p[d] for d in dims] for p in self.points])

    def append_dim(self, value : Union[float,List[float]] = 0.0) -> None:
        """Appends a dimension to every point.  If value is a list, then it
        must be the same length as the number of points.  Otherwise, the value
        is appended to every point."""
        if isinstance(value,(int,float)):
            for p in self.points:
                p.append(value)
        else:
            if len(self.points) != len(value):
                raise ValueError("Invalid length of values to append")
            for p,v in zip(self.points,value):
                p.append(v)
    
    def trim(self, start : float, end : float) -> Path:
        """Returns a copy of this path but trimmed to the given parameter range."""
        sind,su = self.parameter_to_index(start)
        eind,eu = self.parameter_to_index(end)
        s = self.eval(start)
        e = self.eval(end)
        return replace(self,points=[s]+self.points[sind+1:eind]+[e])
    

    def fit_curve_radius(self, vehicle_position: List[float], n_points: int) -> float:
        """
        Fits a circular curve to n upcoming points from the vehicle's current position
        and returns the radius of that curve.
        
        Args:
            vehicle_position: Current position of the vehicle [x, y, ...]
            n_points: Number of upcoming points to consider for curve fitting
            
        Returns:
            float: Radius of the fitted circular curve. Returns float('inf') for
                straight lines or when insufficient points are available.
        """
        # Find the closest point on the path to the vehicle position
        _, closest_point_idx_float = self.closest_point(vehicle_position)

        # I use ceil here because we want to be looking forward to the next set of points
        closest_point_idx = int(math.ceil(closest_point_idx_float))
        
        # Get indices for n upcoming points
        start_idx = closest_point_idx
        end_idx = min(start_idx + n_points, len(self.points) - 1)
        
        # If we don't have enough points for fitting, return infinite radius (straight line)
        if end_idx - start_idx < 2:
            return float('inf')
        

        # Extract upcoming points for fitting
        points_to_fit = self.points[start_idx:end_idx+1]
        
        # Ensure we have at least 3 points for circle fitting
        if len(points_to_fit) < 3:
            return float('inf')
        
        # For now, we'll focus on 2D paths
        # If points are higher dimensional, we'll use only x,y coordinates
        points_2d = [[p[0], p[1]] for p in points_to_fit]
        
        # Fit circle to points using least squares method
        try:
            # Center of circle (h, k) and radius r
            h, k, r = self._fit_circle_to_points(points_2d)
            return r
        except:
            # If fitting fails (e.g., collinear points), return infinite radius
            return float('inf')

    def _fit_circle_to_points(self, points: List[List[float]]) -> Tuple[float, float, float]:
        """
        Fits a circle to a set of 2D points using least squares method.
        
        Args:
            points: List of [x, y] coordinates
            
        Returns:
            Tuple[float, float, float]: Center coordinates (h, k) and radius r
        """
        # Number of points
        n = len(points)
        
        # Compute means of x and y coordinates
        x_mean = sum(p[0] for p in points) / n
        y_mean = sum(p[1] for p in points) / n
        
        # Compute sums for least squares fitting
        sum_x = sum((p[0] - x_mean) ** 2 for p in points)
        sum_y = sum((p[1] - y_mean) ** 2 for p in points)
        sum_xy = sum((p[0] - x_mean) * (p[1] - y_mean) for p in points)
        sum_x2y = sum((p[0] - x_mean) ** 2 * (p[1] - y_mean) for p in points)
        sum_xy2 = sum((p[0] - x_mean) * (p[1] - y_mean) ** 2 for p in points)
        sum_x3 = sum((p[0] - x_mean) ** 3 for p in points)
        sum_y3 = sum((p[1] - y_mean) ** 3 for p in points)
        
        # Matrix coefficients
        a = sum_x2y - sum_x * sum_xy / n
        b = sum_xy2 - sum_y * sum_xy / n
        c = sum_x3 + sum_xy2 - (sum_x * sum_x + sum_xy) / n * sum_x
        d = sum_y3 + sum_x2y - (sum_y * sum_y + sum_xy) / n * sum_y
        
        # Compute circle parameters
        # If the points are collinear or nearly so, determinant will be near zero
        # leading to unstable solutions, in which case we'll raise an exception
        det = a * d - b * c
        if abs(det) < 1e-10:
            raise ValueError("Points are collinear or near-collinear")
        
        # Circle center coordinates
        h = (d * sum_x - b * sum_y) / (2 * det) + x_mean
        k = (-c * sum_x + a * sum_y) / (2 * det) + y_mean
        
        # Compute radius as average distance from center to all points
        r = sum(math.sqrt((p[0] - h) ** 2 + (p[1] - k) ** 2) for p in points) / n
        
        return h, k, r


@dataclass
@register
class Trajectory(Path):
    """A timed, piecewise linear path."""
    times : List[float]
    velocities : Optional[List[float]] = None 

    def domain(self) -> Tuple[float,float]:
        """Returns the time parameter domain"""
        return (self.times[0],self.times[-1])

    def time_to_index(self, t : float) -> Tuple[int,float]:
        """Converts a time to an (edge index, edge parameter) tuple."""
        if len(self.points) < 2:
            return 0,0.0
        ind = 0
        while ind < len(self.times) and self.times[ind] < t:
            ind += 1
        if ind == 0: return 0,0.0
        if ind >= len(self.times): return len(self.points)-2,1.0
        u = (t - self.times[ind-1])/(self.times[ind] - self.times[ind-1])
        return ind-1,u
    
    def time_to_parameter(self, t : float) -> float:
        """Converts a time to a parameter."""
        ind,u = self.time_to_index(t)
        return ind+u
    
    def parameter_to_time(self, u : float) -> float:
        """Converts a parameter to a time"""
        if len(self.points) < 2:
            return self.times[0]
        ind = int(math.floor(u))
        if ind < 0: ind = 0
        if ind >= len(self.times)-1: ind = len(self.times)-2
        u = u - ind
        if u > 1: u = 1
        if u < 0: u = 0
        return self.times[ind] + u*(self.times[ind+1]-self.times[ind])

    def eval(self, t : float) -> List[float]:
        """Evaluates the trajectory at a given time."""
        if len(self.points) < 2:
            return self.points[0]
        ind,u = self.time_to_index(t)
        return transforms.vector_madd(self.points[ind],transforms.vector_sub(self.points[ind+1],self.points[ind]),u)

    def eval_derivative(self, t : float) -> List[float]:
        """Evaluates the derivative (velocity) at a given time."""
        if len(self.points) < 2:
            return transforms.vector_mul(self.points[0],0.0)
        ind,u = self.time_to_index(t)
        return transforms.vector_mul(transforms.vector_sub(self.points[ind+1],self.points[ind]),1.0/(self.times[ind+1]-self.times[ind]))

    def eval_tangent(self, t : float) -> List[float]:
        """Evaluates the tangent of the curve at a given time. This is related
        to the velocity but normalized; at cusp points the previous tangent (or
        next tangent, if the point is at the start) is returned. """
        if len(self.points) < 2:
            raise ValueError("Trajectory has no tangent: only 1 point")
        ind,u = self.time_to_index(t)
        d = transforms.vector_sub(self.points[ind+1],self.points[ind])
        l = transforms.vector_norm(d)
        pos = (ind == 0)
        while l == 0:
            if ind == 0:
                if not pos:
                    raise ValueError("Trajectory has no tangent: all points are coincident")
                ind+=1
            else:
                if pos:
                    raise ValueError("Trajectory has no tangent: all points are coincident")
                ind-=1
            d = transforms.vector_sub(self.points[ind+1],self.points[ind])
            l = transforms.vector_norm(d)
        return transforms.vector_mul(d,1.0/l)

    def closest_point(self, x : List[float], edges = True) -> Tuple[float,float]:
        """Returns the closest point on the path to the given point.  If
        edges=False, only computes the distances to the vertices, not the
        edges.  This is slightly faster but less accurate.
        
        Returns (distance, closest_time)
        """
        distance, closest_index = Path.closest_point(self,x,edges)
        closest_time = self.parameter_to_time(closest_index)
        return distance, closest_time

    def closest_point_local(self, x : List[float], time_range=Tuple[float,float], edges = True) -> Tuple[float,float]:
        """Returns the closest point on the path to the given point within
        the given time range.
        
        If edges=False, only computes the distances to the vertices, not the
        edges.  This is slightly faster but less accurate.
        
        Returns (distance, closest_time)
        """
        param_range = [self.time_to_parameter(time_range[0]),self.time_to_parameter(time_range[1])]
        #print("Searching within time range",time_range,"= param range",param_range)
        distance, closest_index = Path.closest_point_local(self,x,param_range,edges)
        closest_time = self.parameter_to_time(closest_index)
        return distance, closest_time
    
    def trim(self, start : float, end : float) -> Trajectory:
        """Returns a copy of this trajectory but trimmed to the given time range."""
        sind,su = self.time_to_index(start)
        eind,eu = self.time_to_index(end)
        s = self.eval(start)
        e = self.eval(end)
        return replace(self,points=[s]+self.points[sind+1:eind]+[e],times=[start]+self.times[sind+1:eind]+[end])




def compute_headings(path : Path, smoothed = False) -> Path:
    """Converts a 2D (x,y) path into a 3D path (x,y,heading) or a 3D
    (x,y,z) path into a 5D path (x,y,z,heading,pitch). 
    
    If smoothed=True, then the path is smoothed using a spline to better
    estimate good tangent vectors.
    """
    if smoothed:
        raise NotImplementedError("Smoothing not done yet")
    if len(path.points) < 2:
        raise ValueError("Path must have at least 2 points")
    derivs = []
    derivs.append(transforms.vector_sub(path.points[1],path.points[0]))
    for i in range(1,len(path.points)-1):
        derivs.append(transforms.vector_sub(path.points[i+1],path.points[i-1]))
    derivs[-1] = transforms.vector_sub(path.points[-1],path.points[-2])
    nd = len(path.points[0])-1
    coords = []
    for d in derivs:
        if transforms.vector2_dist(d,[0,0]) < 1e-6:
            coords.append([0]*nd)
        else:
            dunit = transforms.normalize_vector(d)
            if nd == 1:
                coords.append((transforms.vector2_angle(dunit,[1,0]),))
            elif nd == 2:
                azimuth = transforms.vector2_angle(dunit[:2],[1,0])
                elevation = transforms.vector2_angle((dunit[2],transforms.vector_norm(dunit[:2])),[0,0,1])
                coords.append((azimuth,elevation))
    return replace(path,points=[tuple(p)+tuple(c) for p,c in zip(path.points,coords)])
