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

    def to_frame(self, frame : ObjectFrameEnum, current_pose : None, start_pose_abs : None) -> Path:
        """Converts the route to a different frame."""
        new_points = [convert_point(p,self.frame,frame,current_pose,start_pose_abs) for p in self.points]
        return replace(self,frame=frame,points=new_points)

    def eval(self, u : float) -> List[float]:
        """Evaluates the path at a given parameter.  The integer part of u
        indicates the segment, and the fractional part indicates the progress
        along the segment"""
        ind = int(u)
        if ind < 0: ind = 0
        if ind >= len(self.points)-1: ind = len(self.points)-2
        u = u - ind
        if u > 1: u = 1
        if u < 0: u = 0
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

    def arc_length_parameterize(self, speed = 1.0) -> Trajectory:
        """Returns a new path that is parameterized by arc length."""
        times = [0]
        for i in range(len(self.points)-1):
            p1 = self.points[i]
            p2 = self.points[i+1]
            times.append(times[-1] + transforms.vector_dist(p1,p2)/speed)
        return Trajectory(frame=self.frame,points=self.points,times=times)

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

@dataclass
@register
class Trajectory(Path):
    """A timed, piecewise linear path."""
    times : List[float]

    def time_to_index(self, t : float) -> Tuple[int,float]:
        """Converts a time to an (edge index, parameter) tuple."""
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
        ind = int(math.floor(u))
        if ind < 0: ind = 0
        if ind >= len(self.times)-1: ind = len(self.times)-2
        u = u - ind
        if u > 1: u = 1
        if u < 0: u = 0
        return self.times[ind] + u*(self.times[ind+1]-self.times[ind])

    def eval(self, t : float) -> List[float]:
        """Evaluates the trajectory at a given time."""
        ind,u = self.time_to_index(t)
        return transforms.vector_madd(self.points[ind],transforms.vector_sub(self.points[ind+1],self.points[ind]),u)

    def eval_derivative(self, t : float) -> List[float]:
        """Evaluates the derivative at a given time."""
        ind,u = self.time_to_index(t)
        return transforms.vector_sub(self.points[ind+1],self.points[ind])
        
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
        print("Searching within range",param_range)
        distance, closest_index = Path.closest_point_local(self,x,param_range,edges)
        closest_time = self.parameter_to_time(closest_index)
        return distance, closest_time



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
