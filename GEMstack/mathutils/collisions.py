"""Implements several collision detection primitives. 

Uses the `shapely` package for all 2D collision detection. 

The `CollisionDetector2D` class provides a convenience wrapper to store
multiple objects in a scene and perform collision queries. 

TODO: collision acceleration?
"""

from .transforms import vector2_dist
import shapely
import math
from typing import Tuple, List, Iterator, Optional

def point_in_circle_2d(point : Tuple[float,float], center : Tuple[float,float], radius : float) -> bool:
    """Returns true if the given point is inside a circle.
    """
    return (point[0]-center[0])**2 + (point[1]-center[1])**2 <= radius**2

def point_in_polygon_2d(point : Tuple[float,float], polygon : List[Tuple[float,float]]) -> bool:
    """Returns true if the given point is inside a polygon.

    Faster to create a CollisionDetector2D object if you will be doing
    multiple queries.
    """
    return shapely.Polygon(polygon).contains(shapely.Point(point[0], point[1]))

def point_circle_signed_distance_2d(point : Tuple[float,float], center : Tuple[float,float], radius : float) -> float:
    """Returns the signed distance from the given point to a circle.
    """
    return vector2_dist(point,center) - radius

def point_circle_distance_2d(point : Tuple[float,float], center : Tuple[float,float], radius : float) -> float:
    """Returns the distance from the given point to a circle.
    """
    return max(point_circle_signed_distance_2d(point,center,radius),0)

def point_line_distance_2d(point : Tuple[float,float], vertices : List[Tuple[float,float]]) -> float:
    """Returns distance from the given point to a line string.

    Faster to create a CollisionDetector2D object if you will be doing
    multiple queries.
    """
    return shapely.LineString(vertices).distance(shapely.Point(point[0], point[1]))

def point_polygon_distance_2d(point : Tuple[float,float], polygon : List[Tuple[float,float]]) -> float:
    """Returns distance from the given point to a polygon.

    Faster to create a CollisionDetector2D object if you will be doing
    multiple queries.
    """
    return shapely.Polygon(polygon).distance(shapely.Point(point[0], point[1]))

def circle_in_circle_2d(center1 : Tuple[float,float], radius1: float, center2 : Tuple[float,float], radius2: float) -> bool:
    """Returns whether a circle is contained within another circle.
    """
    d = vector2_dist(center1,center2)
    return d + radius1 <= radius2

def circle_in_polygon_2d(center : Tuple[float,float], radius: float, polygon : List[Tuple[float,float]]) -> bool:
    """Returns whether a circle is contained within a polygon.

    Faster to create a CollisionDetector2D object if you will be doing
    multiple queries.
    """
    return shapely.Polygon(polygon).contains(shapely.Point(center[0], center[1]).buffer(radius))

def circle_intersects_circle_2d(center1 : Tuple[float,float], radius1: float, center2 : Tuple[float,float], radius2: float) -> bool:
    """Returns whether a circle overlaps another circle.
    """
    d = vector2_dist(center1,center2)
    return d <= radius1 + radius2 

def circle_intersects_line_2d(center : Tuple[float,float], radius: float, vertices : List[Tuple[float,float]]) -> bool:
    """Returns whether a circle intersects a line string.

    Faster to create a CollisionDetector2D object if you will be doing
    multiple queries.
    """
    return shapely.LineString(vertices).intersects(shapely.Point(center[0], center[1]).buffer(radius))

def circle_intersects_polygon_2d(center : Tuple[float,float], radius: float, polygon : List[Tuple[float,float]]) -> bool:
    """Returns whether a circle intersects a polygon.

    Faster to create a CollisionDetector2D object if you will be doing
    multiple queries.
    """
    return shapely.Polygon(polygon).intersects(shapely.Point(center[0], center[1]).buffer(radius))

def circle_circle_signed_distance_2d(center1 : Tuple[float,float], radius1: float, center2 : Tuple[float,float], radius2: float) -> bool:
    """Returns signed distance between circles
    """
    return vector2_dist(center1,center2) - (radius1 + radius2)

def circle_circle_distance_2d(center1 : Tuple[float,float], radius1: float, center2 : Tuple[float,float], radius2: float) -> bool:
    """Returns distance between circles
    """
    return max(circle_circle_signed_distance_2d(center1,radius1,center2,radius2),0)

def circle_line_signed_distance_2d(center : Tuple[float,float], radius: float, vertices : List[Tuple[float,float]]) -> float:
    """Returns signed distance from circle to a line string.
    """
    return shapely.LineString(vertices).distance(shapely.Point(center[0], center[1])) - radius

def circle_line_distance_2d(center : Tuple[float,float], radius: float, vertices : List[Tuple[float,float]]) -> float:
    """Returns distance from the given circle to a line string.

    Faster to create a CollisionDetector2D object if you will be doing
    multiple queries.
    """
    return shapely.LineString(vertices).distance(shapely.Point(center[0], center[1]).buffer(radius))

def circle_polygon_distance_2d(center : Tuple[float,float], radius: float, polygon : List[Tuple[float,float]]) -> float:
    """Returns distance from the given point to a polygon.

    Faster to create a CollisionDetector2D object if you will be doing
    multiple queries.
    """
    return shapely.Polygon(polygon).distance(shapely.Point(center[0], center[1]).buffer(radius))

def line_intersects_line_2d(vertices1 : List[Tuple[float,float]], vertices2 : List[Tuple[float,float]]) -> bool:
    """Returns whether a line intersects a line.

    Faster to create a CollisionDetector2D object if you will be doing
    multiple queries.
    """
    return shapely.LineString(vertices1).intersects(shapely.LineString(vertices2))

def line_intersects_polygon_2d(vertices : List[Tuple[float,float]], polygon : List[Tuple[float,float]]) -> bool:
    """Returns whether a line intersects a polygon.

    Faster to create a CollisionDetector2D object if you will be doing
    multiple queries.
    """
    return shapely.Polygon(polygon).intersects(shapely.LineString(vertices))

def line_intersects_line_2d(vertices1 : List[Tuple[float,float]], vertices2 : List[Tuple[float,float]]) -> bool:
    """Returns distance from a line to a line

    Faster to create a CollisionDetector2D object if you will be doing
    multiple queries.
    """
    return shapely.LineString(vertices1).intersects(shapely.LineString(vertices2))

def line_polygon_distance_2d(vertices : List[Tuple[float,float]], polygon : List[Tuple[float,float]]) -> float:
    """Returns distance from a line to a polygon.

    Faster to create a CollisionDetector2D object if you will be doing
    multiple queries.
    """
    return shapely.Polygon(polygon).distance(shapely.LineString(vertices))

def polygon_intersects_polygon_2d(poly1 : List[Tuple[float,float]], poly2 : List[Tuple[float,float]]) -> bool:
    """Returns whether two polygons intersect.

    Faster to create a CollisionDetector2D object if you will be doing
    multiple queries.
    """
    return shapely.Polygon(poly1).intersects(shapely.Polygon(poly2))

def polygon_polygon_distance_2d(poly1 : List[Tuple[float,float]], poly2 : List[Tuple[float,float]]) -> float:
    """Returns the distance between two polygons.

    Faster to create a CollisionDetector2D object if you will be doing
    multiple queries.
    """
    return shapely.Polygon(poly1).distance(shapely.Polygon(poly2))


class CollisionDetector2D:
    """A class for detecting collisions between many types of objects.
    Simply provides a convenience wrapper around shapely objects.

    Does not implement any acceleration data structures, so this may be
    best combined with a grid for faster collision detection.
    """
    def __init__(self):
        self.objects = dict()
        self.collision_pairs = set()

    def remove(self, name : str) -> None:
        """Removes an object from the collision detector."""
        del self.objects[name]
        for name2 in self.objects.keys():
            if name < name2:
                 self.collision_pairs.discard((name,name2))
            else:
                 self.collision_pairs.discard((name2,name))
    
    def add_polygon(self, name : str, polygon : List[Tuple[float,float]]) -> None:
        """Adds an object to the collision detector."""
        if name in self.objects:
            raise ValueError("Object named "+name+" already exists")
        for name2 in self.objects.keys():
            if name < name2:
                 self.collision_pairs.add((name,name2))
            else:
                 self.collision_pairs.add((name2,name))
        self.objects[name] = shapely.Polygon(polygon)
    
    def add_circle(self, name : str, center : Tuple[float,float], radius : float) -> None:
        """Adds an object to the collision detector."""
        if name in self.objects:
            raise ValueError("Object named "+name+" already exists")
        for name2 in self.objects.keys():
            if name < name2:
                 self.collision_pairs.add((name,name2))
            else:
                 self.collision_pairs.add((name2,name))
        self.objects[name] = shapely.Point(center[0], center[1]).buffer(radius)
    
    def add_line(self, name : str, vertices : List[Tuple[float,float]]) -> None:
        """Adds an object to the collision detector."""
        if name in self.objects:
            raise ValueError("Object named "+name+" already exists")
        for name2 in self.objects.keys():
            if name < name2:
                 self.collision_pairs.add((name,name2))
            else:
                 self.collision_pairs.add((name2,name))
        self.objects[name] = shapely.LineString(vertices)
    
    def ignore_collisions(self, name1 : str, name2 : str) -> None:
        """Ignores collisions between two objects."""
        if name1 < name2:
            self.collision_pairs.discard((name1,name2))
        else:
            self.collision_pairs.discard((name2,name1))

    def distance(self, name1 : str, name2 : Optional[str] = None) -> float:
        """Returns the distance between two objects.  If name2 is None, then
        this returns the minimum distance from name1 to any other object."""
        if name2 is None:
            d = min(self.objects[name1].distance(self.objects[name2]) for name2 in self.collision_pairs[name1])
            for name2,olist in self.collision_pairs.items():
                if name1 in olist:
                    d = min(d,self.objects[name1].distance(self.objects[name2]))
            return d
        return self.objects[name1].distance(self.objects[name2])

    def items_colliding(self, name : Optional[str] = None) -> Iterator:
        """Returns an iterator over pairs of objects that are colliding.
        
        If `name` is provided, then this returns an iterator over objects
        that are colliding with the named object.
        """
        if name is None:
            for name1, name2 in self.collision_pairs:
                if self.objects[name1].intersects(self.objects[name2]):
                    yield (name1,name2)
        else:
            o = self.objects[name]
            for name2 in self.collision_pairs[name]:
                if o.intersects(self.objects[name2]):
                    yield (name,name2)
            for name2,olist in self.collision_pairs.items():
                if name in olist:
                    if o.intersects(self.objects[name2]):
                        yield (name2,name)
    
    def items_containing(self, point : Tuple[float,float]) -> Iterator:
        """Returns an iterator over objects that contain the given point."""
        pt = shapely.Point(point[0], point[1])
        for name,obj in self.objects.items():
            if obj.contains(pt):
                yield name
    
    def items_within_circle(self, center : Tuple[float,float], radius : float) -> Iterator:
        """Returns an iterator over objects that are within the given circle."""
        circle = shapely.Point(center[0], center[1]).buffer(radius)
        for name,obj in self.objects.items():
            if circle.contains(obj):
                yield name
    
    def items_within_box(self, lower : Tuple[float,float], upper : Tuple[float,float]) -> Iterator:
        """Returns an iterator over objects that are within the given box."""
        box = shapely.box(lower[0], lower[1], upper[0], upper[1])
        for name,obj in self.objects.items():
            if box.contains(obj):
                yield name