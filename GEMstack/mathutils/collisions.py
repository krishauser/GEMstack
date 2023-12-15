import shapely

def point_in_polygon_2d(point, polygon):
    """Returns true if the given point is inside the given polygon.

    Faster to create a CollisionDetector object if you will be doing
    multiple queries.
    """
    return shapely.Polygon(polygon).contains(shapely.Point(point[0], point[1]))
