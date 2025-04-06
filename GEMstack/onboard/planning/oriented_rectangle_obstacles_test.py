import numpy as np
from shapely.geometry import Polygon
from typing import List, Tuple

Pose   = Tuple[float, float, float]         # (x, y, yaw)
Dims   = Tuple[float, float]                # (width, length)
Obstacle = Tuple[float, float, float, Dims] # (x, y, yaw, (width, length))

def rectangle_polygon(center: Pose, dims: Dims) -> Polygon:
    """
    Build a shapely Polygon for an oriented rectangle.
    
    Args:
        center: (x, y, yaw) of rectangle center in world frame.
        dims:   (width, length) of rectangle.
        
    Returns:
        shapely Polygon of the 4 corners, in CCW order.
    """
    x, y, yaw = center
    w, L      = dims
    # corners in vehicle frame (forward is +x, left is +y)
    corners = np.array([
        [ +L/2, +w/2],
        [ +L/2, -w/2],
        [ -L/2, -w/2],
        [ -L/2, +w/2],
    ])
    # rotation matrix
    R = np.array([[np.cos(yaw), -np.sin(yaw)],
                  [np.sin(yaw),  np.cos(yaw)]])
    # rotate & translate
    world_corners = (R @ corners.T).T + np.array([x, y])
    return Polygon(world_corners)

def is_trajectory_collision_free(
    trajectory: List[Pose],
    vehicle_dims: Dims,
    obstacles: List[Obstacle]
) -> bool:
    """
    Check whether following `trajectory` will avoid all `obstacles`.
    
    Args:
        trajectory: List of (x, y, yaw) vehicle poses along planned path.
        vehicle_dims: (width, length) of your vehicle rectangle.
        obstacles: List of static obstacles, each as (x, y, yaw, (width, length)).
        
    Returns:
        True if no pose along the trajectory intersects any obstacle; False otherwise.
    """
    # Pre-build obstacle polygons once
    obstacle_polys = [
        rectangle_polygon((ox, oy, oyaw), dims)
        for ox, oy, oyaw, dims in obstacles
    ]
    
    # For each pose along the trajectory, test intersection
    for pose in trajectory:
        veh_poly = rectangle_polygon(pose, vehicle_dims)
        for obs_poly in obstacle_polys:
            if veh_poly.intersects(obs_poly):
                # collision detected
                return False
    # no collisions anywhere
    return True

# Example usage:
if __name__ == "__main__":
    # vehicle is 1.0 m wide, 2.0 m long
    vehicle_dims = (1.0, 2.0)
    
    # define a straight-line trajectory: from x=0→5, heading along +x
    trajectory = [(x, 0.0, 0.0) for x in np.linspace(0, 5, num=50)]
    print(  "Trajectory:", trajectory)
    
    # two static obstacles
    obstacles = [
        # centered at (2.5, 0.0), aligned with world frame, size 0.5×0.5 m
        (2.5, 0.0, 0.0, (0.5, 0.5)),
        # another at (4.0, 1.0), rotated 45°
        (4.0, 1.0, np.pi/4, (1.0, 0.5)),
    ]
    
    ok = is_trajectory_collision_free(trajectory, vehicle_dims, obstacles)
    print("Collision‐free?" , ok)
