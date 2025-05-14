import numpy as np
from scipy.ndimage import distance_transform_edt

def build_collision_lookup(grid, safety_margin=2, vehicle_width=1.0):
    """
    Build a collision lookup table for efficient collision checking.
    
    Args:
        grid: Binary occupancy grid (1=obstacle, 0=free)
        safety_margin: Additional safety margin in grid cells
        vehicle_width: Width of the vehicle in grid cells
        
    Returns:
        Dictionary with distance transform, collision mask, and margin
    """
    dist = distance_transform_edt(1 - grid)
    margin = safety_margin + vehicle_width / 2.0
    return {
        "distance": dist,
        "collision_mask": dist <= margin,
        "margin": margin,
    }


def fast_collision_check(x, y, lookup):
    """
    Check if a point is in collision using the precomputed lookup table.
    
    Args:
        x, y: Coordinates to check
        lookup: Collision lookup table from build_collision_lookup
        
    Returns:
        True if in collision, False if free
    """
    xi, yi = int(round(x)), int(round(y))
    cm = lookup["collision_mask"]
    if xi < 0 or yi < 0 or xi >= cm.shape[0] or yi >= cm.shape[1]:
        return True  # Out of bounds is considered collision
    return cm[xi, yi]


def path_collision_check(path, lookup, x_off=0, y_off=0):
    """
    Check if a path is collision-free.
    
    Args:
        path: List of (x, y, theta) poses
        lookup: Collision lookup table
        x_off, y_off: Optional offsets to apply
        
    Returns:
        True if collision detected, False if path is free
    """
    for px, py, _ in path:
        if fast_collision_check(px + x_off, py + y_off, lookup):
            return True
    return False