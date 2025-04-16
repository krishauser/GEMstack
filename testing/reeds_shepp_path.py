import numpy as np
import math

def path_length(start, goal, turning_radius):
    """
    Calculate the length of the Reeds-Shepp path between two configurations.
    
    Args:
        start: (x, y, theta) tuple for start configuration
        goal: (x, y, theta) tuple for goal configuration
        turning_radius: minimum turning radius of the vehicle
        
    Returns:
        float: length of the shortest path
    """
    x1, y1, theta1 = start
    x2, y2, theta2 = goal
    
    # Convert to relative coordinates
    dx = x2 - x1
    dy = y2 - y1
    dtheta = theta2 - theta1
    
    # Calculate Euclidean distance
    euclidean_dist = math.sqrt(dx*dx + dy*dy)
    
    # Calculate orientation difference
    orientation_diff = abs(dtheta)
    
    # Simple approximation: path length is at least the Euclidean distance
    # plus a penalty for orientation changes
    path_length = euclidean_dist + turning_radius * orientation_diff
    
    return path_length 