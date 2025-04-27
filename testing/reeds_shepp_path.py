import numpy as np
import math

def normalize_angle(theta):
    """Normalize angle to [-pi, pi]"""
    return math.atan2(math.sin(theta), math.cos(theta))

def path_length(start, goal, turning_radius):
    """
    Calculate the length of the Reeds-Shepp path between two configurations.
    Implements a simplified version of Reeds-Shepp path planning that considers:
    - Straight line segments
    - Circular arcs (left and right turns)
    - Forward and backward motion
    
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
    dtheta = normalize_angle(theta2 - theta1)
    
    # Calculate Euclidean distance
    euclidean_dist = math.sqrt(dx*dx + dy*dy)
    
    # If points are very close, use a simple approximation
    if euclidean_dist < 0.1:
        return abs(dtheta) * turning_radius
    
    # Calculate the angle to the goal point
    alpha = math.atan2(dy, dx) - theta1
    alpha = normalize_angle(alpha)
    
    # Calculate the distance to the goal point
    d = euclidean_dist
    
    # Calculate the minimum turning radius needed
    R = turning_radius
    
    # Calculate the center of the turning circle
    if abs(alpha) < math.pi/2:
        # Forward motion
        xc = x1 + R * math.cos(theta1 + math.pi/2)
        yc = y1 + R * math.sin(theta1 + math.pi/2)
    else:
        # Backward motion
        xc = x1 - R * math.cos(theta1 + math.pi/2)
        yc = y1 - R * math.sin(theta1 + math.pi/2)
    
    # Calculate the angle to the goal point from the center
    beta = math.atan2(y2 - yc, x2 - xc)
    beta = normalize_angle(beta)
    
    # Calculate the arc length
    arc_length = abs(beta - alpha) * R
    
    # Calculate the straight line length
    straight_length = math.sqrt((x2 - xc)**2 + (y2 - yc)**2)
    
    # Calculate the final orientation difference
    final_angle_diff = abs(normalize_angle(theta2 - beta))
    
    # Total path length is the sum of:
    # 1. Arc length for the initial turn
    # 2. Straight line length
    # 3. Arc length for the final turn
    total_length = arc_length + straight_length + final_angle_diff * R
    
    return total_length 