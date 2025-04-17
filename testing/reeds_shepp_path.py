import ompl.base as ob
import ompl.geometric as og
import numpy as np
import math
import os
import sys
import logging
from contextlib import contextmanager

@contextmanager
def suppress_output():
    # Disable all logging
    logging.disable(logging.CRITICAL)
    
    # Save the original stdout and stderr
    original_stdout = sys.stdout
    original_stderr = sys.stderr
    
    # Open null device
    with open(os.devnull, 'w') as devnull:
        # Redirect stdout and stderr to null device
        sys.stdout = devnull
        sys.stderr = devnull
        
        # Also redirect file descriptors 1 and 2 (stdout and stderr)
        original_stdout_fd = os.dup(1)
        original_stderr_fd = os.dup(2)
        os.dup2(devnull.fileno(), 1)
        os.dup2(devnull.fileno(), 2)
        
        try:
            yield
        finally:
            # Restore original stdout and stderr
            sys.stdout = original_stdout
            sys.stderr = original_stderr
            
            # Restore file descriptors
            os.dup2(original_stdout_fd, 1)
            os.dup2(original_stderr_fd, 2)
            os.close(original_stdout_fd)
            os.close(original_stderr_fd)
            
            # Re-enable logging
            logging.disable(logging.NOTSET)

# Create space once and reuse
_space = ob.ReedsSheppStateSpace(0.25)  # Default turning radius
bounds = ob.RealVectorBounds(2)
bounds.setLow(-10)
bounds.setHigh(10)
_space.setBounds(bounds)

def path_length(start, goal, turning_radius):
    """
    Calculate the length of the Reeds-Shepp path between two configurations using OMPL.
    
    Args:
        start: (x, y, theta) tuple for start configuration
        goal: (x, y, theta) tuple for goal configuration
        turning_radius: minimum turning radius of the vehicle
        
    Returns:
        float: length of the shortest path
    """
    global _space
    
    if abs(turning_radius - 0.75) > 1e-6:
        # Only recreate space if turning radius changes significantly
        _space = ob.ReedsSheppStateSpace(turning_radius)
        _space.setBounds(bounds)
    
    # Create states
    start_state = ob.State(_space)
    start_state().setXY(start[0], start[1])
    start_state().setYaw(start[2])
    
    goal_state = ob.State(_space)
    goal_state().setXY(goal[0], goal[1])
    goal_state().setYaw(goal[2])
    
    # Calculate distance directly using Reeds-Shepp metric
    return _space.distance(start_state(), goal_state()) 