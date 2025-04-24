import numpy as np
import cv2
import yaml

def load_pgm_to_occupancy_grid(pgm_path, yaml_path):
    """
    Load a PGM file and convert it to an occupancy grid.
    
    Args:
        pgm_path: Path to the PGM file
        yaml_path: Path to the YAML metadata file
        
    Returns:
        Tuple of (occupancy_grid, metadata)
    """
    # Load YAML metadata
    with open(yaml_path, 'r') as f:
        metadata = yaml.safe_load(f)
    
    # Load PGM file
    image = cv2.imread(pgm_path, cv2.IMREAD_GRAYSCALE)
    if image is None:
        raise FileNotFoundError(f"Could not load PGM file: {pgm_path}")
    
    # Use a threshold to create binary occupancy grid
    thresh = 128
    occupancy_grid = (image >= thresh).astype(np.uint8)
    
    # In ROS maps, occupied pixels are typically black (0) and free pixels are white (255)
    # If negate is True, we need to invert the grid
    if metadata.get('negate', 0):
        occupancy_grid = 1 - occupancy_grid
        
    return occupancy_grid, metadata

def downsample_occupancy_grid(grid, factor):
    """
    Downsample the grid by the given factor using max pooling to preserve obstacles.
    
    Args:
        grid: Original occupancy grid
        factor: Downsampling factor
        
    Returns:
        Downsampled grid
    """
    h, w = grid.shape
    nh, nw = h // factor, w // factor
    g = grid[:nh*factor, :nw*factor]
    return g.reshape(nh, factor, nw, factor).max(axis=(1,3))

def world_to_grid(x, y, theta, metadata):
    """
    Convert world coordinates to grid coordinates.
    
    Args:
        x, y, theta: World coordinates
        metadata: Map metadata
        
    Returns:
        Tuple of (grid_x, grid_y, grid_theta)
    """
    origin_x, origin_y, _ = metadata['origin']
    resolution = metadata['resolution']
    
    grid_x = int((x - origin_x) / resolution)
    grid_y = int((y - origin_y) / resolution)
    grid_theta = theta  # Heading remains the same
    
    return grid_x, grid_y, grid_theta

def grid_to_world(grid_x, grid_y, grid_theta, metadata):
    """
    Convert grid coordinates to world coordinates.
    
    Args:
        grid_x, grid_y, grid_theta: Grid coordinates
        metadata: Map metadata
        
    Returns:
        Tuple of (x, y, theta) in world coordinates
    """
    origin_x, origin_y, _ = metadata['origin']
    resolution = metadata['resolution']
    
    x = origin_x + grid_x * resolution
    y = origin_y + grid_y * resolution
    theta = grid_theta  # Heading remains the same
    
    return x, y, theta
