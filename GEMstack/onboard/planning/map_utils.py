import numpy as np
import cv2
import yaml
import matplotlib.pyplot as plt
import math

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
    occupancy_grid = (image > 0).astype(np.uint8)
    
    # In ROS maps, occupied pixels are typically black (0) and free pixels are white (255)
    # If negate is True, we need to invert the grid
    if metadata.get('negate', 0):
        occupancy_grid = 1 - occupancy_grid
        
    return occupancy_grid, metadata

import numpy as np

def text_to_occupancy_grid(filename, grid_resolution=0.1, grid_margin=1.0):
    """
    Reads an obstacle file where each line is 'x y radius'.
    Returns:
      occupancy_grid: 2D numpy array with 1s for occupied, 0s otherwise
      x_min, y_min: to map world coords to grid indices
    """
    # Load obstacles
    obs = np.loadtxt(filename)
    xs, ys, rs = obs[:,0], obs[:,1], obs[:,2]

    # Determine bounds
    x_min, x_max = xs.min() - grid_margin, xs.max() + grid_margin
    y_min, y_max = ys.min() - grid_margin, ys.max() + grid_margin

    # Grid size
    w = int(np.ceil((x_max - x_min) / grid_resolution))
    h = int(np.ceil((y_max - y_min) / grid_resolution))
    grid = np.zeros((h, w), dtype=np.uint8)

    # Fill grid
    for x, y, r in zip(xs, ys, rs):
        ix = int((x - x_min) / grid_resolution)
        iy = int((y - y_min) / grid_resolution)
        r_cells = int(np.ceil(r / grid_resolution))
        for dy in range(-r_cells, r_cells+1):
            for dx in range(-r_cells, r_cells+1):
                xx, yy = ix + dx, iy + dy
                if 0 <= xx < w and 0 <= yy < h:
                    if np.hypot(dx, dy)*grid_resolution <= r:
                        grid[yy, xx] = 1

    start_grid = (10.0, 1.0, 0*math.pi/2)
    goal_grid = (10.0, 9.0, 0*math.pi/2)

    plt.figure(figsize=(8, 6))
    plt.scatter(start_grid[0], start_grid[1], color='green', s=100, marker='o', label='Start')
    plt.scatter(goal_grid[0], goal_grid[1], color='red', s=100, marker='x', label='Goal')
    plt.imshow(grid, cmap='gray',
                extent=[x_min, x_max, y_min, y_max])
    plt.title("Occupancy Grid")
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.grid(True)
    plt.show()

    return grid, x_min, y_min, x_max, y_max

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
