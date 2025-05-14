import numpy as np
import cv2

def load_pgm_to_occupancy_grid(pgm_path):
    """
    Load a PGM file and convert it to an occupancy grid.
    
    Args:
        pgm_path: Path to the PGM file
        
    Returns:
        2D Array occupancy_grid
    """
    # Load PGM file
    image = cv2.imread(pgm_path, cv2.IMREAD_GRAYSCALE)
    if image is None:
        raise FileNotFoundError(f"Could not load PGM file: {pgm_path}")
    
    # Use a threshold to create binary occupancy grid
    thresh = 128
    occupancy_grid = (image >= thresh).astype(np.uint8)
        
    return occupancy_grid

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