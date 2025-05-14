import math
import matplotlib.pyplot as plt
import matplotlib.animation as animation
# from map_utils import world_to_grid
from .map_utils import world_to_grid
import time
def visualize_path(occupancy_grid, path, metadata, start_world, goal_world, show_headings=True):
    """
    Visualize the planned path.
    
    Args:
        occupancy_grid: Binary occupancy grid (1=obstacle, 0=free)
        path: List of (x, y, theta) world coordinates
        metadata: Map metadata
        start_world: (x, y, theta) start position in world coordinates
        goal_world: (x, y, theta) goal position in world coordinates
        show_headings: Whether to show heading arrows along the path
    """
    plt.figure(figsize=(12, 12))
    
    # Show occupancy grid
    plt.imshow(occupancy_grid, cmap='gray')
    
    # Extract path points
    if path:
        grid_path = []
        for world_pt in path:
            grid_pt = world_to_grid(*world_pt, metadata)
            grid_path.append(grid_pt)
            
        xs = [p[0] for p in grid_path]
        ys = [p[1] for p in grid_path]
        
        # Plot path
        plt.plot(xs, ys, 'g-', linewidth=2)
        
        # Show heading arrows
        if show_headings:
            arrow_interval = max(1, len(path) // 20)  # Show ~20 arrows along path
            arrow_length = 10
            
            for i in range(0, len(grid_path), arrow_interval):
                x, y, theta = grid_path[i]
                dx = arrow_length * math.cos(theta)
                dy = arrow_length * math.sin(theta)
                plt.arrow(x, y, dx, dy, head_width=3, head_length=6, fc='blue', ec='blue')
    
    # Show start and goal
    start_grid = world_to_grid(*start_world, metadata)
    goal_grid = world_to_grid(*goal_world, metadata)
    
    plt.scatter(start_grid[0], start_grid[1], color='green', s=100, marker='o', label='Start')
    plt.scatter(goal_grid[0], goal_grid[1], color='red', s=100, marker='x', label='Goal')
    
    # Draw start and goal heading arrows
    dx_start = 15 * math.cos(start_grid[2])
    dy_start = 15 * math.sin(start_grid[2])
    plt.arrow(start_grid[0], start_grid[1], dx_start, dy_start, 
            head_width=5, head_length=10, fc='green', ec='green')
    
    dx_goal = 15 * math.cos(goal_grid[2])
    dy_goal = 15 * math.sin(goal_grid[2])
    plt.arrow(goal_grid[0], goal_grid[1], dx_goal, dy_goal, 
            head_width=5, head_length=10, fc='red', ec='red')
    
    plt.title('Path Planning Result')
    plt.legend()
    plt.tight_layout()
    #save the plt instead of showing it
    plt.savefig(f"path_planning_result_{time.time()}.png")
