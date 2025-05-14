import math
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import time

def visualize_path(occupancy_grid, path, start_world, goal_world, show_headings=True):
    """
    Visualize the planned path.
    
    Args:
        occupancy_grid: Binary occupancy grid (1=obstacle, 0=free)
        path: List of (x, y, theta) world coordinates
        start_world: (x, y, theta) start position in world coordinates
        goal_world: (x, y, theta) goal position in world coordinates
        show_headings: Whether to show heading arrows along the path
    """
    plt.figure(figsize=(12, 12))
    
    # Show occupancy grid
    plt.imshow(occupancy_grid, cmap='gray' )
    
    # Extract path points
    if path:
            
        xs = [p[0] for p in path]
        ys = [p[1] for p in path]
        
        # Plot path
        plt.plot(xs, ys, 'g-', linewidth=2)
        
        # Show heading arrows
        if show_headings:
            arrow_interval = max(1, len(path) // 20)  # Show ~20 arrows along path
            arrow_length = 10
            
            for i in range(0, len(path), arrow_interval):
                x, y, theta = path[i]
                dx = arrow_length * math.cos(theta)
                dy = arrow_length * math.sin(theta)
                plt.arrow(x, y, dx, dy, head_width=3, head_length=6, fc='blue', ec='blue')
    
    # Show start and goal
    start_grid = start_world
    goal_grid = goal_world
    
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
    # plt.show()

def animate_path(occupancy_grid, path, interval=60, pad_cells=20,
                save="ani.gif", vehicle_len=10):
    """
    Animate the drive and crop axes to the path region.
    
    Args:
        occupancy_grid: Binary occupancy grid (1=obstacle, 0=free)
        path: List of (x, y, theta) world coordinates
        interval: Animation interval in milliseconds
        pad_cells: Extra cells to pad around trajectory bbox
        save: If provided, save animation to this file
        vehicle_len: Visual length of car icon (cells)
    
    Returns:
        Animation object
    """

    gpath = [pt for pt in path]
    xs = [p[0] for p in gpath]; ys = [p[1] for p in gpath]

    xmin, xmax = min(xs)-pad_cells, max(xs)+pad_cells
    ymin, ymax = min(ys)-pad_cells, max(ys)+pad_cells

    fig, ax = plt.subplots(figsize=((xmax-xmin)/50, (ymax-ymin)/50))
    ax.imshow(occupancy_grid.T, origin='lower', cmap='gray')
    ax.set_xlim(xmin, xmax); ax.set_ylim(ymin, ymax)
    ax.set_aspect('equal'); ax.set_xticks([]); ax.set_yticks([])

    # artists
    traj_line, = ax.plot([], [], 'g-', lw=2)
    car_line,  = ax.plot([], [], color='red', lw=3)

    def init():
        traj_line.set_data([], []); car_line.set_data([], [])
        return traj_line, car_line

    def update(i):
        traj_line.set_data(xs[:i+1], ys[:i+1])
        x, y, h = gpath[i]
        rear_x = x - vehicle_len*0.4*math.cos(h)
        rear_y = y - vehicle_len*0.4*math.sin(h)
        front_x = x + vehicle_len*0.6*math.cos(h)
        front_y = y + vehicle_len*0.6*math.sin(h)
        car_line.set_data([rear_x, front_x], [rear_y, front_y])
        return traj_line, car_line

    ani = animation.FuncAnimation(fig, update, frames=len(gpath), init_func=init,
                                interval=interval, blit=True, repeat=False)
    if save:  
        ani.save(save, dpi=150, fps=1000//interval)
    else:     
        plt.show()
    return ani