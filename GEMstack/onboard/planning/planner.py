import os
import argparse
import time
import matplotlib
import matplotlib.pyplot as plt

from .collision import build_collision_lookup
from .kinodynamic_rrt_star import OptimizedKinodynamicRRT



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


def optimized_kinodynamic_rrt_planning(start_world, goal_world, occupancy_grid, safety_margin=10, 
                                       vehicle_width=20, vehicle_length=45.0, step_size=1.0, max_iterations=100000):
    """
    Args:
        start_world: (x, y, theta) start position in world coordinates
        goal_world: (x, y, theta) goal position in world coordinates
        occupancy_grid: Binary occupancy grid (1=obstacle, 0=free)
        safety_margin: Safety margin in grid cells
        vehicle_width: Vehicle width in meters
        step_size: Step size for path sampling
        turning_radius: Minimum turning radius
        max_iterations: Maximum RRT iterations
        
    Returns:
        List of (x, y, theta) world coordinates for the full path
    """
    print(f"Starting optimized kinodynamic RRT planning on {occupancy_grid.shape} grid")
    t_start = time.time()
    
    # === STEP 1: Convert world coordinates to grid coordinates ===
    start_grid = start_world
    goal_grid = goal_world
    print(f"Start grid: {start_grid}")
    print(f"Goal grid: {goal_grid}")
    
    # === STEP 2: Build collision lookup ===
    t_lookup = time.time()
    collision_lookup = build_collision_lookup(occupancy_grid, safety_margin, vehicle_width)
    import matplotlib.pyplot as plt

    # Visualize collision lookup
    plt.figure(figsize=(10, 10))
    print(collision_lookup)
    plt.imshow(collision_lookup['collision_mask'], cmap="gray", origin="lower")
    plt.title("Collision Lookup Table")
    plt.xlabel("Grid X")
    plt.ylabel("Grid Y")
    # plt.colorbar(label="Collision Status")
    # plt.show()
    plt.savefig("collision_lookup.png")
    print(f"Built collision lookup in {time.time() - t_lookup:.2f}s")
    
    # === STEP 3: Run Optimized Kinodynamic RRT ===
    t_rrt_start = time.time()
    
    # Initialize planner with optimized parameters
    planner = OptimizedKinodynamicRRT(
        occupancy_grid=occupancy_grid,
        collision_lookup_table=collision_lookup,
        start_pose_tuple=start_grid,
        goal_pose_tuple=goal_grid,
        max_iterations=max_iterations,
        local_sampling_step_size=step_size,
        vehicle_width=vehicle_width,
        vehicle_length=vehicle_length
        # bidirectional=bidirectional
    )

    ## Testing Summoning RRT implementation
    # planner = BiRRT(start_grid, goal_grid, 1-occupancy_grid, [0,4384,0,4667])
    # grid_path = planner.search()
    t_rrt = time.time()
    grid_path = planner.plan(visualize_planning_output=True)
    # anim = planner.animate_sampling()   
    t_rrt_final = time.time() - t_rrt
    print(f"RRT planning completed in {t_rrt_final:.2f}s")
    
    if grid_path is None:
        print("Failed to find path")
        return None
    
    # print(f"Conversion completed in {time.time() - t_convert:.2f}s")
    print(f"Final path has {len(grid_path)} points")
    print(f"Total planning time: {time.time() - t_start:.2f}s")
    
    return grid_path
