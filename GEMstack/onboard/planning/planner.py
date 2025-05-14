import os
import argparse
import time
import matplotlib

from .collision import build_collision_lookup
from .kinodynamic_rrt_star import OptimizedKinodynamicRRT

DEBUG = False

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
    if DEBUG:
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
    plt.savefig("collision_lookup.png")
    if DEBUG:
        print(f"Built collision lookup in {time.time() - t_lookup:.2f}s")
    
    # Initialize planner with optimized parameters
    planner = OptimizedKinodynamicRRT(
        occupancy_grid=occupancy_grid,
        collision_lookup_table=collision_lookup,
        start_pose_tuple=start_grid,
        goal_pose_tuple=goal_grid,
        max_iterations=max_iterations,
        local_sampling_step_size=step_size,
        vehicle_width=vehicle_width,
        vehicle_length=vehicle_length,
    )

    t_rrt = time.time()
    grid_path = planner.plan(visualize_planning_output=True)
    t_rrt_final = time.time() - t_rrt
    if DEBUG:
        print(f"RRT planning completed in {t_rrt_final:.2f}s")
    
    if grid_path is None:
        print("Failed to find path")
        return None
    
    if DEBUG:
        print(f"Final path has {len(grid_path)} points")
        print(f"Total planning time: {time.time() - t_start:.2f}s")
    
    return grid_path
