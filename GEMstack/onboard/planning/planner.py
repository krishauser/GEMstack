import os
import argparse
import time
import math
import matplotlib

from .collision import build_collision_lookup
from .map_utils import world_to_grid, grid_to_world
from .kinodynamic_rrt_star import OptimizedKinodynamicRRT  # Updated import
#from summon_rrt import BiRRT
from .map_utils import load_pgm_to_occupancy_grid
from .visualization import visualize_path, animate_path
# from test_dubins import run_tests

def optimized_kinodynamic_rrt_planning(start_world, goal_world, occupancy_grid,
                                safety_margin=2, vehicle_width=1.7, step_size=1.0,
                                turning_radius=3.0, max_iterations=3000):
    """
    Optimized Kinodynamic RRT planning from start to goal
    
    Args:
        start_world: (x, y, theta) start position in world coordinates
        goal_world: (x, y, theta) goal position in world coordinates
        occupancy_grid: Binary occupancy grid (1=obstacle, 0=free)
        metadata: Map metadata
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
    print(f"Built collision lookup in {time.time() - t_lookup:.2f}s")
    
    # === STEP 3: Run Optimized Kinodynamic RRT ===
    t_rrt_start = time.time()
    
    # Initialize planner with optimized parameters
    planner = OptimizedKinodynamicRRT(
        occupancy_grid=occupancy_grid,
        collision_lookup=collision_lookup,
        start=start_grid,
        goal=goal_grid,
        max_iter=max_iterations,
        step_size=step_size * 2.0,
        goal_sample_rate=0.25,
        turning_radius=turning_radius,
        vehicle_length=vehicle_width * 2.5
    )

    ## Testing Summoning RRT implementation
    # planner = BiRRT(start_grid, goal_grid, 1-occupancy_grid, [0,4384,0,4667])
    # grid_path = planner.search()
    
    grid_path = planner.plan(visualize_final=True)
    t_rrt = time.time() - t_rrt_start
    print(f"RRT planning completed in {t_rrt:.2f}s")
    
    if grid_path is None:
        print("Failed to find path")
        return None
    
    # === STEP 4: Convert path to world coordinates ===
    # t_convert = time.time()
    # world_path = []
    # for grid_point in grid_path:
    #     world_point = grid_to_world(grid_point[0], grid_point[1], grid_point[2], metadata)
    #     world_path.append(world_point)
    
    # print(f"Conversion completed in {time.time() - t_convert:.2f}s")
    # print(f"Final path has {len(world_path)} points")
    # print(f"Total planning time: {time.time() - t_start:.2f}s")
    
    return grid_path

def main():
    """Main function for running the planner."""
    parser = argparse.ArgumentParser(description="Optimized Kinodynamic RRT planner")
    parser.add_argument("--test", action="store_true", help="run unit tests and exit")
    parser.add_argument("--vis", action="store_true", help="show visualizations for tests or planning result")
    parser.add_argument("--animate", "-a", action="store_true", help="animate planned path")
    parser.add_argument("--max-iter", type=int, default=20000, help="maximum RRT iterations")
    parser.add_argument("--pad", type=int, default=100, help="crop padding (cells)")
    parser.add_argument("--safety", type=int, default=2, help="safety margin (cells)")
    parser.add_argument("--map-pgm", type=str, default="rrt_occupancy_map.pgm", help="path to PGM map file")
    parser.add_argument("--map-yaml", type=str, default="rrt_occupancy_map.yaml", help="path to YAML metadata file")
    parser.add_argument("--step-size", type=float, default=1.0, help="step size for path sampling")
    parser.add_argument("--turning-radius", type=float, default=1.0, help="minimum turning radius")
    parser.add_argument("--vehicle-width", type=float, default=1.7, help="vehicle width in meters")
    
    if "--vis" not in os.sys.argv and "--animate" not in os.sys.argv and "-a" not in os.sys.argv:
        matplotlib.use("Agg")
    
    args = parser.parse_args()
    
    if args.test:
        run_tests(show=args.vis)
        return
    
    if not (os.path.exists(args.map_pgm) and os.path.exists(args.map_yaml)):
        print(f"Map files not found: {args.map_pgm} and/or {args.map_yaml}")
        print("Use --test for CI runs or provide map files with --map-pgm and --map-yaml")
        return
    
    occupancy_grid, meta = load_pgm_to_occupancy_grid(args.map_pgm, args.map_yaml)
    
    start_w = (-15.0, -35.0, 2*math.pi/2)
    goal_w = (5.0, 45.0, 2*math.pi/2)
    # goal_w = (10.0, -25.0, 0*math.pi/2)
    
    path = optimized_kinodynamic_rrt_planning(
        start_w, goal_w, occupancy_grid, meta,
        safety_margin=args.safety,
        vehicle_width=args.vehicle_width,
        step_size=args.step_size,
        turning_radius=args.turning_radius,
        max_iterations=args.max_iter
    )
    
    if path:
        print(f"Planned path with {len(path)} poses")
        if args.vis:
            visualize_path(occupancy_grid, path, meta, start_w, goal_w)
        if args.animate:
            animate_path(occupancy_grid, path, meta, pad_cells=args.pad)
    else:
        print("Failed to find path")

if __name__ == "__main__":
    main()
