import os
import argparse
import time
import matplotlib

from .collision import build_collision_lookup
from .map_utils import load_pgm_to_occupancy_grid
from .visualization import visualize_path
from .kinodynamic_rrt_star import OptimizedKinodynamicRRT

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

# Used for local testing
def main():
    """Main function for running the planner."""
    parser = argparse.ArgumentParser(description="Optimized Kinodynamic RRT planner")
    parser.add_argument("--vis", default=True, action="store_true", help="show visualizations for tests or planning result")
    parser.add_argument("--animate", "-a", action="store_true", help="animate planned path")
    parser.add_argument("--max-iter", type=int, default=10000, help="maximum RRT iterations")
    parser.add_argument("--pad", type=int, default=100, help="crop padding (cells)")
    parser.add_argument("--safety", type=int, default=2, help="safety margin (cells)")
    parser.add_argument("--map-pgm", type=str, default="occupancy_grid_after_>0.pgm", help="path to PGM map file")
    parser.add_argument("--step-size", type=float, default=100.0, help="step size for path sampling")
    parser.add_argument("--vehicle-width", type=float, default=20, help="vehicle width in meters")
    parser.add_argument("--vehicle-length", type=float, default=45, help="vehicle length in meters")
    
    if "--vis" not in os.sys.argv and "--animate" not in os.sys.argv and "-a" not in os.sys.argv:
        matplotlib.use("Agg")
    
    args = parser.parse_args()
    
    if not (os.path.exists(args.map_pgm)):
        print(f"Map files not found: {args.map_pgm}")
        print("Use --test for CI runs or provide map files with --map-pgm")
        return
    
    occupancy_grid = load_pgm_to_occupancy_grid(args.map_pgm)
    
    # start_w = (-200.0, -137.0, 0 * math.pi / 2)
    # goal_w = (-100.0, -137.0, 3*math.pi/2)
    start_w = (1200, 1025, 5.977692900114106)
    goal_w = (420, 1071, 4.713123114093702)
    # start_w = (-200.0, -137.0, 0 * math.pi / 2)
    # goal_w = (-200.0, -110.0, 2*math.pi/2)

    # start_w = (15.0, 2.0, 0*math.pi/2)
    # goal_w = (15.0, 11.0, 2*math.pi/2)
    # goal_w = (.0, 5.0, 1*math.pi/2)

    visualize_path(occupancy_grid, [], start_w, goal_w)
    
    path = optimized_kinodynamic_rrt_planning(
        start_w, goal_w, occupancy_grid,
        safety_margin=args.safety,
        vehicle_width=args.vehicle_width,
        step_size=args.step_size,
        max_iterations=args.max_iter,
    )
