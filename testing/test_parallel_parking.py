import sys
import os
sys.path.append(os.getcwd())

from GEMstack.state import Path, ObjectFrameEnum
from GEMstack.onboard.planning.hybrid_astar import Astar
import matplotlib.pyplot as plt
import numpy as np

def plot_parking_scenario(start_config, goal_config, path=None, obstacles=None):
    """
    Visualize the parking scenario including start position, goal position, obstacles, and path
    """
    plt.figure(figsize=(10, 6))
    
    # Plot start position
    plt.plot(start_config[0], start_config[1], 'go', markersize=10, label='Start')
    plt.arrow(start_config[0], start_config[1], 
             np.cos(start_config[2]), np.sin(start_config[2]), 
             head_width=0.1, color='g')
    
    # Plot goal position
    plt.plot(goal_config[0], goal_config[1], 'ro', markersize=10, label='Goal')
    plt.arrow(goal_config[0], goal_config[1], 
             np.cos(goal_config[2]), np.sin(goal_config[2]), 
             head_width=0.1, color='r')
    
    # Plot obstacles if provided
    if obstacles is not None:
        for obs in obstacles:
            plt.plot(obs[0], obs[1], 'ks', markersize=20)
    
    # Plot path if provided
    if path is not None:
        path = np.array(path)
        plt.plot(path[:, 0], path[:, 1], 'b-', label='Planned Path')
    
    plt.grid(True)
    plt.axis('equal')
    plt.legend()
    plt.title('Parallel Parking Scenario')
    plt.xlabel('X (meters)')
    plt.ylabel('Y (meters)')
    plt.show()

def euclidean_distance_heuristic(current, goal):
    """
    Calculate Euclidean distance between current and goal configurations
    Also considers heading difference as part of the heuristic
    """
    pos_distance = np.linalg.norm(np.array(current[:2]) - np.array(goal[:2]))
    angle_diff = abs(current[2] - goal[2])
    angle_diff = min(angle_diff, 2*np.pi - angle_diff)
    
    # Weight for combining position and angle differences
    angle_weight = 0.5
    return pos_distance + angle_weight * angle_diff

def test_parallel_parking():
    # Define start and goal configurations [x, y, heading]
    # Start: Vehicle is parallel to the road
    start_config = (0, 0, 0)  # (x=0, y=0, heading=0 radians)
    
    # Goal: Vehicle is in the parking spot
    goal_config = (-2, -2, -np.pi/2)  # (x=-2, y=-2, heading=-90 degrees)
    
    # Create obstacles representing parked cars
    obstacles = [
        (-2, 0),   # Car in front of parking spot
        (-2, -4)   # Car behind parking spot
    ]
    
    # Initialize planner
    planner = Astar(start_config, goal_config, euclidean_distance_heuristic)
    
    # Run the planner
    path = planner(start_config, goal_config)
    
    if path:
        print("Path found successfully!")
        print("Path points:", path)
        
        # Visualize the scenario and solution
        plot_parking_scenario(start_config, goal_config, path, obstacles)
    else:
        print("No valid path found!")

def main():
    print("Testing Hybrid A* Parallel Parking Planner...")
    test_parallel_parking()

if __name__ == '__main__':
    main() 