#needed to import GEMstack from top level directory
import sys
import os
sys.path.append(os.getcwd())

from GEMstack.state import Path, ObjectFrameEnum
from GEMstack.onboard.planning.hybrid_astar import Astar
import matplotlib.pyplot as plt
import numpy as np

def plot_path(start_config, goal_config, path=None):
    """Helper function to visualize the path"""
    plt.figure(figsize=(8, 8))
    
    # Plot start and goal
    plt.plot(start_config[0], start_config[1], 'go', markersize=10, label='Start')
    plt.plot(goal_config[0], goal_config[1], 'ro', markersize=10, label='Goal')
    
    # Plot path if available
    if path:
        path = np.array(path)
        plt.plot(path[:, 0], path[:, 1], 'b.-', label='Path')
    
    plt.grid(True)
    plt.axis('equal')
    plt.legend()
    plt.title('A* Path Planning Test')
    plt.xlabel('X (meters)')
    plt.ylabel('Y (meters)')
    plt.show()

def euclidean_heuristic(current, goal):
    """Simple Euclidean distance heuristic"""
    return np.linalg.norm(np.array(current) - np.array(goal))

def test_straight_line():
    """Test 1: Simple straight line path"""
    print("\nTest 1: Straight Line Path")
    start = (0, 0, 0)
    goal = (5, 0, 0)
    
    planner = Astar(start, goal, euclidean_heuristic)
    path = planner(start, goal)
    
    assert path is not False, "Planner failed to find path"
    assert len(path) > 0, "Path is empty"
    
    print("Path found:", path)
    plot_path(start, goal, path)
    return path

def test_diagonal_path():
    """Test 2: Diagonal path"""
    print("\nTest 2: Diagonal Path")
    start = (0, 0, 0)
    goal = (3, 3, 0)
    
    planner = Astar(start, goal, euclidean_heuristic)
    path = planner(start, goal)
    
    assert path is not False, "Planner failed to find path"
    assert len(path) > 0, "Path is empty"
    
    print("Path found:", path)
    plot_path(start, goal, path)
    return path

def test_path_length():
    """Test 3: Verify path length is reasonable"""
    print("\nTest 3: Path Length Test")
    start = (0, 0, 0)
    goal = (2, 2, 0)
    
    planner = Astar(start, goal, euclidean_heuristic)
    path = planner(start, goal)
    
    # Check if path exists
    assert path is not False, "Planner failed to find path"
    
    # Check if path length is reasonable
    path_length = 0
    for i in range(len(path)-1):
        path_length += euclidean_heuristic(path[i], path[i+1])
    
    # Path length should be close to the direct distance
    direct_distance = euclidean_heuristic(start, goal)
    assert path_length <= direct_distance * 1.5, "Path is too long"
    
    print(f"Path length: {path_length:.2f}")
    print(f"Direct distance: {direct_distance:.2f}")
    plot_path(start, goal, path)
    return path

def test_start_goal_same():
    """Test 4: Start and goal at same position"""
    print("\nTest 4: Start and Goal Same Position")
    start = (1, 1, 0)
    goal = (1, 1, 0)
    
    planner = Astar(start, goal, euclidean_heuristic)
    path = planner(start, goal)
    
    assert path is not False, "Planner failed to find path"
    assert len(path) > 0, "Path is empty"
    assert path[0] == start and path[-1] == goal, "Path should contain at least start/goal"
    
    print("Path found:", path)
    plot_path(start, goal, path)
    return path

def run_all_tests():
    """Run all test cases"""
    try:
        test_straight_line()
        test_diagonal_path()
        test_path_length()
        test_start_goal_same()
        print("\nAll tests passed successfully!")
    except AssertionError as e:
        print(f"\nTest failed: {str(e)}")
    except Exception as e:
        print(f"\nUnexpected error: {str(e)}")

if __name__ == '__main__':
    run_all_tests()