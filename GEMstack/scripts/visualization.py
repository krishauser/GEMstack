import json
import os
import time
from klampt import vis
from klampt.model.trajectory import Trajectory
import matplotlib.pyplot as plt
from ..onboard.visualization.klampt_visualization import KlamptVisualization
from ..onboard.visualization.mpl_visualization import MPLVisualization

LOG_DIR = "logs"
BEHAVIOR_FILE = "behavior.json"

def select_log_folder():
    log_folders = [f for f in os.listdir(LOG_DIR) if os.path.isdir(os.path.join(LOG_DIR, f))]
    if not log_folders:
        print("\033[91mNo log folders found.\033[0m")
        return None

    print("\n\033[94mAvailable log folders:\033[0m")
    for idx, folder in enumerate(log_folders):
        print(f"{idx + 1}. {folder}")

    while True:
        try:
            choice = int(input("\n\033[93mEnter the number of the log folder to use:\033[0m ")) - 1
            if 0 <= choice < len(log_folders):
                return os.path.join(LOG_DIR, log_folders[choice])
            print("\033[91mInvalid selection. Please enter a valid number.\033[0m")
        except ValueError:
            print("\033[91mPlease enter a number.\033[0m")

def select_visualizer():
    print("\n\033[94mChoose a visualization method:\033[0m")
    print("1. Klampt (3D visualization)")
    print("2. MPL (Matplotlib 2D visualization)")

    while True:
        try:
            choice = int(input("\n\033[93mEnter 1 or 2:\033[0m "))
            if choice in [1, 2]:
                return choice
            print("\033[91mInvalid selection. Please enter 1 or 2.\033[0m")
        except ValueError:
            print("\033[91mPlease enter a valid number.\033[0m")

# Load and extract pedestrian and vehicle data
def extract_behavior_data(log_dir):
    behavior_path = os.path.join(log_dir, BEHAVIOR_FILE)
    if not os.path.exists(behavior_path):
        print(f"\033[91mError: {behavior_path} not found.\033[0m")
        return None, None, None, None, None, None, None, None

    with open(behavior_path, "r") as f:
        data = [json.loads(line) for line in f]

    pedestrian_paths = {}
    pedestrian_times = {}
    vehicle_path = []
    vehicle_times = []
    speeds = []
    accelerators = []
    brakes = []
    steering_angles = []

    for entry in data:
        time_stamp = entry.get("time", 0)

        # Extract pedestrian data
        if "agents" in entry:
            for agent_id, agent in entry["agents"].items():
                agent_data = agent.get("data", {})
                if agent_data.get("type") == 3:  # Type 3 = pedestrian
                    pose = agent_data.get("pose", {})
                    x, y = pose.get("x", 0), pose.get("y", 0)

                    if agent_id not in pedestrian_paths:
                        pedestrian_paths[agent_id] = []
                        pedestrian_times[agent_id] = []

                    pedestrian_paths[agent_id].append((x, y))
                    pedestrian_times[agent_id].append(time_stamp)

        # Extract vehicle data
        if "vehicle" in entry:
            vehicle_data = entry["vehicle"].get("data", {})
            pose = vehicle_data.get("pose", {})
            x, y = pose.get("x", 0), pose.get("y", 0)

            vehicle_path.append((x, y))
            vehicle_times.append(time_stamp)
            speeds.append(vehicle_data.get("v", 0))
            accelerators.append(vehicle_data.get("accelerator_pedal_position", 0))
            brakes.append(vehicle_data.get("brake_pedal_position", 0))
            steering_angles.append(vehicle_data.get("steering_wheel_angle", 0))

    return pedestrian_paths, pedestrian_times, vehicle_path, vehicle_times, speeds, accelerators, brakes, steering_angles

# Klampt 3D Visualization
def visualize_with_klampt(pedestrian_paths, pedestrian_times, vehicle_path):
    """Uses Klampt to visualize pedestrian and vehicle paths."""
    vis.init("PyQt6")
    vis.setWindowTitle("Pedestrian and Vehicle Path Visualization")

    klampt_vis = KlamptVisualization(vehicle_interface=None, rate=20.0)

    for agent_id, path in pedestrian_paths.items():
        if len(path) < 2:
            continue

        times = pedestrian_times[agent_id]
        path_3d = [[float(x), float(y), 0.0] for x, y in path]  

        trajectory = Trajectory(times, path_3d)
        vis.add(agent_id, trajectory, color=(0, 1, 0, 1), width=2)

    # if vehicle_path:
    #     vehicle_x, vehicle_y = zip(*vehicle_path)
    #     vehicle_tuples = [[float(x), float(y), 0.0] for x, y in zip(vehicle_x, vehicle_y)]
    #     vis.add("Vehicle Path", vehicle_tuples, color=(1, 0, 0, 1), width=2)

    klampt_vis.initialize()
    vis.show()

    while vis.shown():
        time.sleep(0.05)

    klampt_vis.cleanup()
    vis.kill()

# MPL 2D Visualization
def visualize_with_mpl(pedestrian_paths, pedestrian_times, vehicle_path, vehicle_data):
    vis = MPLVisualization(rate=10.0)
    vis.initialize()

    fig, axs = vis.fig, vis.axs
    axs[0].clear()
    axs[1].clear()

    # Left Plot: Pedestrian & Vehicle Trajectories
    axs[0].set_xlabel("X Position")
    axs[0].set_ylabel("Y Position")
    axs[0].set_title("Pedestrian & Vehicle Trajectories")

    for agent_id, path in pedestrian_paths.items():
        path_x, path_y = zip(*path)
        axs[0].plot(path_x, path_y, linestyle='-', marker='o', label=f"Pedestrian {agent_id}")

    # if vehicle_path:
    #     vehicle_x, vehicle_y = zip(*vehicle_path)
    #     axs[0].plot(vehicle_x, vehicle_y, linestyle='-', marker='s', color='red', label="Vehicle Path")

    axs[0].legend()

    # Right Plot: Vehicle Controls Over Time
    times, speeds, accelerators, brakes, steering_angles = vehicle_data
    axs[1].set_xlabel("Time (s)")
    axs[1].set_title("Vehicle Controls Over Time")

    axs[1].plot(times, speeds, label="Speed (m/s)", color="blue")
    axs[1].plot(times, accelerators, label="Accelerator (%)", color="green")
    axs[1].plot(times, brakes, label="Brake (%)", color="red")
    axs[1].plot(times, steering_angles, label="Steering Angle", color="purple")
    axs[1].legend()

    fig.canvas.draw_idle()
    fig.canvas.flush_events()
    plt.show(block=True)
    
    vis.cleanup()

# Main Execution
if __name__ == "__main__":
    visualizer_choice = select_visualizer()
    selected_log_folder = select_log_folder()

    if selected_log_folder:
        print(f"\n\033[94mLoading behavior data from:\033[0m {selected_log_folder}")
        data = extract_behavior_data(selected_log_folder)

        pedestrian_paths, pedestrian_times, vehicle_path, vehicle_times, speeds, accelerators, brakes, steering_angles = data
        vehicle_data = (vehicle_times, speeds, accelerators, brakes, steering_angles)

        if pedestrian_paths or vehicle_path:
            if visualizer_choice == 1:
                print("\033[92mUsing Klampt for visualization...\033[0m")
                visualize_with_klampt(pedestrian_paths, pedestrian_times, vehicle_path)
            else:
                print("\033[92mUsing MPL (Matplotlib) for visualization...\033[0m")
                visualize_with_mpl(pedestrian_paths, pedestrian_times, vehicle_path, vehicle_data)
        else:
            print("\033[91mNo pedestrian or vehicle data found.\033[0m")
