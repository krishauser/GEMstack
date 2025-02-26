import sys
import os
sys.path.append(os.getcwd())

from GEMstack.state import AgentEnum
import json
import matplotlib.pyplot as plt
import numpy as np

CMAP = "RdYlGn"

def compute_safety_factor(value, safe_thresh, unsafe_thresh, flip=False):
    """
    Computes a safety factor between 0(unsafe) and 1(safe)
    If flip is True, the threshold bounds are reversed.
    """
    abs_val = abs(value)
    if abs_val <= safe_thresh:
        factor = 1.0
    elif abs_val >= unsafe_thresh:
        factor = 0.0
    else:
        factor = 1.0 - (abs_val - safe_thresh) / (unsafe_thresh - safe_thresh)
    
    if flip:
        return 1.0 - factor
    return factor

def parse_behavior_log(filename):
    """
    Parses the behavior log file and extracts the following data:
    - vehicle time
    - vehicle acceleration
    - vehicle heading rate
    - pedestrian time
    - pedestrian distance
    """
    times = []
    accelerations = []
    heading_rates = []
    pedestrian_times = []
    pedestrian_distances = []
    
    with open(filename, 'r') as f:
        for line in f:
            try:
                entry = json.loads(line)
            except json.JSONDecodeError:
                print(f"Skipping invalid JSON line: {line.strip()}")
                continue
            # Process vehicle state data
            if "vehicle" in entry:
                t = entry.get("time")
                vehicle_data = entry["vehicle"].get("data", {})
                acceleration = vehicle_data.get("acceleration")
                heading_rate = vehicle_data.get("heading_rate")
                # Only add if all fields are available
                if None not in (t, acceleration, heading_rate):
                    times.append(t)
                    accelerations.append(acceleration)
                    heading_rates.append(heading_rate)
            # Process agent state data
            if "agents" in entry:
                for agent in entry["agents"].values():
                    agent_data = agent.get("data", {})
                    # Check if the agent is a pedestrian
                    if agent_data.get("type") == AgentEnum.PEDESTRIAN.value:
                        t = entry.get("time")
                        pose = agent_data.get("pose", {})
                        x_agent = pose.get("x")
                        y_agent = pose.get("y")
                        if None not in (t, x_agent, y_agent):
                            pedestrian_times.append(t)
                            dist = np.sqrt(x_agent**2 + y_agent**2)
                            pedestrian_distances.append(dist)
    
    return (np.array(times), np.array(accelerations), np.array(heading_rates), 
            np.array(pedestrian_times), np.array(pedestrian_distances))

def parse_tracker_csv(filename):
    """
    Parses the pure pursuit tracker log file and extracts the following data:
      - vehicle time (from column index 18)
      - Crosstrack error (from column index 20)
      - X position actual (from column index 2)
      - Y position actual (from column index 5)
      - X position desired (from column index 11)
      - Y position desired (from column index 14)
    """
    data = np.genfromtxt(filename, delimiter=',', skip_header=1)
    vehicle_time = data[:, 18]
    cte = data[:, 20]
    x_actual, y_actual = data[:, 2], data[:, 5]
    x_desired, y_desired = data[:, 11], data[:, 14]
    return vehicle_time, cte, x_actual, y_actual, x_desired, y_desired

def compute_derivative(times, values):
    """
    Computes the derivative of array with respect to time.
    Returns the time array and derivative values.
    """
    dt = np.diff(times)
    dv = np.diff(values)
    derivative = dv / dt
    return times[1:], derivative

def add_safety_colorbar(figure):
    """Adds a colorbar to the right of the figure"""
    cbar_ax = figure.add_axes([0.92, 0.2, 0.02, 0.6])
    sm = plt.cm.ScalarMappable(cmap=CMAP)
    cbar = figure.colorbar(sm, cax=cbar_ax)
    cbar.set_label("Comfort/Safety Level")

def plot_metrics(time_jerk, jerk, time_heading_acc, heading_acc, vehicle_time, cte, 
                 x_actual, y_actual, x_desired, y_desired, pedestrian_times, pedestrian_distances):
    """Plots all metrics in 2x3 grid"""
    fig, axs = plt.subplots(2, 3, figsize=(12, 8))
    fig.subplots_adjust(hspace=0.375, wspace=0.35)
    axs[1,2].axis('off')

    plot_jerk(axs[0,0], time_jerk, jerk)
    plot_heading_acceleration(axs[0,1], time_heading_acc, heading_acc)
    plot_crosstrack_error(axs[1,0], vehicle_time, cte)
    plot_position(axs[1,1], x_actual, y_actual, x_desired, y_desired)
    plot_pedestrian_dist(axs[0,2], pedestrian_times, pedestrian_distances)
    
    # Colorbar on the right side
    add_safety_colorbar(fig)
    
    plt.show()

def plot_jerk(axis, time, jerk, safe_thresh=1.0, unsafe_thresh=2.5):
    """Plots vehicle jerk (rate of acceleration) vs. time"""
    safety_scores = np.vectorize(compute_safety_factor)(jerk, safe_thresh, unsafe_thresh)
    
    axis.plot(time, jerk, color="black", linewidth=0.8, alpha=0.5)
    axis.scatter(time, jerk, c=safety_scores, cmap=CMAP, vmin=0, vmax=1, edgecolors="black")

    axis.set_xlabel("Time (s)")
    axis.set_ylabel("Jerk (m/s³)")
    axis.set_title("Vehicle Jerk Over Time")
    axis.grid(True)

def plot_heading_acceleration(axis, time, heading_acc, safe_thresh=0.0075, unsafe_thresh=0.25):
    """Plots vehicle heading acceleration vs. time"""
    safety_scores = np.vectorize(compute_safety_factor)(heading_acc, safe_thresh, unsafe_thresh)
    
    axis.plot(time, heading_acc, color="black", linewidth=0.8, alpha=0.5)
    axis.scatter(time, heading_acc, c=safety_scores, cmap=CMAP, vmin=0, vmax=1, edgecolors="black")
    
    axis.set_xlabel("Time (s)")
    axis.set_ylabel("Heading Acceleration (rad/s²)")
    axis.set_title("Vehicle Heading Acceleration Over Time")
    axis.grid(True)

def plot_crosstrack_error(axis, time, cte, safe_thresh=0.1, unsafe_thresh=0.4):
    """Plots vehicle cross track error vs. time"""
    safety_scores = np.vectorize(compute_safety_factor)(cte, safe_thresh, unsafe_thresh)
    
    axis.plot(time, cte, color="black", linewidth=0.8, alpha=0.5)
    axis.scatter(time, cte, c=safety_scores, cmap=CMAP, vmin=0, vmax=1, edgecolors="black")
    
    axis.set_xlabel("Time (s)")
    axis.set_ylabel("Cross Track Error")
    axis.set_title("Vehicle Cross Track Error Over Time")
    axis.grid(True)

def plot_position(axis, x_actual, y_actual, x_desired, y_desired, safe_thresh=1, unsafe_thresh=2.5):
    """Plots vehicle actual and desired positions vs. time"""
    position_error = np.sqrt((x_desired - x_actual) ** 2 + (y_desired - y_actual) ** 2)
    safety_scores = np.vectorize(compute_safety_factor)(position_error, safe_thresh, unsafe_thresh)
    
    axis.plot(y_desired, x_desired, marker='.', linestyle=':', color='blue', label='Desired')
    axis.plot(y_actual, x_actual, color="black", linewidth=0.8, alpha=0.5)
    axis.scatter(y_actual, x_actual, c=safety_scores, cmap=CMAP, vmin=0, vmax=1, edgecolors="black")
    
    axis.set_xlabel("Y Position (m)")
    axis.set_ylabel("X Position (m)")
    axis.set_title("Vehicle Position vs. Desired Position")
    axis.legend()
    axis.grid(True)

def plot_pedestrian_dist(axis, pedestrian_times, pedestrian_distances, safe_thresh=5.0, unsafe_thresh=2.0):
    """Plots pedestrian distance to vehicle vs. time"""
    if len(pedestrian_times) > 0:
        safety_scores = np.vectorize(compute_safety_factor)(pedestrian_distances, safe_thresh, unsafe_thresh, flip=True)
        axis.plot(pedestrian_times, pedestrian_distances, color="black", linewidth=0.8, alpha=0.5)
        axis.scatter(pedestrian_times, pedestrian_distances, c=safety_scores, cmap=CMAP, vmin=0, vmax=1, edgecolors="black")
    
    axis.set_xlabel("Time (s)")
    axis.set_ylabel("Pedestrian Distance (m)")
    axis.set_title("Pedestrian Distance Over Time")
    axis.grid(True)

if __name__=='__main__':
    if len(sys.argv) != 2:
        print("Usage: python test_comfort_metrics.py <log_directory>")
        sys.exit(1)
    
    log_dir = sys.argv[1]
    behavior_file = os.path.join(log_dir, "behavior.json")
    tracker_file = os.path.join(log_dir, "PurePursuitTrajectoryTracker_debug.csv")
    
    # if behavior.json doesn't exist, print error and exit
    if not os.path.exists(behavior_file):
        print("Error: behavior.json file is missing in log folder.")
        sys.exit(1)
    
    # Parse behavior log file and compute metrics
    times, accelerations, heading_rates, ped_times, ped_distances = parse_behavior_log(behavior_file)
    time_jerk, jerk = compute_derivative(times, accelerations)
    time_heading_acc, heading_acc = compute_derivative(times, heading_rates)
    
    # Pure pursuit tracker file exists: parse and plot all metrics
    if os.path.exists(tracker_file):
        vehicle_time, cte, x_actual, y_actual, x_desired, y_desired = parse_tracker_csv(tracker_file)
        plot_metrics(time_jerk, jerk, time_heading_acc, heading_acc, vehicle_time, cte, 
                     x_actual, y_actual, x_desired, y_desired, ped_times, ped_distances)
        
        print("RMS Cross Track Error:", np.sqrt(np.mean(cte**2)), "m")
        print("RMS Position Error:", np.sqrt(np.mean((x_actual - x_desired)**2 + (y_actual - y_desired)**2)), 'm')
    # Pure pursuit tracker file is missing: plot only behavior.json metrics
    else:
        print("Tracker file is missing. Skipping cross track error and vehicle position plots.")
        # Plot only jerk, heading acceleration, and pedestrian distance
        fig, axs = plt.subplots(1, 3, figsize=(12, 4))
        plot_jerk(axs[0], time_jerk, jerk)
        plot_heading_acceleration(axs[1], time_heading_acc, heading_acc)
        plot_pedestrian_dist(axs[2], ped_times, ped_distances)
        add_safety_colorbar(fig)
        plt.show()
        
    print("RMS Jerk:", np.sqrt(np.mean(jerk**2)), "m/s³")
    print("RMS Heading Acceleration:", np.sqrt(np.mean(heading_acc**2)), "rad/s²")
    if len(ped_distances) > 0:
        print("Minimum Pedestrian Distance:", np.min(ped_distances), "m")