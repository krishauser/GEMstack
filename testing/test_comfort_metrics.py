import sys
import os
sys.path.append(os.getcwd())

from GEMstack.state import AgentEnum
import json
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.collections import LineCollection
from matplotlib.colors import Normalize
from matplotlib import cm
import os

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
    
    # Avoid division by zero or very small values
    mask = np.abs(dt) > 1e-10
    derivative = np.zeros_like(dt)
    derivative[mask] = dv[mask] / dt[mask]
    
    return times[1:], derivative

def add_safety_colorbar(figure):
    """Adds a colorbar to the right of the figure"""
    cbar_ax = figure.add_axes([0.92, 0.2, 0.02, 0.6])
    sm = cm.ScalarMappable(cmap=CMAP)
    cbar = figure.colorbar(sm, cax=cbar_ax)
    cbar.set_label("Comfort/Safety Level")

def plot_metrics(time_jerk, jerk, time_heading_acc, heading_acc, time_accel, accel, vehicle_time, cte,
                 x_actual, y_actual, x_desired, y_desired, pedestrian_times, pedestrian_distances):
    """Plots all metrics in 2x3 grid"""
    fig, axs = plt.subplots(2, 3, figsize=(12, 8))
    fig.subplots_adjust(hspace=0.375, wspace=0.35)
    # axs[1,2].axis('off')

    plot_jerk(axs[0,0], time_jerk, jerk)
    plot_heading_acceleration(axs[0,1], time_heading_acc, heading_acc)
    plot_acceleration(axs[0,2], time_accel, accel)
    plot_crosstrack_error(axs[1,0], vehicle_time, cte)
    plot_position(axs[1,1], x_actual, y_actual, x_desired, y_desired)
    plot_pedestrian_dist(axs[1,2], pedestrian_times, pedestrian_distances)

    # Colorbar on the right side
    add_safety_colorbar(fig)
    plt.show()

def plot_jerk(axis, time, jerk, safe_thresh=1.0, unsafe_thresh=2.5):
    """
    Plots vehicle jerk (rate of acceleration) vs. time as a colored line.
    """
    # compute safety scores
    safety = np.vectorize(compute_safety_factor)(jerk, safe_thresh, unsafe_thresh)

    # build line segments
    points   = np.vstack([time, jerk]).T.reshape(-1,1,2)
    segments = np.concatenate([points[:-1], points[1:]], axis=1)

    # create colored LineCollection
    norm = Normalize(vmin=0, vmax=1)
    lc   = LineCollection(segments, cmap=CMAP, norm=norm, linewidth=1.5)
    lc.set_array(safety[:-1])
    axis.add_collection(lc)

    # Set axis limits safely, avoiding NaN/Inf values
    if len(jerk) > 0:
        valid_jerk = jerk[~np.isnan(jerk) & ~np.isinf(jerk)]
        if len(valid_jerk) > 0:
            ymin, ymax = valid_jerk.min(), valid_jerk.max()
            # Add small margin if min equals max
            if ymin == ymax:
                ymin -= 0.1
                ymax += 0.1
            axis.set_ylim(ymin, ymax)
    
    axis.set_xlim(time.min(), time.max())
    axis.set_xlabel("Time (s)")
    axis.set_ylabel("Jerk (m/s³)")
    axis.set_title("Vehicle Jerk Over Time")
    axis.grid(True)

def plot_acceleration(axis, time, acceleration, safe_thresh=0.5, unsafe_thresh=1.5):
    """
    Plots vehicle acceleration vs. time as a colored line.
    """
    safety = np.vectorize(compute_safety_factor)(acceleration, safe_thresh, unsafe_thresh)

    points   = np.vstack([time, acceleration]).T.reshape(-1,1,2)
    segments = np.concatenate([points[:-1], points[1:]], axis=1)

    norm = Normalize(vmin=0, vmax=1)
    lc   = LineCollection(segments, cmap=CMAP, norm=norm, linewidth=1.5)
    lc.set_array(safety[:-1])
    axis.add_collection(lc)

    # Set axis limits safely
    if len(acceleration) > 0:
        valid_accel = acceleration[~np.isnan(acceleration) & ~np.isinf(acceleration)]
        if len(valid_accel) > 0:
            ymin, ymax = valid_accel.min(), valid_accel.max()
            if ymin == ymax:
                ymin -= 0.1
                ymax += 0.1
            axis.set_ylim(ymin, ymax)
    
    axis.set_xlim(time.min(), time.max())
    axis.set_xlabel("Time (s)")
    axis.set_ylabel("Acceleration (m/s²)")
    axis.set_title("Vehicle Acceleration Over Time")
    axis.grid(True)

def plot_heading_acceleration(axis, time, heading_acc, safe_thresh=0.0075, unsafe_thresh=0.25):
    """
    Plots vehicle heading acceleration vs. time as a colored line.
    """
    safety = np.vectorize(compute_safety_factor)(heading_acc, safe_thresh, unsafe_thresh)

    points   = np.vstack([time, heading_acc]).T.reshape(-1,1,2)
    segments = np.concatenate([points[:-1], points[1:]], axis=1)

    norm = Normalize(vmin=0, vmax=1)
    lc   = LineCollection(segments, cmap=CMAP, norm=norm, linewidth=1.5)
    lc.set_array(safety[:-1])
    axis.add_collection(lc)

    # Set axis limits safely
    if len(heading_acc) > 0:
        valid_heading = heading_acc[~np.isnan(heading_acc) & ~np.isinf(heading_acc)]
        if len(valid_heading) > 0:
            ymin, ymax = valid_heading.min(), valid_heading.max()
            if ymin == ymax:
                ymin -= 0.1
                ymax += 0.1
            axis.set_ylim(ymin, ymax)
    
    axis.set_xlim(time.min(), time.max())
    axis.set_xlabel("Time (s)")
    axis.set_ylabel("Heading Acceleration (rad/s²)")
    axis.set_title("Vehicle Heading Acceleration Over Time")
    axis.grid(True)

def plot_crosstrack_error(axis, time, cte, safe_thresh=0.1, unsafe_thresh=0.4):
    """
    Plots vehicle cross track error vs. time as a colored line.
    """
    # compute safety scores for each point
    safety = np.vectorize(compute_safety_factor)(cte, safe_thresh, unsafe_thresh)

    points = np.vstack([time, cte]).T.reshape(-1,1,2)
    segments = np.concatenate([points[:-1], points[1:]], axis=1)

    lc = LineCollection(segments, cmap=CMAP, norm=Normalize(0,1))
    lc.set_array(safety[:-1])
    lc.set_linewidth(2.0)
    axis.add_collection(lc)

    # Set axis limits safely
    if len(cte) > 0:
        valid_cte = cte[~np.isnan(cte) & ~np.isinf(cte)]
        if len(valid_cte) > 0:
            ymin, ymax = valid_cte.min(), valid_cte.max()
            if ymin == ymax:
                ymin -= 0.1
                ymax += 0.1
            axis.set_ylim(ymin, ymax)
    
    axis.set_xlim(time.min(), time.max())
    axis.set_xlabel("Time (s)")
    axis.set_ylabel("Cross Track Error")
    axis.set_title("Vehicle Cross Track Error Over Time")
    axis.grid(True)

def plot_position(axis, x_actual, y_actual, x_desired, y_desired,
                  safe_thresh=1.0, unsafe_thresh=2.5):
    """
    Plots vehicle actual and desired positions
    """
    # compute positional error and safety at each point
    pos_error = np.sqrt((x_desired - x_actual)**2 + (y_desired - y_actual)**2)
    safety = np.vectorize(compute_safety_factor)(pos_error, safe_thresh, unsafe_thresh)

    # actual path segments
    actual_pts = np.vstack([y_actual, x_actual]).T.reshape(-1,1,2)
    actual_segs = np.concatenate([actual_pts[:-1], actual_pts[1:]], axis=1)
    norm = Normalize(vmin=0, vmax=1)
    lc_actual = LineCollection(actual_segs, cmap=CMAP, norm=norm)
    lc_actual.set_array(safety[:-1])
    lc_actual.set_linewidth(2.0)
    axis.add_collection(lc_actual)

    # desired path as dashed gray line
    axis.plot(y_desired, x_desired,
              linestyle='--', linewidth=1.5, color='gray', label='Desired')

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
    if len(sys.argv) == 2:
       log_dir = "logs/" + sys.argv[1]
    else:

        logs_root = "logs/"
        # Get all subdirectories inside "logs/"
        subdirs = [os.path.join(logs_root, d) for d in os.listdir(logs_root) if os.path.isdir(os.path.join(logs_root, d))]

        # Find the latest directory based on modification time
        log_dir = max(subdirs, key=os.path.getmtime)

    print(f"Using latest log directory: {log_dir}")
    behavior_file = os.path.join(log_dir, "behavior.json")
    tracker_file = os.path.join(log_dir, "PurePursuitTrajectoryTracker_debug.csv")

    # if behavior.json doesn't exist, print error and exit
    if not os.path.exists(behavior_file):
        print("Error: behavior.json file is missing in log folder.")
        sys.exit(1)

    # Parse behavior log file and compute metrics
    times, accelerations, heading_rates, ped_times, ped_distances = parse_behavior_log(behavior_file)
    
    # Ensure we have valid data before computing derivatives
    if len(times) > 1 and len(accelerations) > 1 and len(heading_rates) > 1:
        time_jerk, jerk = compute_derivative(times, accelerations)
        time_heading_acc, heading_acc = compute_derivative(times, heading_rates)
    else:
        print("Warning: Not enough data points to compute derivatives.")
        time_jerk, jerk = np.array([]), np.array([])
        time_heading_acc, heading_acc = np.array([]), np.array([])

    # Pure pursuit tracker file exists: parse and plot all metrics
    if os.path.exists(tracker_file):
        vehicle_time, cte, x_actual, y_actual, x_desired, y_desired = parse_tracker_csv(tracker_file)
        plot_metrics(time_jerk, jerk, time_heading_acc, heading_acc, times, accelerations, vehicle_time, cte,
                     x_actual, y_actual, x_desired, y_desired, ped_times, ped_distances)

        # Calculate RMS values only for valid data points
        valid_cte = cte[~np.isnan(cte) & ~np.isinf(cte)]
        if len(valid_cte) > 0:
            print("RMS Cross Track Error:", np.sqrt(np.mean(valid_cte**2)), "m")
        
        valid_pos_error = np.sqrt((x_actual - x_desired)**2 + (y_actual - y_desired)**2)
        valid_pos_error = valid_pos_error[~np.isnan(valid_pos_error) & ~np.isinf(valid_pos_error)]
        if len(valid_pos_error) > 0:
            print("RMS Position Error:", np.sqrt(np.mean(valid_pos_error**2)), 'm')
    # Pure pursuit tracker file is missing: plot only behavior.json metrics
    else:
        print("Tracker file is missing. Skipping cross track error and vehicle position plots.")
        # Plot only jerk, heading acceleration, and pedestrian distance
        fig, axs = plt.subplots(2, 2, figsize=(12, 4))
        plot_jerk(axs[0, 0], time_jerk, jerk)
        plot_heading_acceleration(axs[0, 1], time_heading_acc, heading_acc)
        plot_acceleration(axs[1, 0], times, accelerations)
        plot_pedestrian_dist(axs[1, 1], ped_times, ped_distances)

        add_safety_colorbar(fig)
        plt.show()

    # Calculate RMS values only for valid data points
    valid_jerk = jerk[~np.isnan(jerk) & ~np.isinf(jerk)]
    if len(valid_jerk) > 0:
        print("RMS Jerk:", np.sqrt(np.mean(valid_jerk**2)), "m/s³")
    
    valid_heading_acc = heading_acc[~np.isnan(heading_acc) & ~np.isinf(heading_acc)]
    if len(valid_heading_acc) > 0:
        print("RMS Heading Acceleration:", np.sqrt(np.mean(valid_heading_acc**2)), "rad/s²")
    
    if len(ped_distances) > 0:
        print("Minimum Pedestrian Distance:", np.min(ped_distances), "m")
