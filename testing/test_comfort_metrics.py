import json
import sys
import os
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors
import numpy as np

CMAP = "RdYlGn"

def compute_safety_factor(value, safe_thresh, unsafe_thresh):
    """Computes a safety factor between 0(unsafe) and 1(safe)"""
    abs_val = abs(value)
    if abs_val <= safe_thresh:
        return 1.0
    elif abs_val >= unsafe_thresh:
        return 0.0
    else:
        return 1.0 - (abs_val - safe_thresh) / (unsafe_thresh - safe_thresh)

def parse_behavior_log(filename):
    times = []
    accelerations = []
    heading_rates = []

    with open(filename, 'r') as f:
        for line in f:
            try:
                entry = json.loads(line)
            except json.JSONDecodeError:
                print(f"Skipping invalid JSON line: {line.strip()}")
                continue
            if "vehicle" in entry:
                t = entry.get("time")
                vehicle_data = entry["vehicle"].get("data", {})
                acceleration = vehicle_data.get("acceleration")
                heading_rate = vehicle_data.get("heading_rate")
                # Only add if all required fields are available
                if t is not None and acceleration is not None and heading_rate is not None:
                    times.append(t)
                    accelerations.append(acceleration)
                    heading_rates.append(heading_rate)
    
    return np.array(times), np.array(accelerations), np.array(heading_rates)

def parse_tracker_csv(filename):
    """
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

def plot_metrics(time_jerk, jerk, time_heading_acc, heading_acc, vehicle_time, cte, x_actual, y_actual, x_desired, y_desired):
    """Plots jerk, heading acceleration, and cross-track error in subplots."""
    fig, axs = plt.subplots(2, 2, figsize=(12, 8))
    fig.subplots_adjust(hspace=0.375, wspace=0.35)

    plot_jerk(axs[0,0], time_jerk, jerk)
    plot_heading_acceleration(axs[0,1], time_heading_acc, heading_acc)
    plot_crosstrack_error(axs[1,0], vehicle_time, cte)
    plot_position(axs[1,1], x_actual, y_actual, x_desired, y_desired)
    
    cbar_ax = fig.add_axes([0.92, 0.2, 0.02, 0.6])  # Position for the colorbar
    sm = plt.cm.ScalarMappable(cmap=CMAP)
    cbar = fig.colorbar(sm, cax=cbar_ax)
    cbar.set_label("Comfort/Safety Level")
    
    plt.show()

def compute_derivative(times, values):
    """
    Computes the derivative of array with respect to time.
    Returns the time array and derivative values.
    """
    dt = np.diff(times)
    dv = np.diff(values)
    derivative = dv / dt
    return times[1:], derivative

def plot_jerk(axis, time, jerk, safe_thresh=1.0, unsafe_thresh=2.5):
    """Plots vehicle jerk (rate of acceleration) vs. time"""
    safety_scores = np.vectorize(compute_safety_factor)(jerk, safe_thresh, unsafe_thresh)
    
    axis.plot(time, jerk, color="black", linewidth=0.8, alpha=0.5)
    scatter = axis.scatter(time, jerk, c=safety_scores, cmap=CMAP, vmin=0, vmax=1, edgecolors="black")

    axis.set_xlabel("Time (s)")
    axis.set_ylabel("Jerk (m/s³)")
    axis.set_title("Vehicle Jerk Over Time")
    axis.grid(True)
    # cbar = plt.colorbar(scatter, ax=axis)
    # cbar.set_label("Comfort Level")


def plot_heading_acceleration(axis, time, heading_acc, safe_thresh=0.0075, unsafe_thresh=0.25):
    """Plots vehicle heading acceleration vs. time"""
    safety_scores = np.vectorize(compute_safety_factor)(heading_acc, safe_thresh, unsafe_thresh)
    
    axis.plot(time, heading_acc, color="black", linewidth=0.8, alpha=0.5)
    scatter = axis.scatter(time, heading_acc, c=safety_scores, cmap=CMAP, vmin=0, vmax=1, edgecolors="black")
    
    axis.set_xlabel("Time (s)")
    axis.set_ylabel("Heading Acceleration (rad/s²)")
    axis.set_title("Vehicle Heading Acceleration Over Time")
    axis.grid(True)
    # cbar = plt.colorbar(scatter, ax=axis)
    # cbar.set_label("Comfort Level")

def plot_crosstrack_error(axis, time, cte, safe_thresh=0.1, unsafe_thresh=0.4):
    """Plots vehicle cross track error vs. time"""
    safety_scores = np.vectorize(compute_safety_factor)(cte, safe_thresh, unsafe_thresh)
    
    axis.plot(time, cte, color="black", linewidth=0.8, alpha=0.5)
    scatter = axis.scatter(time, cte, c=safety_scores, cmap=CMAP, vmin=0, vmax=1, edgecolors="black")
    
    axis.set_xlabel("Time (s)")
    axis.set_ylabel("Cross Track Error")
    axis.set_title("Vehicle Cross Track Error Over Time")
    axis.grid(True)
    # cbar = plt.colorbar(scatter, ax=axis)
    # cbar.set_label("Safety Level")

def plot_position(axis, x_actual, y_actual, x_desired, y_desired, safe_thresh=1, unsafe_thresh=2.5):
    """Plots vehicle actual and desired positions vs. time"""
    position_error = np.sqrt((x_desired - x_actual) ** 2 + (y_desired - y_actual) ** 2)
    safety_scores = np.vectorize(compute_safety_factor)(position_error, safe_thresh, unsafe_thresh)
    
    axis.plot(y_desired, x_desired, marker='.', linestyle=':', color='blue', label='Desired')
    axis.plot(y_actual, x_actual, color="black", linewidth=0.8, alpha=0.5)
    scatter = axis.scatter(y_actual, x_actual, c=safety_scores, cmap=CMAP, vmin=0, vmax=1, edgecolors="black")
    
    axis.set_xlabel("Y Position (m)")
    axis.set_ylabel("X Position (m)")
    axis.set_title("Vehicle Position vs. Desired Position")
    axis.legend()
    axis.grid(True)
    # cbar = plt.colorbar(scatter, ax=axis)
    # cbar.set_label("Safety Level")


if __name__=='__main__':
    if len(sys.argv) != 2:
        print("Usage: python test_comfort_metrics.py <log_directory>")
        sys.exit(1)
    
    log_dir = sys.argv[1]
    behavior_file = os.path.join(log_dir, "behavior.json")
    tracker_file = os.path.join(log_dir, "PurePursuitTrajectoryTracker_debug.csv")
    
    times, accelerations, heading_rates = parse_behavior_log(behavior_file)
    vehicle_time, cte, x_actual, y_actual, x_desired, y_desired = parse_tracker_csv(tracker_file)
    # calculate derivatives
    time_jerk, jerk = compute_derivative(times, accelerations)
    time_heading_acc, heading_acc = compute_derivative(times, heading_rates)
    
    plot_metrics(time_jerk, jerk, time_heading_acc, heading_acc, vehicle_time, cte, x_actual, y_actual, x_desired, y_desired)
    
    print("RMS Jerk:", np.sqrt(np.mean(jerk**2)), "m/s³")
    print("RMS Heading Acceleration:", np.sqrt(np.mean(heading_acc**2)), "rad/s²")
    print("RMS Cross Track Error:", np.sqrt(np.mean(cte**2)), "m")
    print("RMS Position Error:", np.sqrt(np.mean((x_actual - x_desired)**2 + (y_actual - y_desired)**2)), 'm')