import json
import sys
import os
import matplotlib.pyplot as plt
import numpy as np

# Safety thresholds (not used in the current plots)
ACC_SAFE = 0.8    # |acceleration| <= 0.8 m/s² is safe 
ACC_UNSAFE = 2.0  # |acceleration| >= 2.0 m/s² is unsafe
HR_SAFE = 0.05    # |heading_rate| <= 0.05 rad/s is safe 
HR_UNSAFE = 0.2   # |heading_rate| >= 0.2 rad/s is unsafe

def compute_safety_factor(value, safe_thresh, unsafe_thresh):
    """Computes a safety factor between 0 and 1 (for coloring in plots)"""
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
      - Crosstrack error time (from column index 18)
      - Crosstrack error (from column index 20)
      - X position actual (from column index 2)
      - Y position actual (from column index 5)
      - X position desired (from column index 11)
      - Y position desired (from column index 14)
    """
    data = np.genfromtxt(filename, delimiter=',', skip_header=1)
    cte_time = data[:, 18]
    cte = data[:, 20]
    x_actual, y_actual = data[:, 2], data[:, 5]
    x_desired, y_desired = data[:, 11], data[:, 14]
    return cte_time, cte, x_actual, y_actual, x_desired, y_desired


def plot_metrics(time_jerk, jerk, time_heading_acc, heading_acc, cte_time, cte, x_actual, y_actual, x_desired, y_desired):
    """Plots jerk, heading acceleration, and cross-track error in subplots."""
    fig, axs = plt.subplots(2, 2, figsize=(12, 8))
    fig.subplots_adjust(hspace=0.4, wspace=0.3)

    # Jerk plot
    axs[0,0].plot(time_jerk, jerk, marker='o', linestyle='-', color='blue')
    axs[0,0].set_ylabel("Jerk (m/s³)")
    axs[0,0].set_title("Vehicle Jerk Over Time")
    axs[0,0].grid(True)

    # Heading acceleration plot
    axs[0,1].plot(time_heading_acc, heading_acc, marker='x', linestyle='-', color='orange')
    axs[0,1].set_ylabel("Heading Acceleration (rad/s²)")
    axs[0,1].set_title("Vehicle Heading Acceleration Over Time")
    axs[0,1].grid(True)

    # Cross track error plot
    axs[1,0].plot(cte_time, cte, marker='s', linestyle='-', color='green')
    axs[1,0].set_xlabel("Time (s)")
    axs[1,0].set_ylabel("Cross Track Error")
    axs[1,0].set_title("Vehicle Cross Track Error Over Time")
    axs[1,0].grid(True)
    
    # Position plot
    axs[1,1].plot(x_actual, y_actual, marker='o', linestyle='-', color='blue', label='Actual')
    axs[1,1].plot(x_desired, y_desired, marker='x', linestyle='-', color='orange', label='Desired')
    axs[1,1].set_xlabel("X Position (m)")
    axs[1,1].set_ylabel("Y Position (m)")
    axs[1,1].set_title("Vehicle Position")
    axs[1,1].legend()
    axs[1,1].grid(True)

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

def plot_jerk(time, jerk):
    """Plots vehicle jerk (rate of acceleration) vs. time"""
    plt.figure(figsize=(12, 6))
    plt.plot(time, jerk, marker='o', linestyle='-', color='blue')
    plt.xlabel("Time (s)")
    plt.ylabel("Jerk (m/s³)")
    plt.title("Vehicle Jerk Over Time")
    plt.grid(True)
    plt.show()

def plot_heading_acceleration(time, heading_acc):
    """Plots vehicle heading acceleration vs. time"""
    plt.figure(figsize=(12, 6))
    plt.plot(time, heading_acc, marker='x', linestyle='-', color='orange')
    plt.xlabel("Time (s)")
    plt.ylabel("Heading Acceleration (rad/s²)")
    plt.title("Vehicle Heading Acceleration Over Time")
    plt.grid(True)
    plt.show()

def plot_crosstrack_error(time, cte):
    """Plots vehicle cross track error vs. time"""
    plt.figure(figsize=(12, 6))
    plt.plot(time, cte, marker='s', linestyle='-', color='green')
    plt.xlabel("Time (s)")
    plt.ylabel("Cross Track Error")
    plt.title("Vehicle Cross Track Error Over Time")
    plt.grid(True)
    plt.show()

def plot_position(time, x_actual, y_actual, x_desired, y_desired):
    """Plots vehicle actual and desired positions vs. time"""
    plt.figure(figsize=(12, 6))
    plt.plot(x_actual, y_actual, marker='o', linestyle='-', color='blue', label='Actual')
    plt.plot(x_desired, y_desired, marker='x', linestyle='-', color='orange', label='Desired')
    plt.xlabel("X Position (m)")
    plt.ylabel("Y Position (m)")
    plt.title("Vehicle Position Over Time")
    plt.legend()
    plt.grid(True)
    plt.show()


if __name__=='__main__':
    if len(sys.argv) != 2:
        print("Usage: python test_comfort_metrics.py <log_directory>")
        sys.exit(1)
    
    log_dir = sys.argv[1]
    behavior_file = os.path.join(log_dir, "behavior.json")
    tracker_file = os.path.join(log_dir, "PurePursuitTrajectoryTracker_debug.csv")
    
    times, accelerations, heading_rates = parse_behavior_log(behavior_file)
    time_jerk, jerk = compute_derivative(times, accelerations)
    time_heading_acc, heading_acc = compute_derivative(times, heading_rates)
    
    cte_time, cte, x_actual, y_actual, x_desired, y_desired = parse_tracker_csv(tracker_file)
    
    plot_metrics(time_jerk, jerk, time_heading_acc, heading_acc, cte_time, cte, x_actual, y_actual, x_desired, y_desired)
    
    print("Max (abs) jerk:", np.max(np.abs(jerk)))
    print("Max (abs) heading acceleration:", np.max(np.abs(heading_acc)))
    print("Max (abs) cross track error:", np.max(np.abs(cte)))