####Following code can be used to visualize performance of Stanley after using it.###
###How to use?- After running the simulation or on actual vehicle you get logs in log folder###
### You can run the command-   python3 logplot_s.py "logfilename".###

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

def parse_behavior_log(filename):
    times = []
    accelerations = []
    heading_rates = []
    
    with open(filename, 'r') as f:
        first_time = None  # Store the first timestamp for shifting
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
                if t is not None and acceleration is not None and heading_rate is not None:
                    if first_time is None:
                        first_time = t  # Set the first timestamp
                    times.append(t - first_time)  # Shift time to start from 0
                    accelerations.append(acceleration)
                    heading_rates.append(heading_rate)
    
    return np.array(times), np.array(accelerations), np.array(heading_rates)

def parse_tracker_csv(filename):
    data = np.genfromtxt(filename, delimiter=',', skip_header=1)
    first_time = data[0, 18]  # Normalize time to start from 0
    cte_time = data[:, 18] - first_time
    cte = data[:, 20]
    x_actual, y_actual = data[:, 2], data[:, 5]
    x_desired, y_desired = data[:, 8], data[:, 11]
    return cte_time, cte, x_actual, y_actual, x_desired, y_desired

def compute_derivative(times, values):
    dt = np.diff(times)
    dv = np.diff(values)
    derivative = dv / dt
    return times[1:], derivative

def plot_metrics(time_jerk, jerk, time_heading_acc, heading_acc, cte_time, cte, x_actual, y_actual, x_desired, y_desired):
    global_min_time = min(time_jerk[0], time_heading_acc[0], cte_time[0])
    global_max_time = max(time_jerk[-1], time_heading_acc[-1], cte_time[-1])
    
    fig, axs = plt.subplots(2, 2, figsize=(12, 8))
    fig.subplots_adjust(hspace=0.4, wspace=0.3)

    axs[0,0].plot(time_jerk, jerk, marker='', linestyle='-', color='blue')
    axs[0,0].set_ylabel("Jerk (m/s³)")
    axs[0,0].set_title("Vehicle Jerk Over Time")
    axs[0,0].grid(True)
    axs[0,0].set_xlim(global_min_time, global_max_time)

    axs[0,1].plot(time_heading_acc, heading_acc, marker='', linestyle='-', color='orange')
    axs[0,1].set_ylabel("Heading Acceleration (rad/s²)")
    axs[0,1].set_title("Vehicle Heading Acceleration Over Time")
    axs[0,1].grid(True)
    axs[0,1].set_xlim(global_min_time, global_max_time)

    axs[1,0].plot(cte_time, cte, marker='', linestyle='-', color='green')
    axs[1,0].set_xlabel("Time (s)")
    axs[1,0].set_ylabel("Cross Track Error")
    axs[1,0].set_title("Vehicle Cross Track Error Over Time")
    axs[1,0].grid(True)
    axs[1,0].set_xlim(global_min_time, global_max_time)

    axs[1,1].plot(x_actual, y_actual, marker='', linestyle='-', color='blue', label='Actual')
    axs[1,1].plot(x_desired, y_desired, marker='', linestyle='-', color='orange', label='Desired')
    axs[1,1].set_xlabel("X Position (m)")
    axs[1,1].set_ylabel("Y Position (m)")
    axs[1,1].set_title("Vehicle Position")
    axs[1,1].legend()
    axs[1,1].grid(True)

    plt.show()

if __name__=='__main__':
    if len(sys.argv) != 2:
        print("Usage: python test_comfort_metrics.py <log_directory>")
        sys.exit(1)

    log_dir = sys.argv[1]
    behavior_file = os.path.join(log_dir, "behavior.json")
    #tracker_file = os.path.join(log_dir, "PurePursuitTrajectoryTracker_debug.csv")
    tracker_file = os.path.join(log_dir, "StanleyTrajectoryTracker_debug.csv")

    times, accelerations, heading_rates = parse_behavior_log(behavior_file)
    time_jerk, jerk = compute_derivative(times, accelerations)
    time_heading_acc, heading_acc = compute_derivative(times, heading_rates)
    
    cte_time, cte, x_actual, y_actual, x_desired, y_desired = parse_tracker_csv(tracker_file)
    
    plot_metrics(time_jerk, jerk, time_heading_acc, heading_acc, cte_time, cte, x_actual, y_actual, x_desired, y_desired)
    
    print("Max (abs) jerk:", np.max(np.abs(jerk)))
    print("Avg jerk:", np.mean(np.abs(jerk)))
    print("Max (abs) heading acceleration:", np.max(np.abs(heading_acc)))
    print("Avg heading acceleration:", np.mean(np.abs(heading_acc)))
    print("Max (abs) cross track error:", np.max(np.abs(cte)))
    print("Avg cross track error:", np.mean(np.abs(cte)))

