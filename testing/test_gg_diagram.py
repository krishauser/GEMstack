import numpy as np
import matplotlib.pyplot as plt

import sys
import os
sys.path.append(os.getcwd())

import json
import matplotlib.pyplot as plt
import numpy as np

def parse_behavior_log(filename):
    """
    Parses the behavior log file and extracts the following data:
    - vehicle time
    - vehicle acceleration
    - vehicle heading rate
    - speed
    """
    times = []
    accelerations = []
    heading_rates = []
    speeds = []
    
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
                speed = vehicle_data.get("speed")
                # Only add if all fields are available
                if None not in (t, acceleration, heading_rate, speed):
                    times.append(t)
                    accelerations.append(acceleration)
                    heading_rates.append(heading_rate)
                    speeds.append(speed)
    return (np.array(times), np.array(accelerations), np.array(heading_rates), 
            np.array(speeds))

def parse_tracker_csv(filename):
    """
    Parses the pure pursuit tracker log file and extracts the following data:
      - vehicle time (from column index 19)
      - X position actual (from column index 2)
      - Y position actual (from column index 5)
      - X position desired (from column index 11)
      - Y position desired (from column index 14)
    """

    data = np.genfromtxt(filename, delimiter=',', skip_header=1)
    vehicle_time = data[:, 19]
    cte = data[:, 20]
    x_actual, y_actual = data[:, 2], data[:, 5]
    x_desired, y_desired = data[:, 11], data[:, 14]
    speed_actual = data[:, -1]
    return vehicle_time, cte, x_actual, y_actual, x_desired, y_desired, speed_actual

def compute_derivative(times, values):
    """
    Computes the derivative of array with respect to time.
    Returns the time array and derivative values.
    """
    dt = np.diff(times)
    dv = np.diff(values)
    derivative = dv / dt
    return times[1:], derivative

def compute_gs(times, xs, ys):
    xtimes, vxs = compute_derivative(times, xs)
    ytimes, vys = compute_derivative(times, ys)
    vs = np.sqrt(vxs**2 + vys**2)
    long_times, long_accels = compute_derivative(times[1:], vs)
    psis = np.arctan2(vys, vxs)
    yaw_rate_times, yaw_rates = compute_derivative(times[1:], psis)
    lat_accels = yaw_rates * vs[1:]
    g = 9.81
    longitudinal_gs = long_accels / g
    lateral_gs = lat_accels / g
    return longitudinal_gs, lateral_gs, vs, lat_accels, long_accels

def plot_position(axis, x_actual, y_actual, x_desired, y_desired, safe_thresh=1, unsafe_thresh=2.5):
    """Plots vehicle actual and desired positions vs. time"""
    position_error = np.sqrt((x_desired - x_actual) ** 2 + (y_desired - y_actual) ** 2)
    # safety_scores = np.vectorize(compute_safety_factor)(position_error, safe_thresh, unsafe_thresh)
    
    axis.plot(y_desired, x_desired, linestyle='--', color='blue', label='Desired')
    axis.plot(y_actual, x_actual, color="black", linewidth=0.8, alpha=0.5, label='Actual')
    # axis.scatter(y_actual, x_actual, c=safety_scores, cmap=CMAP, vmin=0, vmax=1, edgecolors="black")
    
    axis.set_xlabel("Y Position (m)")
    axis.set_ylabel("X Position (m)")
    axis.set_title("Vehicle Position vs. Desired Position")
    axis.legend()
    axis.grid(True)

def plot_speeds(axis, speed_actual, comptued_speed, time):
    axis.plot(time, comptued_speed, label='computed speed')
    axis.plot(time, speed_actual, linestyle="--", label='current speed')
    axis.set_xlabel("time")
    axis.set_ylabel("speed m/s")
    axis.set_title("current speed vs computed speed")
    axis.legend()
    axis.grid(True)

def plot_accelerations(axis, accelerations, time):
    axis.plot(time, accelerations, label='accelerationn')
    axis.set_xlabel("time")
    axis.set_ylabel("accel m/s^2")
    axis.set_title("long accelerations")
    axis.legend()
    axis.grid(True)

def plot_gg_diagram(axis, longitudinal_gs, lateral_gs):
    """Plots gg diagram"""
    # Plot G-G diagram
    axis.scatter(longitudinal_gs, lateral_gs, alpha=0.5, label="Data Points")

    # Draw friction ellipse (assuming µ = 1.0)
    mu = 1.0
    theta = np.linspace(0, 2 * np.pi, 100)
    axis.plot(mu * np.cos(theta), mu * np.sin(theta), 'r', label="Theoretical Limit")

    axis.axhline(0, color='black', linewidth=0.8)
    axis.axvline(0, color='black', linewidth=0.8)
    axis.set_xlabel("Lateral Acceleration (g)")
    axis.set_ylabel("Longitudinal Acceleration (g)")
    axis.set_title("G-G Diagram from Acceleration & Yaw Rate")
    axis.legend()
    axis.grid()

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
    times, accelerations, heading_rates, speeds = parse_behavior_log(behavior_file)
    time_jerk, jerk = compute_derivative(times, accelerations)
    time_heading_acc, heading_acc = compute_derivative(times, heading_rates)

    # Pure pursuit tracker file exists: parse and plot all metrics
    if os.path.exists(tracker_file):
        vehicle_time, cte, x_actual, y_actual, x_desired, y_desired, speed_actual = parse_tracker_csv(tracker_file)
        
        longitudinal_gs, lateral_gs, calculated_speed, lat_accels, long_accels = compute_gs(vehicle_time, x_actual, y_actual)
        # print(longitudinal_gs)
        fig, axs = plt.subplots(2, 2, figsize=(12, 4))
        plot_gg_diagram(axs[0, 0], longitudinal_gs, lateral_gs)
        plot_position(axs[0, 1], x_actual, y_actual, x_desired, y_desired)
        plot_speeds(axs[1, 0], speed_actual[1:], calculated_speed, vehicle_time[1:])
        plot_accelerations(axs[1, 1], long_accels, vehicle_time[2:])
        plt.show()
    # Pure pursuit tracker file is missing: plot only behavior.json metrics
    else:
        print("Tracker file is missing. Skipping cross track error and vehicle position plots.")
