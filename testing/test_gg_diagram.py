import numpy as np
import matplotlib.pyplot as plt

import sys
import os
sys.path.append(os.getcwd())

import json
import matplotlib.pyplot as plt
import numpy as np
from GEMstack.mathutils.transforms import lat_lon_to_xy

# from pyproj import Proj, Transformer
from scipy.spatial import ConvexHull

def latlon_to_xy(latitudes, longitudes, origin_lat, origin_lon):
    """
    Converts lat/lon to local X/Y coordinates in meters using a local ENU projection.

    latitudes, longitudes: arrays of GPS coordinates
    origin_lat, origin_lon: reference point (usually the first point)

    Returns: xs, ys (in meters, relative to origin)
    """
    # WGS84 to ENU local projection (east-north-up)
    transformer = Transformer.from_crs(
        "epsg:4326",  # WGS84 lat/lon
        f"+proj=tmerc +lat_0={origin_lat} +lon_0={origin_lon} +k=1 +x_0=0 +y_0=0 +ellps=WGS84",
        always_xy=True
    )

    xs, ys = transformer.transform(longitudes, latitudes)
    return xs, ys


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
    xs = []
    ys = []
    ref_lat = 0.0
    ref_long = 0.0
    
    with open(filename, 'r') as f:
        for idx, line in enumerate(f):
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
                speed = vehicle_data.get("v")
                # print(speed)
                # pose_data = vehicle_data["pose"].get("pose")
                frame = vehicle_data["pose"].get("frame")
                x = vehicle_data["pose"].get("x")
                y = vehicle_data["pose"].get("y")
                if idx == 0:
                    ref_lat = y
                    ref_long = x
                x, y = lat_lon_to_xy(y,x,ref_lat,ref_long)
                # Only add if all fields are available
                if None not in (t, acceleration, heading_rate, speed)  and frame != 3:
                    times.append(t)
                    accelerations.append(acceleration)
                    heading_rates.append(heading_rate)
                    speeds.append(speed)
                    
                    xs.append(x)
    
                    ys.append(y)
        # print(len(speeds))
        # print(len(times))
    return np.array(times), np.array(xs), np.array(ys), np.array(speeds), np.array(accelerations), np.array(heading_rates)

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
    # x_desired, y_desired = data[:, 11], data[:, 14]  ### PP
    x_desired, y_desired = data[:, 8], data[:, 11] #Stanley
    speed_actual = data[:, -1]
    des_speed = data[:, -13]
    return vehicle_time, cte, x_actual, y_actual, x_desired, y_desired, speed_actual, des_speed

def compute_derivative(times, values):
    """
    Computes the derivative of array with respect to time.
    Returns the time array and derivative values.
    """
    dt = np.diff(times)
    dv = np.diff(values)
    derivative = dv / dt
    return times[1:], derivative

def moving_average(data, window_size=5):
    """
    Simple moving average smoother for 1D array.
    Pads edges by repeating border values.
    """
    pad = window_size // 2
    padded = np.pad(data, (pad, pad), mode='edge')
    smoothed = np.convolve(padded, np.ones(window_size) / window_size, mode='valid')
    return smoothed

def remove_duplicate_positions(times, xs, ys):
    """
    Removes consecutive duplicate (x, y) points.
    Keeps only unique steps in position.
    """
    xs = np.array(xs)
    ys = np.array(ys)
    times = np.array(times)

    # Compute deltas
    dx = np.diff(xs)
    dy = np.diff(ys)

    # Find where movement occurs
    moved = (dx != 0) | (dy != 0)

    # Always keep the first point
    keep_idx = np.insert(moved, 0, True)

    return times[keep_idx], xs[keep_idx], ys[keep_idx]

def compute_gs(times, xs, ys, speeds, accelerations, heading_rates):
    # vxs = np.gradient(xs, times)
    # vys  = np.gradient(ys, times)
    times = times - times[0]
    dx = np.diff(xs)
    dy = np.diff(ys)
    dt = np.diff(times)

    vxs = dx/dt
    vys = dy/dt
    vs = np.sqrt(vxs**2 + vys**2)

    vs = moving_average(vs, window_size=7)

    valid_mask = vs > 1e-6  # small threshold to avoid float noise
    vs = vs[valid_mask]
    times = times[:-1]
    times = times[valid_mask]

    long_accels = np.gradient(vs, times)

    accel_valid_mask = np.abs(long_accels) <  5 

    psis = np.arctan2(vys, vxs)
    psis = psis[valid_mask]
    psi_mask = psis > 1e-6
    psis = psis[psi_mask]
    times = times[psi_mask]
    vs = vs[psi_mask]
    long_accels = long_accels[psi_mask]

    yaw_rates = np.gradient(psis, times)
    
    lat_accels = yaw_rates * vs
    g = 9.81
    longitudinal_gs = long_accels / g
    lateral_gs = lat_accels / g

    print("max long accel: " + str(np.max(long_accels)))
    print("min long accel: " + str(np.min(long_accels)))
    print("max lat accel: " + str(np.max(lat_accels)))


    return longitudinal_gs, lateral_gs, vs, lat_accels, long_accels, times, valid_mask

def plot_position(axis, x_actual, y_actual, x_desired=None, y_desired=None, safe_thresh=1, unsafe_thresh=2.5):
    """Plots vehicle actual and desired positions vs. time"""
    # position_error = np.sqrt((x_desired - x_actual) ** 2 + (y_desired - y_actual) ** 2)
    
    axis.plot(y_desired, x_desired, linestyle='--', color='blue', label='Desired')
    axis.plot(y_actual, x_actual, color="black", linewidth=0.8, alpha=0.5, label='Actual')
    
    axis.set_xlabel("Y Position (m)")
    axis.set_ylabel("X Position (m)")
    axis.set_title("GEM Position")
    axis.legend()
    axis.grid(True)

def plot_speeds(axis, time, speed_actual, comptued_speed = None):
    if  comptued_speed is not None:
        axis.plot(time, comptued_speed, label='computed speed')
    axis.plot(time, speed_actual, linestyle="--")#, label='current speed')
    axis.set_xlabel("time")
    axis.set_ylabel("speed m/s")
    axis.set_title("GEM Speed")
    # axis.legend()
    axis.grid(True)

def plot_accelerations(axis, accelerations, time):
    axis.plot(time, accelerations, linestyle='--')#,label='acceleration')
    axis.set_xlabel("time")
    axis.set_ylabel("accel m/s^2")
    axis.set_title("GEM Accelerations")
    # axis.legend()
    axis.grid(True)

def plot_gg_diagram(axis, longitudinal_gs, lateral_gs):
    """Plots gg diagram"""
    # Plot G-G diagram
    axis.scatter(lateral_gs, longitudinal_gs, alpha=0.5, label="Data Points")

    # draw op envelope
    gy_full = np.concatenate([longitudinal_gs, longitudinal_gs])
    gx_full = np.concatenate([lateral_gs, -lateral_gs])

    points = np.vstack((gx_full, gy_full)).T
    hull = ConvexHull(points)
    for simplex in hull.simplices:
        axis.plot(points[simplex, 0], points[simplex, 1], 'r-')
    axis.fill(points[hull.vertices, 0], points[hull.vertices, 1], 
                'r', alpha=0.1, label='Operating Envelope')

    axis.axhline(0, color='black', linewidth=0.8)
    axis.axvline(0, color='black', linewidth=0.8)
    axis.set_xlabel("Lateral Acceleration (g)")
    axis.set_ylabel("Longitudinal Acceleration (g)")
    axis.set_title("G-G Diagram and Operating Envelope")
    axis.legend()
    axis.grid()



if __name__=='__main__':
    if len(sys.argv) != 2:
        print("Usage: python test_comfort_metrics.py <log_directory>")
        sys.exit(1)
    
    log_dir = sys.argv[1]
    behavior_file = os.path.join(log_dir, "behavior.json")
    tracker_file = os.path.join(log_dir, "StanleyTrajectoryTracker_debug.csv")
    # tracker_file = os.path.join(log_dir, "PurePursuitTrajectoryTracker_debug.csv")
    
    # if behavior.json doesn't exist, print error and exit
    if not os.path.exists(behavior_file):
        print("Error: behavior.json file is missing in log folder.")
        sys.exit(1)
    
    # Parse behavior log file and compute metrics
    times, xs, ys, speeds, accelerations, heading_rates = parse_behavior_log(behavior_file)
    time_jerk, jerk = compute_derivative(times, accelerations)
    time_heading_acc, heading_acc = compute_derivative(times, heading_rates)

    times, xs, ys = remove_duplicate_positions(times, xs, ys)
    longitudinal_gs, lateral_gs, vs, lat_accels, long_accels, times, valid_mask = compute_gs(times, xs, ys, speeds, accelerations, heading_rates)

    vehicle_time, cte, x_actual, y_actual, x_desired, y_desired, speed_actual, des_speed = parse_tracker_csv(tracker_file)

    fig, axs = plt.subplots(2, 2, figsize=(12, 8))
    plot_gg_diagram(axs[0, 0], longitudinal_gs, lateral_gs)
    # plot_position(axs[0, 1], xs, ys) #, x_desired, y_desired)
    plot_position(axs[0, 1], x_actual, y_actual, x_desired, y_desired)
    # plot_speeds(axs[1, 0], times, vs)
    plot_speeds(axs[1, 0], vehicle_time, des_speed, speed_actual)

    plot_accelerations(axs[1, 1], long_accels, times)
    plt.tight_layout()
    name = "gg_diagram.png"
    file_path = os.path.join(log_dir, name)

    plt.savefig(file_path, dpi=300, format='png')
    print(f"Plots saved as {name}")

    plt.show()

    # # Pure pursuit tracker file exists: parse and plot all metrics
    # if os.path.exists(tracker_file):
        
    #     longitudinal_gs, lateral_gs, calculated_speed, lat_accels, long_accels, times, valid_mask = compute_gs(vehicle_time, x_actual, y_actual, speed_actual)
    #     fig, axs = plt.subplots(2, 2)
    #     plot_gg_diagram(axs[0, 0], longitudinal_gs, lateral_gs)
    #     plot_position(axs[0, 1], x_actual, y_actual, x_desired, y_desired)
    #     plot_speeds(axs[1, 0], speed_actual[valid_mask], calculated_speed, times)

    #     plot_accelerations(axs[1, 1], long_accels, times)

    #     plt.show()
    # # Pure pursuit tracker file is missing: plot only behavior.json metrics
    # else:
    #     print("Tracker file is missing. Skipping cross track error and vehicle position plots.")
