import numpy as np
import sys
import os
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
from scipy.signal import savgol_filter
from scipy.interpolate import splprep, splev
from ...knowledge.vehicle.geometry import steer2front

# from ..stanley import normalise_angle
from ...knowledge.vehicle import dynamics
from ...knowledge.vehicle import geometry
from ...utils import settings
import pandas as pd


# from ..mathutils import transforms,collisions


def parse_route_csv(filename):
    """
    Parses the pure pursuit tracker log file and extracts the following data:
      - vehicle time (from column index 19)
      - X position actual (from column index 2)
      - Y position actual (from column index 5)
      - X position desired (from column index 11)
      - Y position desired (from column index 14)
    """

    data = np.genfromtxt(filename, delimiter=',', skip_header=1)
    x_desired = data[:, 0]
    y_desired = data[:, 1]
    return x_desired, y_desired

def moving_average(data, window_size=9):
    """
    Simple moving average smoother for 1D array.
    Pads edges by repeating border values.
    """
    pad = window_size // 2
    padded = np.pad(data, (pad, pad), mode='edge')
    smoothed = np.convolve(padded, np.ones(window_size) / window_size, mode='valid')
    return smoothed

def compute_curvature_by_distance(xs, ys, ds, spacing=0.5):
    """
    Computes curvature using 3-point method with approximately equal distance on both sides.
    spacing: desired distance (in meters) from center point to side points.
    """
    xs = np.array(xs)
    ys = np.array(ys)
    N = len(xs)
    kappa = np.zeros(N)

    # Compute cumulative distances
    s = np.insert(np.cumsum(ds), 0, 0)

    for i in range(N):
        # Find index j before i such that s[i] - s[j] ≈ spacing
        j = i
        while j > 0 and (s[i] - s[j]) < spacing:
            j -= 1

        # Find index k after i such that s[k] - s[i] ≈ spacing
        k = i
        while k < N - 1 and (s[k] - s[i]) < spacing:
            k += 1

        if j == i or k == i or j < 0 or k >= N:
            kappa[i] = 0
            continue

        # Points: P1 (j), P2 (i), P3 (k)
        x1, y1 = xs[j], ys[j]
        x2, y2 = xs[i], ys[i]
        x3, y3 = xs[k], ys[k]

        # Triangle side lengths
        a = np.hypot(x2 - x1, y2 - y1)
        b = np.hypot(x3 - x2, y3 - y2)
        c = np.hypot(x1 - x3, y1 - y3)
        s_tri = (a + b + c) / 2

        # Heron’s formula for area
        area = np.sqrt(max(s_tri * (s_tri - a) * (s_tri - b) * (s_tri - c), 0))
        denom = a * b * c

        if area > 1e-6 and denom > 1e-6:
            radius = (a * b * c) / (4 * area)
            kappa[i] = 1 / radius
        else:
            kappa[i] = 0

    kappa[np.abs(kappa) < 1e-2] = 0

    return kappa

def compute_spline_curvature(x, y, s=0.0, num=1000):
    """
    Computes curvature from 2D path (x, y) using parametric splines.
    
    Parameters:
        x, y : list or np.array
            Input path coordinates
        s : float
            Smoothing factor for spline (0 = interpolating spline)
        num : int
            Number of points to evaluate spline and curvature at
            
    Returns:
        x_smooth, y_smooth, curvature : np.ndarrays
    """
    # Fit parametric spline
    num = len(x)
    tck, u = splprep([x, y], s=s)
    u_fine = np.linspace(0, 1, num)
    
    # Evaluate spline and its first and second derivatives
    x_smooth, y_smooth = splev(u_fine, tck)
    dx, dy = splev(u_fine, tck, der=1)
    ddx, ddy = splev(u_fine, tck, der=2)

    # Compute curvature: κ = (x'y'' - y'x'') / (x'^2 + y'^2)^(3/2)
    denom = (dx**2 + dy**2)**1.5 + 1e-8  # Avoid divide by zero
    kappa = (dx * ddy - dy * ddx) / denom

    return kappa


def lateral_speed_limit(kappa, ay_max, v_max):
    v_lat = np.sqrt(np.maximum(ay_max / (np.abs(kappa) + 1e-8), 0))
    return np.minimum(v_lat, v_max)

def limit_ax_for_friction_ellipse(v, kappa, ax_limit, ay_max):
    # Compute a_y at current v
    ay = v**2 * kappa
    ay_ratio = ay / ay_max
    # print(ay_ratio)
    ay_ratio = np.clip(ay_ratio, -1.0, 1.0)
    # Remaining fraction for ax
    ax_ratio = np.sqrt(np.maximum(1.0 - ay_ratio**2, 0.0))
    return ax_limit * ax_ratio

def apply_trail_braking(ds, v_lat, kappa, ax_max, ax_min, ay_max, max_iter=2, tol=1e-3):
    N = len(ds)
    v_profile = v_lat.copy()
    v_profile[0] = 0.0
    forward_vs = v_lat.copy()
    backward_vs = v_lat.copy()

    v_old = v_profile.copy()

    # Forward (accelerating out of curves)
    for i in range(1, N):
        ax_limit = limit_ax_for_friction_ellipse(v_profile[i-1], kappa[i-1], ax_max, ay_max)
        ax_limit = np.clip(ax_limit, ax_min, ax_max)
        v_allowed = np.sqrt(v_profile[i-1]**2 + 2 * ax_limit * ds[i-1])
        v_profile[i] = min(v_profile[i], v_allowed)
        forward_vs[i] = v_allowed
        if v_allowed < 0:
            print("v < 0 in forward pass")
        # forward_vs[i] = v_profile[i]

    # Backward (trail braking into corners)
    for i in range(N - 1, 0, -1):
        ax_limit = limit_ax_for_friction_ellipse(v_profile[i], kappa[i], abs(ax_min), ay_max)
        arg = v_profile[i]**2 + 2 * ax_limit * ds[i]
        v_allowed = np.sqrt(max(arg,0.0))
        v_profile[i-1] = min(v_profile[i-1], v_allowed)
        
        backward_vs[i-1] = v_allowed
        if v_allowed < 0:
            print("v < 0 in backward pass")
        
    return v_profile, forward_vs, backward_vs

def limit_velocity_by_steering_rate(ds, velocity, wheelbase, max_steering_rate, kappa):
    """
    Limits the velocity based on steering angle rate of change.
    
    Parameters:
        x (np.ndarray): x positions along the trajectory.
        y (np.ndarray): y positions along the trajectory.
        velocity (np.ndarray): velocity profile along the trajectory.
        wheelbase (float): vehicle's wheelbase (L).
        max_steering_rate (float): maximum allowable steering angle rate (rad/s).
        
    Returns:
        np.ndarray: limited velocity profile.
    """
    # Compute steering angle delta: delta = arctan(L * kappa)
    steering_angle = np.arctan(wheelbase * kappa)

    # Compute steering rate: d(delta)/dt
    ddelta = np.gradient(steering_angle)
    dt = ds / (velocity + 1e-6)  # Add small value to prevent division by zero
    ddelta_dt = ddelta / (dt + 1e-6)

    # max steering wheel rate to steering rate
    max_steering_rate = steer2front(max_steering_rate)

    # Limit velocity where steering rate exceeds max
    limited_velocity = np.copy(velocity)
    for i in range(len(velocity)):
        if abs(ddelta_dt[i]) > max_steering_rate:
            ddelta_dt[i] = max_steering_rate
            limited_velocity[i] = max(0, min(
                velocity[i],
                abs((ds[i] / (ddelta[i] / ddelta_dt[i])+ 1e-6) - 1e-6)
            ))
    return limited_velocity, steering_angle


def compute_time_profile(xs, ys, v_profile):
    xs = np.array(xs)
    ys = np.array(ys)
    v_profile = np.array(v_profile)

    dx = np.diff(xs)
    dy = np.diff(ys)
    ds = np.hypot(dx, dy)  # segment distances

    # Use average speed between points to estimate time between them
    v_mid = (v_profile[:-1] + v_profile[1:]) / 2
    v_mid = np.clip(v_mid, 0.1, None)  # Avoid divide-by-zero

    dt = ds / v_mid
    t = np.insert(np.cumsum(dt), 0, 0.0)  # cumulative time starting from 0

    return t

def plot_speed_profile_gradient(fig, axis, xs, ys, speeds, cmap='jet'):
    xs = np.array(xs)
    ys = np.array(ys)
    speeds = np.array(speeds)

    # Build segments between points
    points = np.array([xs, ys]).T.reshape(-1, 1, 2)
    segments = np.concatenate([points[:-1], points[1:]], axis=1)
    # segments = [np.column_stack([xs, ysi]) for ysi in ys.T]

    # Create line collection
    lc = LineCollection(segments, cmap=cmap, linewidth=2)
    lc.set_array(speeds[:-1])  # color values
    lc.set_linewidth(2)

    # fig, ax = plt.subplots(figsize=(8, 6))
    line = axis.add_collection(lc)
    fig.colorbar(line, ax=axis, label='Speed (m/s)')
    axis.set_xlim(xs.min(), xs.max())
    axis.set_ylim(ys.min(), ys.max())
    axis.set_aspect('equal', adjustable='box')
    axis.set_title('Speed Profile Along Path')
    axis.set_xlabel('X [m]')
    axis.set_ylabel('Y [m]')
    plt.grid(True)

def plot_x_y(axis, x, y, x_label, y_label):
    axis.plot(x, y)#, label='Actual')
    axis.set_xlabel(x_label)
    axis.set_ylabel(y_label)
    axis.grid(True)

def compute_velocity_profile(points, plot=True):

    v_max  = settings.get('vehicle.limits.max_speed')
    ax_max = settings.get('vehicle.limits.max_longitudinal_acceleration')
    ax_min = settings.get('vehicle.limits.min_longitudinal_acceleration')
    ay_max = settings.get('vehicle.limits.max_lateral_acceleration')
    max_steering_rate = settings.get('vehicle.limits.max_steering_rate')
    wheelbase = settings.get('vehicle.geometry.wheelbase')

    points = np.array(points)
    print(points)
    xs, ys = points[:,0], points [:,1]
    print(xs)

    dx = np.diff(xs)
    dy = np.diff(ys)
    ds = np.hypot(dx, dy)
    ds = np.append(ds, ds[-1])

    # kappa = compute_curvature_by_distance(xs,ys, ds)
    kappa = compute_spline_curvature(xs, ys)

    # max speeds from lateral acceleration limits
    v_lat = lateral_speed_limit(kappa, ay_max, v_max)

    t_lat = compute_time_profile(xs, ys,v_lat)

    # max speeds from steering limits
    v_steer, steering_angles = limit_velocity_by_steering_rate(ds, v_lat, wheelbase, max_steering_rate, kappa)
    
    # v_steer = savgol_filter(v_steer, window_length=9, polyorder=3)
    
    t_steer = compute_time_profile(xs, ys, v_steer)

    #max speeds from longitudinal acceleration limits
    v_profile, forward_vs, backward_vs = apply_trail_braking(ds, v_steer, kappa, ax_max, ax_min, ay_max)

    # v_profile = savgol_filter(v_profile, window_length=9, polyorder=3)

    t = compute_time_profile(xs, ys, v_profile)

    if plot is False:
        return t, v_profile
    else:
        fig, axs = plt.subplots(2, 2, figsize=(12, 8))

        dv = np.diff(v_profile)
        dt = np.diff(t)

        a = dv/dt

        plot_x_y(axs[0, 0], t, v_profile, "t", "v")
        # plot_x_y(axs[1, 1], t_steer, v_steer, "t", "v_steer")
        # plot_x_y(axs[0, 1], t_steer, steering_angles, "t", "angles")
        plot_x_y(axs[1, 0], t[:-1], a, "t", "a")
        # plot_x_y(axs[1, 0], xs, ys, "x", "y")
        # plot_x_y(axs[1, 1], t, forward_vs, "t", "forward & backward vs")
        # plot_x_y(axs[1, 1], t, backward_vs, "t", "forward & backward vs")
        plot_x_y(axs[0, 1], t, kappa, "t", "kappa")
        # plot_x_y(axs[1, 1], t, ys, "t", "x & y")
        # plt.show()

        plot_speed_profile_gradient(fig, axs[1, 1], xs, ys, v_profile)
        # plot_speed_profile_gradient(fig, axs[1, 1], xs, ys, v_steer)


        # Save to CSV
        df = pd.DataFrame({'x': xs, 'y': ys, 'v': v_profile, 't': t})
        df.to_csv('v_profile.csv', index=False)


        plt.tight_layout()
        plt.show()

        return t, v_profile



if __name__=='__main__':
    if len(sys.argv) != 2:
        print("Usage: python velocity_profile.py <route file name>")
        sys.exit(1)
    
    path_file = sys.argv[1]
    
    # if behavior.json doesn't exist, print error and exit
    if not os.path.exists(path_file):
        print("Error: route file not found.")
        sys.exit(1)
    
    xs, ys = parse_route_csv(path_file)

    points = list(zip(xs,ys))

    t, v_profile, t2, v_profile2 = compute_velocity_profile(points, plot=True)

    # # max long accel: 2.0769700074721493
    # # min long accel: -2.6197231578072313
    # # max lat accel: 3.3754426456004567

    fig, axs = plt.subplots(2, 2, figsize=(12, 8))

    dv = np.diff(v_profile)
    dt = np.diff(t)

    a = dv/dt

    plot_x_y(axs[0, 0], t, v_profile, "t", "v")
    plot_x_y(axs[0, 1], t2, v_profile2, "t", "v2")
    # plot_x_y(axs[0, 1], t[:-1], a, "t", "a")
    # plot_x_y(axs[1, 0], xs, ys, "x", "y")
    # plot_x_y(axs[1, 1], t, forward_vs, "t", "forward & backward vs")
    # plot_x_y(axs[1, 1], t, backward_vs, "t", "forward & backward vs")
    # plot_x_y(axs[1, 1], t, xs, "t", "x & y")
    # plot_x_y(axs[1, 1], t, ys, "t", "x & y")
    # plt.show()

    plot_speed_profile_gradient(fig, axs[1, 0], xs, ys, v_profile)
    plot_speed_profile_gradient(fig, axs[1, 1], xs, ys, v_profile2)
    plt.tight_layout()
    plt.show()