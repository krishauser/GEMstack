import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Wedge, Circle

def simulate_camera_path(circle_center, radius, fov_deg, path_type="circle", num_points=100, scaling = 1.5):
    xc, yc = circle_center
    fov_rad = np.radians(fov_deg)
    offset_rad = np.radians(45)  # 45 degree camera offset

    covered_intervals = []

    if path_type == "circle":
        angles = np.linspace(0, 2 * np.pi, num_points, endpoint=False)
        path_points = [(xc + scaling * radius * np.cos(a), yc + scaling * radius * np.sin(a)) for a in angles]
        path_points = path_points
    elif path_type == "line":
        path_points = [(xc - scaling * radius + 2 * scaling * radius * i / (num_points - 1), yc + scaling * radius)
                       for i in range(num_points)]
    else:
        raise ValueError("Unknown path_type")

    for px, py in path_points:
        dx = xc - px
        dy = yc - py
        angle_to_center = np.arctan2(dy, dx) + offset_rad  # Add 45 degree offset
        covered_intervals.append((angle_to_center - fov_rad / 2, angle_to_center + fov_rad / 2))

    covered_angles = np.zeros(360)
    for start, end in covered_intervals:
        start_deg = int(np.degrees(start)) % 360
        end_deg = int(np.degrees(end)) % 360

        if start_deg <= end_deg:
            covered_angles[start_deg:end_deg + 1] = 1
        else:
            covered_angles[start_deg:] = 1
            covered_angles[:end_deg + 1] = 1

    covered_fraction = np.sum(covered_angles) / 360

    return covered_angles, covered_fraction, path_points


def plot_simulation(circle_center, radius, path_points, covered_angles, fov_deg):
    fig, ax = plt.subplots()
    ax.set_aspect('equal')

    obj = Circle(circle_center, radius, fill=False, linestyle='--', color='black', label='Object')
    ax.add_patch(obj)

    for px, py in path_points:
        ax.plot(px, py, 'ro', markersize=2)

    fov_rad = np.radians(fov_deg)
    offset_deg = 45  # 45 degree offset for camera visualization
    for (px, py) in path_points[::10]:
        dx = circle_center[0] - px
        dy = circle_center[1] - py
        theta = np.degrees(np.arctan2(dy, dx)) + offset_deg  # Add offset
        fov = Wedge((px, py), 1.5 * radius, theta - fov_deg / 2, theta + fov_deg / 2, color='blue', alpha=0.15)
        ax.add_patch(fov)

    theta_vals = np.linspace(0, 2 * np.pi, 360)
    for i, covered in enumerate(covered_angles):
        if covered:
            theta = np.radians(i)
            x = circle_center[0] + radius * np.cos(theta)
            y = circle_center[1] + radius * np.sin(theta)
            ax.plot(x, y, 'g.', markersize=1)

    ax.set_title('Simulated Coverage with 45° Camera Offset')
    ax.legend()
    plt.grid(True)
    plt.show()


# === Run Simulation ===
circle_center = (0, 0)
radius = 1.0
fov_deg = 60

covered_angles, covered_fraction, path_points = simulate_camera_path(
    circle_center, radius, fov_deg, path_type="circle"
)

print("Covered Fraction", covered_fraction)
plot_simulation(circle_center, radius, path_points, covered_angles, fov_deg)
