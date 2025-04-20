import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches

# Constants
circle_center = (0, 0)
radius = 3
fov_deg = 60  # Field of view in degrees
camera_offset_angle_deg = 45  # Camera is 45° from tangent
num_path_points = 36

# Convert FOV and camera offset to radians
fov_rad = np.radians(fov_deg)
camera_offset_angle_rad = np.radians(camera_offset_angle_deg)

# Generate circular path around the object
angles = np.linspace(0, 2 * np.pi, num_path_points, endpoint=False)
path_points = [(circle_center[0] + radius * np.cos(a), circle_center[1] + radius * np.sin(a)) for a in angles]

# Plot
fig, ax = plt.subplots(figsize=(8, 8))
ax.set_aspect('equal')

# Draw the object
circle = plt.Circle(circle_center, 0.5, color='red', label='Object')
ax.add_patch(circle)

# Draw the circular path
circle_path = plt.Circle(circle_center, radius, color='gray', linestyle='--', fill=False, label='Path')
ax.add_patch(circle_path)

# Draw FOV cones
for angle, (px, py) in zip(angles, path_points):
    # Heading direction of the vehicle is tangent: +90° to the radial vector
    tangent_angle = angle + np.pi / 2
    camera_direction = tangent_angle + camera_offset_angle_rad

    # FOV wedge
    wedge = patches.Wedge(
        (px, py), 1,  # position and radius
        np.degrees(camera_direction - fov_rad / 2),
        np.degrees(camera_direction + fov_rad / 2),
        color='skyblue', alpha=0.5
    )
    ax.add_patch(wedge)

# Formatting
ax.plot(*zip(*path_points), marker='o', color='black', linestyle='None', label='Camera Points')
ax.set_xlim(-radius-2, radius+2)
ax.set_ylim(-radius-2, radius+2)
ax.grid(True)
ax.legend()
plt.title("Camera FOV Coverage Around an Object")
plt.show()
