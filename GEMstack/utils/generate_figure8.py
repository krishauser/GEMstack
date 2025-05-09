import numpy as np
import pandas as pd

# Parameters
radius_left = 10         # Radius scaling on the left side
radius_right = 20       # Radius scaling on the right side
num_points = 200        # Total number of points
rotation_angle_deg = -45 # Rotation angle in degrees

# Convert degrees to radians
theta = np.radians(rotation_angle_deg)

# Generate parametric figure-8 (lemniscate-style)
t = np.linspace(-np.pi/2, 3*np.pi/2, num_points)
x = np.sin(t)
y = np.sin(t) * np.cos(t)

# Apply asymmetric scaling
x_scaled = np.where(x < 0, x * radius_left, x * radius_right)
y_scaled = np.where(x < 0, y * radius_left, y * radius_right)

# Apply rotation
x_rotated = x_scaled * np.cos(theta) - y_scaled * np.sin(theta)
y_rotated = x_scaled * np.sin(theta) + y_scaled * np.cos(theta)

# Create DataFrame and save
figure8_df = pd.DataFrame({'x': x_rotated, 'y': y_rotated})
figure8_df.to_csv('../GEMstack/GEMstack/knowledge/routes/figure8.csv', index=False)