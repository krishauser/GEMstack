import numpy as np
import pandas as pd

# Parameters
radius_left = 10     # Radius scaling on the left side
radius_right = 20   # Radius scaling on the right side
num_points = 200    # Total number of points

# Generate t values from -π/2 to 3π/2 to make a full loop
t = np.linspace(-np.pi/2, 3*np.pi/2, num_points)

# Parametric figure-8 (lemniscate-like), with asymmetric lobes
x = np.sin(t)
y = np.sin(t) * np.cos(t)

# Apply asymmetric scaling
x_scaled = np.where(x < 0, x * radius_left, x * radius_right)
y_scaled = np.where(x < 0, y * radius_left, y * radius_right)

# Create DataFrame and save
figure8_df = pd.DataFrame({'x': x_scaled, 'y': y_scaled})
figure8_df.to_csv('/home/hbst/RACING/GEMstack/GEMstack/knowledge/routes/figure8_points.csv', index=False)