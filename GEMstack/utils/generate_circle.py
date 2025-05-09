import numpy as np
import pandas as pd

# Parameters
radius = 20  # radius of the circle in meters
num_points = 100  # number of points around the circle

# Generate angles from 0 to 2π
angles = np.linspace(0, 2 * np.pi, num_points, endpoint=False)

# Compute x and y coordinates
x = radius * np.cos(angles)
y = radius * np.sin(angles)

# Create a DataFrame
circle_df = pd.DataFrame({'x': x, 'y': y})

# Save to CSV
circle_df.to_csv('circle_points.csv', index=False)