import json
import matplotlib.pyplot as plt
import matplotlib.dates as mdates
from datetime import datetime

# Read the NDJSON data
data = []
with open('test1/behavior.json', 'r') as f:
    for line in f:
        if line.strip():
            data.append(json.loads(line))

# Extract fields
x_vals = []
y_vals = []
speeds = []
times = []

for entry in data:
    # Skip if position is missing
    if 'position' not in entry or 'vehicle_interface_reading' not in entry:
        continue
    
    pos = entry['position']
    x_vals.append(pos.get('x', 0.0))
    y_vals.append(pos.get('y', 0.0))
    
    speeds.append(entry['vehicle_interface_reading']['data'].get('speed', 0.0))
    times.append(datetime.fromtimestamp(entry['time']))

# Plotting
fig, ax = plt.subplots(figsize=(10, 6))
sc = ax.scatter(x_vals, y_vals, c=speeds, cmap='plasma', s=30, edgecolor='k')
cbar = plt.colorbar(sc, ax=ax)
cbar.set_label('Speed (m/s)')

ax.set_title('Vehicle Position Colored by Speed')
ax.set_xlabel('X Position (m)')
ax.set_ylabel('Y Position (m)')
ax.grid(True)

plt.tight_layout()
plt.show()