import json
import matplotlib.pyplot as plt
from datetime import datetime
from bisect import bisect_left

# Read the file and separate pose and speed entries
pose_data = []
speed_data = []

with open("test1/behavior.json", "r") as f:
    for line in f:
        if not line.strip():
            continue
        entry = json.loads(line)
        if "vehicle" in entry:
            pose = entry["vehicle"]["data"]["pose"]
            timestamp = entry["time"]
            pose_data.append({
                "time": timestamp,
                "x": pose["x"],
                "y": pose["y"]
            })
        elif "vehicle_interface_reading" in entry:
            timestamp = entry["time"]
            speed = entry["vehicle_interface_reading"]["data"].get("speed", 0.0)
            speed_data.append({
                "time": timestamp,
                "speed": speed
            })

# Sort both lists by time
pose_data.sort(key=lambda x: x["time"])
speed_data.sort(key=lambda x: x["time"])
speed_times = [s["time"] for s in speed_data]

# Match each pose with the nearest speed
matched_x = []
matched_y = []
matched_speeds = []

for pose in pose_data:
    t = pose["time"]
    idx = bisect_left(speed_times, t)
    
    # Get closest speed entry within 0.01 seconds
    candidates = []
    if idx > 0:
        candidates.append(speed_data[idx - 1])
    if idx < len(speed_data):
        candidates.append(speed_data[idx])
    
    best_match = min(candidates, key=lambda s: abs(s["time"] - t), default=None)
    
    if best_match and abs(best_match["time"] - t) < 0.01:
        matched_x.append(pose["x"])
        matched_y.append(pose["y"])
        matched_speeds.append(best_match["speed"])

# Plot
plt.figure(figsize=(10, 6))
sc = plt.scatter(matched_x, matched_y, c=matched_speeds, cmap='viridis', edgecolor='k', s=30)
plt.colorbar(sc, label='Speed (m/s)')
plt.xlabel("X Position (m)")
plt.ylabel("Y Position (m)")
plt.title("Vehicle Position Colored by Speed")
plt.grid(True)
plt.tight_layout()
plt.show()