import numpy as np

# Example: list of cone positions in vehicle frame (X forward, Y left, Z up)
cones = np.array([
    [10.17, 10.62],
    [12.44, 11.22],
    [11.78, 7.60],
    [12.48, 7.63],
    [14.68, 7.54],
    [17.04, 7.53]
])

# Step 1: Find pairs of cones closest in Y (forward distance) to vehicle
# Sort by Y (smallest is closest if +Y is forward)
cones_sorted = cones[cones[:, 1].argsort()]

# Step 2: Assume first 2 cones are mouth
mouth = cones_sorted[:2]
rear = cones_sorted[2:4]  # next 2 furthest back

# Step 3: Midpoints of front and rear
mouth_center = np.mean(mouth, axis=0)
rear_center = np.mean(rear, axis=0)

# Step 4: Parking center is midpoint between mouth_center and rear_center
goal_pos = (mouth_center + rear_center) / 2

# Step 5: Orientation (yaw) is the angle from rear to mouth
delta = mouth_center - rear_center
yaw = np.arctan2(delta[1], delta[0])

print(f"Goal Position: x={goal_pos[0]:.2f}, y={goal_pos[1]:.2f}, yaw={np.degrees(yaw):.2f} deg")


import matplotlib.pyplot as plt

plt.scatter(cones[:, 0], cones[:, 1], c='red', label='Cones')
plt.scatter(*goal_pos, c='blue', label='Goal')
plt.arrow(goal_pos[0], goal_pos[1], 0.5*np.cos(yaw), 0.5*np.sin(yaw), 
          head_width=0.3, color='blue')

plt.legend()
plt.gca().set_aspect('equal')
plt.title('Parking Spot and Goal Pose')
plt.show()
