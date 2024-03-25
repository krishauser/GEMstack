import numpy as np
from libraries.hybridtest import hybrid_a_star_planning
import matplotlib.pyplot as plt
import csv

start = [0, 0, 0]
goal = [10, 10, np.pi/2]
# ox = [0,0,0,1,1,1,2,2,2,13,13,13]
# oy = [10,9,8,10,9,8,10,9,8,10,9,8]
ox = [5]
oy = [5]
# Call the function
path = hybrid_a_star_planning(start, goal, ox, oy)

path_coordinates = [(x, y, z) for x, y, z in zip(path.x_list, path.y_list, path.yaw_list)]
print("Path coordinates:")
for point in path_coordinates:
    print(point)


file_path = "../../knowledge/routes/hybrid_path.csv"
with open(file_path, 'w', newline='') as csvfile:
    writer = csv.writer(csvfile)
    for point in path_coordinates:
        writer.writerow(point)


x_values = [point[0] for point in path_coordinates]
y_values = [point[1] for point in path_coordinates]
plt.plot(x_values, y_values, marker='o')
plt.title('Path Visualization')
plt.xlabel('X')
plt.ylabel('Y')
plt.grid(True)
plt.show()