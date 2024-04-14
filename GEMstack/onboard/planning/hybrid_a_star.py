import numpy as np
from libraries.hybridtest import hybrid_a_star_planning
import matplotlib.pyplot as plt
import csv

#start and end positions
#the right is negative and left is positive
start = [0, 0, 0]
# goal = [10, 10, np.pi/2]
goal = [5, 5, np.pi/2]
# goal = [0, 0, np.pi/2]

#obstacle list
ox = [100]
oy = [100]
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