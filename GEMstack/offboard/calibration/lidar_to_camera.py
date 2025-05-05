import numpy as np

T_camera_vehicle = np.array([[ 0.00349517, -0.03239524, 0.99946903, 1.75864913],
          [-0.99996547,  0.00742285,  0.0037375, 0.01238124],
          [-0.00753999, -0.99944757, -0.03236817, 1.54408419],
          [0,0,0,1]])

T_lidar_vehicle = np.array([[ 0.99941328,  0.02547416,  0.02289458, 1.1],
                            [-0.02530855,  0.99965159, -0.00749488, 0.03773044170906172],
                            [-0.02307753,  0.00691106,  0.99970979, 1.9525244316515322],
                            [0,0,0,1]])

# Invert Camera->Vehicle transformation
T_vehicle_camera = np.linalg.inv(T_camera_vehicle)

T_lidar_camera = T_vehicle_camera @ T_lidar_vehicle

# Print result
print("Lidar to Camera Transformation Matrix:")
print(T_lidar_camera)
