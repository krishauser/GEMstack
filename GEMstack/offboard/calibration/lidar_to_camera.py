import numpy as np

T_lidar_vehicle = np.array([[ 0.99939639,  0.02547917,  0.023615,    1.1       ],
                            [-0.02530848,  0.99965156, -0.00749882,  0.03773583],
                            [-0.02379784,  0.00689664,  0.999693,    1.95320223],
                            [ 0.,          0.,          0.,          1.        ]])

T_camera_vehicle = np.array([[-0.025131,   -0.0304479,   0.99922038,  1.78251567],
                             [-0.99892897,  0.0396095,  -0.0239167,   0.18649424],
                             [-0.0388504,  -0.99875123, -0.03141071,  1.5399846 ],
                             [ 0.,          0.,          0.,          1.        ]])

# Invert Camera->Vehicle transformation
T_vehicle_camera = np.linalg.inv(T_camera_vehicle)

T_lidar_camera = T_vehicle_camera @ T_lidar_vehicle

# Print result
print("Lidar to Camera Transformation Matrix:")
print(T_lidar_camera)
