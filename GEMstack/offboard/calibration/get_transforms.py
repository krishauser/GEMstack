import numpy as np
#We want the transforms T_zed^vehicle and T_velodyne^vehicle.

T_velo_vehicle =  np.array([[1, 0, 0, 0.9],   # Transformation matrix Velordyne to Vehicle
                            [0, 1, 0, 0],
                            [0, 0, 1, 1.62],
                            [0, 0, 0, 1]])


V_Z_R =np.array( [[ 0.05889591 ,-0.99814647,  0.01532603], #Rotation matrix Velodyne to Zed
 [-0.21114352 ,-0.02746099, -0.97706924],
 [ 0.97567909 , 0.05430939 ,-0.2123695 ]]
)

V_Z_T= np.array( [ 0.04247593 , 0.44027576, -0.23995718]) #Translation matrix Velodyne to Zed

T_velo_zed = np.eye(4)
T_velo_zed[:3, :3] = V_Z_R
T_velo_zed[:3, 3] = V_Z_T


T_zed_velo = np.linalg.inv(T_velo_zed)
print("T_velo_vehicle", T_velo_vehicle)
print("T_zed_velo", T_zed_velo)

T_zed_vehicle = np.dot(T_velo_vehicle, T_zed_velo)

print("T_zed_vehicle",T_zed_vehicle)
