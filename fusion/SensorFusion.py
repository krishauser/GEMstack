import numpy as np
from scipy.spatial.transform import Rotation as R


def rotate_x(theta):
    c = np.cos(theta) ; s = np.sin(theta)
    return np.array([
        [ 1, 0, 0],
        [ 0, c,-s],
        [ 0, s, c],
    ])

def rotate_y(theta):
    c = np.cos(theta) ; s = np.sin(theta)
    return np.array([
        [ c, 0, s],
        [ 0, 1, 0],
        [-s, 0, c],
    ])

def rotate_z(theta):
    c = np.cos(theta) ; s = np.sin(theta)
    return np.array([
        [ c,-s, 0],
        [ s, c, 0],
        [ 0, 0, 1],
    ])

def cvtQuat2Rotate(quat):
    return R.from_quat(quat).as_matrix()

def cvtTF2TransMat(args):
    """
    x y z yaw pitch row
    """
    x, y, z, yaw, pitch, row = args
    T = np.eye(4)
    
    T[:3, :3] = rotate_z(yaw) @ rotate_y(pitch) @ rotate_x(row)
    T[:3, 3] = [x,y,z]
    return T

# rosrun tf tf_echo base_footprint os_sensor
T_OUSTER_2_BASE_FOOTPRINT = cvtTF2TransMat([-0.151, 0, 2.348, 0, 0, 0])

# rosrun tf tf_echo base_footprint livox_frame
# T_LIVOX_2_BASE_FOOTPRINT = cvtTF2TransMat([0.382, 0, 1.978, 0.02, 0.314, 0])
T_LIVOX_2_BASE_FOOTPRINT = cvtTF2TransMat([0.382, 0, 1.978, 0.02, 0.314, 0])

# rosrun tf tf_echo base_footprint front_camera_link
T_OAK_2_BASE_FOOTPRINT = cvtTF2TransMat([0.535, 0.150, 1.951, -1.57, 0, -1.57])
# T_OAK_2_BASE_FOOTPRINT = cvtTF2TransMat([0.535, 0.000, 1.951, 0, 0, 0])

# rostopic echo -n 1 -p /oak/rgb/image_raw
OAK_RGB_HEIGHT = 720
OAK_RGB_WIDTH = 1152
OAK_RGB_SHAPE = (OAK_RGB_HEIGHT, OAK_RGB_WIDTH)

OAK_RGB_INTRINSIC = np.array([
    [685,   0, OAK_RGB_WIDTH /2],
    [  0, 685, OAK_RGB_HEIGHT/2],
    [  0,   0,                1]
])

OAK_RGB_EXTRINSIC = np.linalg.inv(T_OAK_2_BASE_FOOTPRINT)


if __name__ == '__main__':

    print(f"T_OUSTER_2_BASE_FOOTPRINT : \n{T_OUSTER_2_BASE_FOOTPRINT}")
    print(f"T_LIVOX_2_BASE_FOOTPRINT : \n{T_LIVOX_2_BASE_FOOTPRINT}")
    print(f"T_OAK_2_BASE_FOOTPRINT : \n{T_OAK_2_BASE_FOOTPRINT}")
    
    # np.save("top_lidar_2_vehicle.npy", T_OUSTER_2_BASE_FOOTPRINT)
    # mat = np.load("top_lidar_2_vehicle.npy")
    # print(mat)



"""
roscore
rosbag play -l logs/three_02_05_25/vehicle.bag
roslaunch fusion sensors.launch 

cd fusion
python3 pointcloud2image.py
"""






