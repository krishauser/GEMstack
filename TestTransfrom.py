import numpy as np
from scipy.spatial.transform import Rotation

def create_transformation_matrix(translation, euler_angles):
    """
    euler_angles : yaw, pitch, roll
    """
    rotation_matrix = Rotation.from_euler('ZYX', euler_angles).as_matrix()
    # {X, Y, Z} = intrinsic rotation (current frame)
    # extrinsic rotation (fixed frame)
    
    T = np.eye(4)
    T[:3, :3] = rotation_matrix
    T[:3, 3] = translation
    return T

# x y z | yaw pitch roll

# base_link -> os_sensor (no rotation)
T_os_sensor = create_transformation_matrix([0, 0, 2.23], [0, 0, 0])

# base_link -> livox_frame (yaw z=0, pitch y=0.314, roll x=0.02)
T_livox = create_transformation_matrix([0.56, 0, 1.84], [0.02, 0.314, 0])

print("T_base_link_to_os_sensor:\n", T_os_sensor)
print("\nT_base_link_to_livox_frame:\n", T_livox)


# <node pkg="tf2_ros" type="static_transform_publisher" name="tf_os_sensor"
#     args="0 0 2.23 0 0 0 base_link os_sensor" />
# <!-- base_link -> livox_frame -->
# <node pkg="tf2_ros" type="static_transform_publisher" name="tf_livox"
#     args="0.56 0 1.84 0.02 0.314 0 base_link livox_frame" />


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


print(rotate_x(0) @ rotate_y(0.314) @ rotate_z(0.02))

print(rotate_z(0.02) @ rotate_y(0.314) @ rotate_x(0))

