import math
import numpy as np
from scipy.spatial.transform import Rotation as R

class Pose:
    def __init__(self, x=0, y=0, z=0, yaw=0, pitch=0, roll=0):
        self.x = x
        self.y = y
        self.z = z
        self.yaw = yaw
        self.pitch = pitch
        self.roll = roll

def pose_to_matrix(pose):
    """
    Compose a 4x4 transformation matrix from a pose state.
    Assumes pose has attributes: x, y, z, yaw, pitch, roll (angles in degrees).
    """
    x = pose.x or 0.0
    y = pose.y or 0.0
    z = pose.z or 0.0
    yaw   = math.radians(pose.yaw or 0.0)
    pitch = math.radians(pose.pitch or 0.0)
    roll  = math.radians(pose.roll or 0.0)

    R_mat = R.from_euler('zyx', [yaw, pitch, roll]).as_matrix()
    T = np.eye(4)
    T[:3, :3] = R_mat
    T[:3, 3]  = [x, y, z]
    return T

# A helper to transform a point and compare to expected
def test_transform(pose, point_vehicle, expected_world):
    T = pose_to_matrix(pose)
    hom_pt = np.append(point_vehicle, 1.0)
    pt_world = (T @ hom_pt)[:3]
    print(f"Pose: x={pose.x}, y={pose.y}, yaw={pose.yaw}°")
    print(f" Transformed: {pt_world.round(3)}   Expected: {np.array(expected_world)}\n")

# Define the point in vehicle frame (1 m in front along x)
point = [1, 0, 0]

# Test cases:
poses = [
    (Pose(0, 0, 0, 0),       [1, 0, 0]),  # identity → point stays at (1,0,0)
    (Pose(1, 0, 0, 0),       [2, 0, 0]),  # translate +1x → (1+1,0,0)
    (Pose(1, 0, 0, 45),      [1, 1, 0]),  # then yaw 90° around z → front→world+y
    (Pose(1, 2, 0, 180),     [0, 2, 0]),  # yaw 180°: front→-x, plus translation
    (Pose(0, 0, 0, -90),     [0, -1, 0])  # yaw -90°: front→world -y
]

for p, exp in poses:
    test_transform(p, point, exp)
