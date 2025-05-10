import cv2
import ros_numpy
import numpy as np

def transform_points(points, transform):
    ones_column = np.ones((points.shape[0], 1))
    points_extended = np.hstack((points, ones_column))
    points_transformed = ((transform @ (points_extended.T)).T)
    return points_transformed[:, :3]
    

def filter_ground_points(lidar_points, ground_threshold = 0):
    filtered_array = lidar_points[lidar_points[:, 2] > ground_threshold]
    return filtered_array
    

def pc2_to_numpy(pc2_msg, want_rgb=False):
    # Convert the ROS message to a numpy structured array
    pc = ros_numpy.point_cloud2.pointcloud2_to_array(pc2_msg)
    # Stack x,y,z fields to a (N,3) array
    pts = np.stack((np.array(pc['x']).ravel(),
                    np.array(pc['y']).ravel(),
                    np.array(pc['z']).ravel()), axis=1)
    # Apply filtering (for example, x > 0 and z in a specified range)
    mask = (pts[:, 0] > 0) & (pts[:, 2] < -1.5) & (pts[:, 2] > -2.7)
    return pts[mask]


def is_parallelogram(approx, length_tolerance=0.2):
    if len(approx) != 4:
        return False

    if not cv2.isContourConvex(approx):
        return False

    # Extract the 4 points
    pts = [approx[i][0] for i in range(4)]

    # Compute side lengths
    def side_length(p1, p2):
        return np.linalg.norm(p1 - p2)

    lengths = [
        side_length(pts[0], pts[1]),  # Side 1
        side_length(pts[1], pts[2]),  # Side 2
        side_length(pts[2], pts[3]),  # Side 3
        side_length(pts[3], pts[0])   # Side 4
    ]

    # Check if opposite sides are approximately equal
    def is_close(l1, l2):
        return abs(l1 - l2) / max(l1, l2) < length_tolerance

    if not (is_close(lengths[0], lengths[2]) and is_close(lengths[1], lengths[3])):
        return False

    return True


def is_big_parallelogram(approx, min_area=1000, length_tolerance=0.2):
    if not is_parallelogram(approx, length_tolerance):
        return False
    area = cv2.contourArea(approx)
    return area >= min_area