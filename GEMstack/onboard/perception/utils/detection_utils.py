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