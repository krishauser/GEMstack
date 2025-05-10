import numpy as np

def is_valid_multiple_of_k(n, k):
    return n > k and n % k == 0

def group_points_by_multiple_of_k(points, k):
    if len(points) % k != 0:
        raise ValueError(f"Number of points must be a multiple of {k}.")
    return [points[i:i+k] for i in range(0, len(points), k)]

def closest_point_to_origin(points):
    if not points:
        return None
    points_array = np.array(points)
    distances = np.linalg.norm(points_array, axis=1)
    min_index = np.argmin(distances)
    return tuple(points_array[min_index])