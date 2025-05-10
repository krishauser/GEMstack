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


def cvtFootInch2Meter(ft, inch=0.0):
    return (12 * ft + inch) * 0.0254

pixelFrontRightCamListXY = np.array([
    [87,670],  # FL
    [1625,777],  # FR
    [260,855],  # RL
    [1460,966],  # RR
])

worldFrontRightCamListXY = np.array([
    [cvtFootInch2Meter(0,  35+30    -20), cvtFootInch2Meter(0, -210-170)-1.46+0.151],  # FL
    [cvtFootInch2Meter(0, 250+30), cvtFootInch2Meter(0, 0   -10)-1.46+0.151],  # FR
    [cvtFootInch2Meter(0,  35+30    -10),cvtFootInch2Meter(0, -170)-1.46+0.151],  # RL
    [cvtFootInch2Meter(0, 140+30), cvtFootInch2Meter(0, 0   -5)-1.46+0.151],  # RR
])

def cvtOriginImgPixel2DToVehicleFrameMeter3D(TransMat, pixelPointXY):
    """
    pixelPointXY: (u, v) in original image (pixel)
    TransMat: 3x3 perspective transform matrix (pixel -> real world meter)
    """
    BASE_VEHICLE_DIST = 1.10 
    
    point = np.array([pixelPointXY[0], pixelPointXY[1], 1.0], dtype=np.float32).reshape(3, 1)
    
    transformed = TransMat @ point
    transformed /= transformed[2, 0]  

    x_real = transformed[0, 0] 
    y_real = transformed[1, 0] 
    z_real = 0.0 

    x_pc = -y_real
    y_pc = -x_real
    z_pc = z_real

    return np.array([x_pc + BASE_VEHICLE_DIST, y_pc, z_pc], dtype=np.float32)


def fr_cam_2d_to_vehicle_3d(fr_2d_pt):
    TransMat = cv2.getPerspectiveTransform( np.float32(pixelFrontRightCamListXY), 
                                            np.float32(worldFrontRightCamListXY))
    
    result = cvtOriginImgPixel2DToVehicleFrameMeter3D(TransMat, fr_2d_pt)
    return result.tolist()