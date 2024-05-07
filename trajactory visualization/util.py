import rosbag
import cv2
import numpy as np
from cv_bridge import CvBridge

# Define camera parameters
reference_point = np.array([0, 1.78, 1.58])  # rear axle center
# rotation_matrix = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]])  # Rotation matrix mapping z to forward, x to left, y to down
rotation_matrix = np.array([[-1, 0, 0], [0, 0, 1],[0, -1, 0]])

# Define the camera intrinsic matrix
# distortion_model: "rational_polynomial"
# D: [0.908803403377533, 0.10777730494737625, -0.00011555181117728353, 0.00016320882423315197, 0.021386317908763885, 0.8666334748268127, 0.18107722699642181, 0.012254921719431877]
# K: [684.8333129882812, 0.0, 573.37109375, 0.0, 684.6096801757812, 363.700927734375, 0.0, 0.0, 1.0]
# R: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
# P: [684.8333129882812, 0.0, 573.37109375, 0.0, 0.0, 684.6096801757812, 363.700927734375, 0.0, 0.0, 0.0, 1.0, 0.0]

# Distortion coefficients
D = np.array([0.908803403377533, 0.10777730494737625, -0.00011555181117728353, 0.00016320882423315197, 0.021386317908763885, 0.8666334748268127, 0.18107722699642181, 0.012254921719431877])
# Camera intrinsic matrix
K = np.array([684.8333129882812, 0.0, 573.37109375, 0.0, 684.6096801757812, 363.700927734375, 0.0, 0.0, 1.0])
# Rotation matrix
R = np.array([1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0])
# Projection matrix
P = np.array([684.8333129882812, 0.0, 573.37109375, 0.0, 0.0, 684.6096801757812, 363.700927734375, 0.0, 0.0, 0.0, 1.0, 0.0])

def coordinate_transform(point):
    # Transform the point from the camera frame to the vehicle frame
    # the input point is in the vehicle frame
    # the output point is in the camera frame
    return np.dot(rotation_matrix, point - reference_point)
def inverse_coordinate_transform(point):
    rotation_matrix_inv = np.linalg.inv(rotation_matrix)
    return np.dot(rotation_matrix_inv, point) + reference_point

# [u v w]' = P * [X Y Z 1]'
#        x = u / w
#        y = v / w
def project_to_image(point):
    global K, P
    fx = K[0]
    cx = K[2]
    fy = K[4]
    cy = K[5]
    fxx = P[0]
    cxx = P[2]
    fyy = P[5]
    cyy = P[6]
    # Project the point to the image plane
    X, Y, Z = point
    A = np.transpose(np.array([X, Y, Z, 1]))
    PP = np.array([[fxx, 0, cxx, P[3]],
                  [0, fyy, cyy, P[7]],
                  [0, 0, 1, 0]])
    B = np.dot(PP, A)
    x = B[0] / B[2]
    y = B[1] / B[2]
    return x, y

# pinhole camera model
def project_to_image_pinhole(point):
    global K
    fx = K[0]
    cx = K[2]
    fy = K[4]
    cy = K[5]
    # Project the point to the image plane
    X, Y, Z = point
    x = fx * X / Z + cx
    y = fy * Y / Z + cy
    return x, y

def read_planned_trajectory(file_path):
    # Read the planned trajectory data of the vehicle and return the trajectory data
    # Here, we assume that the trajectory data is read from a text file, with each line containing a pair of (x, y, z) coordinates
    with open(file_path, 'r') as file:
        lines = file.readlines()
        # Read each line and convert the coordinates to floating point numbers
        trajectory = [tuple(map(float, line.split())) for line in lines]
    return trajectory

def process_image(image, trajectory): # image was stored in the rosbag file, topic = /oak/rgb/image_raw
    # Overlay the trajectory on the image and return the image with the trajectory overlaid
    # Convert the image parameter to the cv::UMat type
    image_um = cv2.UMat(image)
    for point in trajectory:
        x, y, z = point
        # Transform the point from the camera frame to the vehicle frame
        point_vehicle_frame = inverse_coordinate_transform(np.array([x, y, z]))
        # Project the point to the image plane
        point_image = project_to_image_pinhole(point_vehicle_frame)
        u, v = map(int, point_image)
        # Draw a red circle on the image to represent the trajectory point
        cv2.circle(image_um, (u, v), 5, (0, 0, 255), -1)
    return image_um.get()  # Convert the cv::UMat type back to a NumPy array and return

def generate_video(frames, trajectory):
    # Generate the video and return the video file path
    video_path = 'op.avi'
    frame_rate = 30

    # Create a video writer object
    first_msg = next(frames)[1]
    bridge = CvBridge()
    first_frame = bridge.imgmsg_to_cv2(first_msg, desired_encoding="passthrough")
    height, width, _ = first_frame.shape
    out = cv2.VideoWriter(video_path, cv2.VideoWriter_fourcc(*'XVID'), frame_rate, (width, height))

    # Process each frame and write it to the video
    for _, msg, _ in frames:
        frame = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        # Overlay the trajectory on the image
        frame_with_trajectory = process_image(frame, trajectory)
        out.write(frame_with_trajectory)

    # Release the video writer object and return the video file path
    out.release()
    return video_path

if __name__ == "__main__":
    # Read the ROS bag file path
    bag_file_path = '2024-04-23-18-00-31.bag'

    # Read the ROS bag file
    bag = rosbag.Bag(bag_file_path)

    # Read the planned trajectory data of the vehicle
    planned_trajectory = read_planned_trajectory('planned_trajectory1.txt')
    # print the trajectory
    print(planned_trajectory)
    # Read the image frames from the ROS bag
    frames = bag.read_messages(topics=['/oak/rgb/image_raw'])
    generate_video(frames, planned_trajectory)
