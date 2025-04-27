import rospy
from sensor_msgs.msg import Image, PointCloud2, CameraInfo
import cv2
from cv_bridge import CvBridge
import numpy as np
import os
import sensor_msgs.point_cloud2 as pc2

# Global variables
bridge = CvBridge()
lidar_points = None
camera_image = None

# Callback for LiDAR data
def lidar_callback(msg):
    global lidar_points
    lidar_points = msg

# Callback for camera image
def camera_callback(msg):
    global camera_image
    camera_image = msg

# Convert PointCloud2 to numpy array
def pc2_to_numpy(pc2_msg):
    return np.array(list(pc2.read_points(pc2_msg, skip_nans=True)), dtype=np.float32)[:, :3]

# Save data
def save_data(lidar_file, image_file):
    global lidar_points, camera_image
    if lidar_points and camera_image:
        # Save LiDAR data
        lidar_np = pc2_to_numpy(lidar_points)
        np.savez(lidar_file, lidar_np)
        # Save Image data
        cv_image = bridge.imgmsg_to_cv2(camera_image, "bgr8")
        cv2.imwrite(image_file, cv_image)
        print(f"Saved: {lidar_file}, {image_file}")

# Main function
def main(output_dir='home/manan/CS588/GEMstack/GEMstack/offboard/calibration'):
    rospy.init_node('capture_ouster_oak', anonymous=True)
    rospy.Subscriber('/oak/rgb/image_raw', Image, camera_callback)
    rospy.Subscriber('/ouster/points', PointCloud2, lidar_callback)

    if not os.path.exists(output_dir):
        os.makedirs(output_dir)

    index = 0
    print("Press 's' to save data or 'q' to quit.")
    
    while not rospy.is_shutdown():
        if camera_image is not None:
            cv2.imshow("Camera Image", bridge.imgmsg_to_cv2(camera_image, "bgr8"))
            key = cv2.waitKey(30)
            if key == ord('s'):  # Save data on 's' key press
                save_data(f"{output_dir}/lidar_{index}.npz", f"{output_dir}/image_{index}.png")
                index += 1
            elif key == ord('q'):  # Quit on 'q' key press
                break

if __name__ == "__main__":
    main()
