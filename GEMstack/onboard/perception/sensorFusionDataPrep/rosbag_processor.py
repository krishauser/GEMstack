import pypcd
import rospy
import message_filters
from sensor_msgs.msg import Image, PointCloud2
import cv2
# import os
from cv_bridge import CvBridge

import numpy as np
np.float = np.float64  # temp fix for the following import
import ros_numpy

from pathlib import Path


class RosbagProcessor():
    # Pairs up and stores Image and PointCloud2 messages into their respective test (20%) and training (80%) png and bin files.
    def __init__(self, DATA_DIR: str, train_counter: int = 0, test_counter: int = 0) -> None:
        self.__img_folder = 'image_2'
        self.__lidar_folder = 'velodyne'

        self.__TEST_DIR = DATA_DIR / 'testing'
        self.__TRAIN_DIR = DATA_DIR / 'training'
        TEST_IMG_DIR = self.__TEST_DIR / self.__img_folder
        TEST_BIN_DIR = self.__TEST_DIR / self.__lidar_folder
        TRAIN_IMG_DIR = self.__TRAIN_DIR / self.__img_folder
        TRAIN_BIN_DIR = self.__TRAIN_DIR / self.__lidar_folder

        # Create directories if they don't exist
        TEST_IMG_DIR.mkdir(parents=True, exist_ok=True)
        TEST_BIN_DIR.mkdir(parents=True, exist_ok=True)
        TRAIN_IMG_DIR.mkdir(parents=True, exist_ok=True)
        TRAIN_BIN_DIR.mkdir(parents=True, exist_ok=True)

        self.__train_counter = train_counter
        self.__test_counter = test_counter
        self.__bridge = CvBridge()

        # Subscribers and sychronizers
        rospy.init_node('bagListener', anonymous=True) 
        self.rgb_rosbag = message_filters.Subscriber('/oak/rgb/image_raw', Image)
        self.top_lidar_rosbag = message_filters.Subscriber('/ouster/points', PointCloud2)
        self.sync = message_filters.ApproximateTimeSynchronizer([self.rgb_rosbag, self.top_lidar_rosbag], queue_size=10, slop=0.1)
        self.sync.registerCallback(self.pair_data_callback)

        rospy.spin()

    def pair_data_callback(self, rgb_image_msg: Image, lidar_pc2_msg: PointCloud2):
        # Divide up 20% of the data for testing:
        if (self.__train_counter + self.__test_counter) % 5 == 0:
            PARENT_FOLDER = self.__TEST_DIR
            idx = self.__test_counter    
        else:
            PARENT_FOLDER = self.__TRAIN_DIR
            idx = self.__train_counter

        # Store point cloud and image data with KITTI formatted filenames:
        image_filename = PARENT_FOLDER / self.__img_folder / f"{idx:06d}.png"
        pc_filename = PARENT_FOLDER / self.__lidar_folder / f"{idx:06d}.bin"
        # image_filename = self.get_path(image_filename)
        # pc_filename = self.get_path(pc_filename)

        # Read and write the received image into a KITTI specified png file
        cv_image = self.__bridge.imgmsg_to_cv2(rgb_image_msg, desired_encoding='bgr8')
        cv2.imwrite(image_filename, cv_image)

        # Read and write the lidar data into a KITTI specified bin file:
        # Convert to numpy array and flatten it
        pc_array = ros_numpy.point_cloud2.pointcloud2_to_array(lidar_pc2_msg)
        pc_array = pc_array.reshape(-1)

        # Create (N, 4) array and fill it
        points = np.zeros((pc_array.shape[0], 4), dtype=np.float32)
        points[:, 0] = pc_array['x']
        points[:, 1] = pc_array['y']
        points[:, 2] = pc_array['z']

        # Check for intensity just incase:
        if 'intensity' in pc_array.dtype.names:
            points[:, 3] = pc_array['intensity']
        else:
            rospy.loginfo(f"Intensity was set to zero")
            points[:, 3] = 0.0

        # Save as binary file
        with open(pc_filename, 'wb') as f:
            f.write(points.tobytes())
        
        if (self.__train_counter + self.__test_counter) % 5 == 0:
            rospy.loginfo(f"Saved test image and point cloud #{str(self.__test_counter)}")
            self.__test_counter += 1
        else:
            rospy.loginfo(f"Saved train image and point cloud #{str(self.__train_counter)}")
            self.__train_counter += 1


if __name__ == '__main__': 
    BASE_DIR = Path(__file__).resolve().parent

    # Path to the destination folder (relative from this script)
    DATA_DIR = BASE_DIR.parent.parent.parent.parent / 'data' / 'sensorFusionData'
    DATA_DIR.mkdir(parents=True, exist_ok=True)
    bag_processor = RosbagProcessor(DATA_DIR=DATA_DIR)
