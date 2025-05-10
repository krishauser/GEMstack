import rospy
import sensor_msgs.point_cloud2 as pc2
import numpy as np
import cv2
from sensor_msgs.msg import PointCloud2, Image
from cv_bridge import CvBridge
import message_filters

class SensorReceiverSync:
    def __init__(self, ouster_file="ouster_pointcloud", livox_file="livox_pointcloud", image_file="oak_image"):
        rospy.init_node("synchronized_sensor_receiver", anonymous=True)

        # 存储文件名
        self.ouster_file = ouster_file
        self.livox_file = livox_file
        self.image_file = image_file

        # 创建 CvBridge 实例，用于转换 ROS Image 到 OpenCV 格式
        self.bridge = CvBridge()

        # 订阅 Ouster 点云、Livox 点云和 RGB 图像
        ouster_sub = message_filters.Subscriber("/ouster/points", PointCloud2)
        livox_sub = message_filters.Subscriber("/livox/lidar", PointCloud2)
        image_sub = message_filters.Subscriber("/oak/rgb/image_raw", Image)

        # 近似时间同步器（允许 0.1s 误差）
        sync = message_filters.ApproximateTimeSynchronizer(
            [ouster_sub, livox_sub, image_sub], queue_size=10, slop=0.1
        )
        sync.registerCallback(self.sync_callback)

        rospy.loginfo("Waiting for synchronized Ouster, Livox point clouds and Oak RGB image...")
        rospy.spin()
        

    def sync_callback(self, ouster_msg, livox_msg, image_msg):
        rospy.loginfo("Received synchronized Ouster & Livox point clouds + RGB image!")

        # Ouster
        ouster_points = self.pointcloud2numpy(ouster_msg)
        np.save(self.ouster_file + ".npy", ouster_points)
        np.savez(self.ouster_file + ".npz", ouster_points)

        # Livox
        livox_points = self.pointcloud2numpy(livox_msg)
        np.save(self.livox_file + ".npy", livox_points)
        np.savez(self.livox_file + ".npz", livox_points)

        # RGB
        image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding="bgr8")
        cv2.imwrite(self.image_file + ".jpg", image)
        np.save(self.image_file + ".npy", image)

        rospy.loginfo(f"{ouster_points.shape[0]} Ouster points saved to {self.ouster_file}.npy")
        rospy.loginfo(f"{livox_points.shape[0]} Livox points saved to {self.livox_file}.npy")
        rospy.loginfo(f"RGB image saved to {self.image_file}.jpg and {self.image_file}.npy")

        rospy.signal_shutdown("Saved point clouds & image, shutting down node.")
        
    def pointcloud2numpy(self, msg):
        points_list = []
        for point in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            points_list.append([point[0], point[1], point[2]])
        return np.array(points_list, dtype=np.float32)

if __name__ == "__main__":
    receiver = SensorReceiverSync("data_new/ouster_pointcloud", "data_new/livox_pointcloud", "data_new/oak_image")
