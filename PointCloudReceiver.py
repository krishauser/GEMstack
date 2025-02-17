import rospy
import sensor_msgs.point_cloud2 as pc2
import numpy as np
from sensor_msgs.msg import PointCloud2


class PointCloudReceiver:    
    def __init__(self, ouster_file="ouster_pointcloud", livox_file="livox_pointcloud"):
        rospy.init_node("pointcloud_receiver", anonymous=True)
        self.node_name = rospy.get_name()

        # 存储文件名
        self.ouster_file = ouster_file
        self.livox_file = livox_file

        # 点云数据
        self.ouster_data = None
        self.livox_data = None

        # 订阅两个点云话题
        rospy.Subscriber("/ouster/points", PointCloud2, self.ouster_callback)
        rospy.Subscriber("/livox/lidar", PointCloud2, self.livox_callback)

        rospy.loginfo("Waiting for point cloud data from /ouster/points and /livox/lidar...")
        rospy.spin()  # 保持节点运行

    def ouster_callback(self, msg):
        rospy.loginfo("Received Ouster point cloud!")

        # 解析 `PointCloud2` 数据
        points = self.pointcloud2numpy(msg)
        self.ouster_data = points
        
        np.save(self.ouster_file + ".npy", points)  # 保存 .npy
        np.savez(self.ouster_file + ".npz", points)  # 另存 .npz

        rospy.loginfo(f"{points.shape[0]} Ouster points saved to {self.ouster_file}.npy")
        
    def livox_callback(self, msg):
        rospy.loginfo("Received Livox point cloud!")

        # 解析 `PointCloud2` 数据
        points = self.pointcloud2numpy(msg)
        self.livox_data = points
        
        np.save(self.livox_file + ".npy", points)  # 保存 .npy
        np.savez(self.livox_file + ".npz", points)  # 另存 .npz

        rospy.loginfo(f"{points.shape[0]} Livox points saved to {self.livox_file}.npy")

    def pointcloud2numpy(self, msg):
        points_list = []
        for point in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            points_list.append([point[0], point[1], point[2]])
        return np.array(points_list, dtype=np.float32)


if __name__ == "__main__":
    receiver = PointCloudReceiver("ouster_pointcloud", "livox_pointcloud")
