import rospy
import sensor_msgs.point_cloud2 as pc2
import numpy as np
from sensor_msgs.msg import PointCloud2
import message_filters  # 用于时间同步

class SynchronizedPointCloudReceiver:
    def __init__(self, ouster_file="ouster_pointcloud", livox_file="livox_pointcloud"):
        rospy.init_node("synchronized_pointcloud_receiver", anonymous=True)
        self.node_name = rospy.get_name()

        # 存储文件名
        self.ouster_file = ouster_file
        self.livox_file = livox_file

        # 创建两个订阅者
        ouster_sub = message_filters.Subscriber("/ouster/points", PointCloud2)
        livox_sub = message_filters.Subscriber("/livox/lidar", PointCloud2)

        # 近似时间同步器
        sync = message_filters.ApproximateTimeSynchronizer(
            [ouster_sub, livox_sub], queue_size=10, slop=0.1
        )
        sync.registerCallback(self.sync_callback)

        rospy.loginfo("Waiting for synchronized point cloud data...")
        rospy.spin()  # 保持节点运行

    def sync_callback(self, ouster_msg, livox_msg):
        rospy.loginfo("Received synchronized Ouster & Livox point clouds!")

        # 解析 Ouster 点云
        ouster_points = self.pointcloud2numpy(ouster_msg)
        np.save(self.ouster_file + ".npy", ouster_points)
        np.savez(self.ouster_file + ".npz", ouster_points)

        # 解析 Livox 点云
        livox_points = self.pointcloud2numpy(livox_msg)
        np.save(self.livox_file + ".npy", livox_points)
        np.savez(self.livox_file + ".npz", livox_points)

        rospy.loginfo(f"{ouster_points.shape[0]} Ouster points saved to {self.ouster_file}.npy")
        rospy.loginfo(f"{livox_points.shape[0]} Livox points saved to {self.livox_file}.npy")

    def pointcloud2numpy(self, msg):
        points_list = []
        for point in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            points_list.append([point[0], point[1], point[2]])
        return np.array(points_list, dtype=np.float32)

if __name__ == "__main__":
    receiver = SynchronizedPointCloudReceiver("ouster_pointcloud", "livox_pointcloud")
