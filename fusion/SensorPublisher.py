import rospy
import numpy as np
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from SensorFusion import T_OUSTER_2_BASE_FOOTPRINT, T_LIVOX_2_BASE_FOOTPRINT


def transformPointCloudFrame(pointcloud, T):
    """转换点云到目标坐标系，点云形状 (N, 3)"""
    assert pointcloud.shape[1] == 3
    ROT = T[:3, :3]
    TRANS = T[:3, 3]
    return pointcloud @ ROT.T + TRANS  # 旋转 + 平移


def numpy_to_pointcloud2(points, frame_id="base_footprint"):
    """
    将 numpy (N, 3) 点云数据转换为 ROS PointCloud2 消息
    :param points: (N, 3) NumPy 数组，每行是 (x, y, z)
    :param frame_id: 话题的坐标系
    :return: ROS PointCloud2 消息
    """
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = frame_id

    fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]

    # 转换点云到 PointCloud2 格式
    return pc2.create_cloud(header, fields, points)


def publish_point_clouds():
    rospy.init_node("pointcloud_publisher", anonymous=True)

    # 创建 ROS 话题发布者
    ouster_pub = rospy.Publisher("/ouster/points_transformed", PointCloud2, queue_size=1)
    livox_pub = rospy.Publisher("/livox/points_transformed", PointCloud2, queue_size=1)

    rospy.loginfo("PointCloud Publisher Node Started!")

    # 读取点云数据
    ouster_points = np.load("data/ouster_pointcloud.npy")  # (N, 3)
    livox_points = np.load("data/livox_pointcloud.npy")  # (M, 3)

    # 变换到 base_footprint 坐标系
    ouster_in_base = transformPointCloudFrame(ouster_points, T_OUSTER_2_BASE_FOOTPRINT)
    livox_in_base = transformPointCloudFrame(livox_points, T_LIVOX_2_BASE_FOOTPRINT)

    # 转换为 ROS PointCloud2 消息
    ouster_msg = numpy_to_pointcloud2(ouster_in_base, "base_footprint")
    livox_msg = numpy_to_pointcloud2(livox_in_base, "base_footprint")

    # 发布点云
    rate = rospy.Rate(1)  # 1Hz 发布一次
    while not rospy.is_shutdown():
        ouster_pub.publish(ouster_msg)
        livox_pub.publish(livox_msg)
        rospy.loginfo("Published transformed point clouds to ROS topics.")
        rate.sleep()


if __name__ == "__main__":
    try:
        publish_point_clouds()
    except rospy.ROSInterruptException:
        pass
