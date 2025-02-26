import rospy
import tf
import numpy as np

def publish_tf():
    rospy.init_node('pointcloud_tf_broadcaster')
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        br.sendTransform(
            (0, 0, 0),  # (x, y, z) translation
            tf.transformations.quaternion_from_euler(0, 0, 0),  # (roll, pitch, yaw)
            rospy.Time.now(),
            "os_sensor",  # Child frame (sensor)
            "map"  # Parent frame (world)
        )
        rate.sleep()

if __name__ == "__main__":
    publish_tf()