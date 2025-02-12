import rospy
import tf

def publish_tf():
    rospy.init_node('pointcloud_tf_broadcaster')
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        br.sendTransform(
            (0, 0, 1),  # (x, y, z) translation
            tf.transformations.quaternion_from_euler(0, 0, 0),  # (roll, pitch, yaw)
            rospy.Time.now(),
            "oak_rgb_camera_optical_frame",  # Child frame (sensor)
            "map"  # Parent frame (world)
        )
        rate.sleep()

if __name__ == "__main__":
    publish_tf()