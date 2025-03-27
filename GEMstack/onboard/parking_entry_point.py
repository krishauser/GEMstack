from pacmod_msgs.msg import PositionWithSpeed, PacmodCmd
import rospy
from sensor_msgs.msg import Image,PointCloud2
import sensor_msgs.point_cloud2 as pc2
import ctypes
import struct
import argparse

def main():
    accel_pub = rospy.Publisher('/pacmod/as_rx/accel_cmd', PacmodCmd, queue_size=1)
    accel_cmd = PacmodCmd()
    accel_cmd.enable = True
    accel_cmd.clear  = False
    accel_cmd.ignore = False

    accel_cmd.f64_cmd = 0.4
    accel_pub.publish(accel_cmd)


    time.sleep(5)

    brake_pub = rospy.Publisher('/pacmod/as_rx/brake_cmd', PacmodCmd, queue_size=1)
    brake_cmd = PacmodCmd()
    brake_cmd.enable = True
    brake_cmd.clear  = False
    brake_cmd.ignore = False
    brake_cmd.f64_cmd = 1
    brake_pub.publish(brake_cmd)


if __name__ == "__main__":
    main()
