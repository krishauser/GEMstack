from pacmod_msgs.msg import PositionWithSpeed, PacmodCmd, VehicleSpeedRpt
import rospy
from sensor_msgs.msg import Image,PointCloud2
import sensor_msgs.point_cloud2 as pc2
import ctypes
import struct
import argparse
import time


speed = 0

def speed_callback(msg):
    global speed
    speed = msg.vehicle_speed
    rospy.loginfo(f"Vehicle Speed: {msg.vehicle_speed} m/s | Valid: {msg.vehicle_speed_valid}")

def main():

    rospy.init_node("straight", anonymous=True)
    accel_pub = rospy.Publisher('/pacmod/as_rx/accel_cmd', PacmodCmd, queue_size=1)
    accel_cmd = PacmodCmd()
    accel_cmd.enable = True
    accel_cmd.clear  = False
    accel_cmd.ignore = False

    brake_pub = rospy.Publisher('/pacmod/as_rx/brake_cmd', PacmodCmd, queue_size=1)
    brake_cmd = PacmodCmd()
    brake_cmd.enable = True
    brake_cmd.clear  = False
    brake_cmd.ignore = False
    brake_cmd.f64_cmd = 1

    speed_sub = rospy.Subscriber('/pacmod/parsed_tx/vehicle_speed_rpt', VehicleSpeedRpt, speed_callback)

    while True:
        # accel_cmd.f64_cmd = 0.4
        # accel_pub.publish(accel_cmd)
        if speed >= 1.5:
            accel_cmd.f64_cmd = 0
            accel_pub.publish(accel_cmd)
        if speed <= 1:
            accel_cmd.f64_cmd = 0.4
            accel_pub.publish(accel_cmd)


            # brake_cmd.f64_cmd = 0.5
            # brake_pub.publish(brake_cmd)
    
    



    # time.sleep(5)

    # brake_pub = rospy.Publisher('/pacmod/as_rx/brake_cmd', PacmodCmd, queue_size=1)
    # brake_cmd = PacmodCmd()
    # brake_cmd.enable = True
    # brake_cmd.clear  = False
    # brake_cmd.ignore = False
    # brake_cmd.f64_cmd = 1
    # brake_pub.publish(brake_cmd)


if __name__ == "__main__":
    main()

