from pacmod_msgs.msg import PositionWithSpeed, PacmodCmd, VehicleSpeedRpt
import rospy
from sensor_msgs.msg import Image,PointCloud2
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Float32MultiArray
import ctypes
import struct
import argparse
import time


speed = 0
detected_space = False
endpoints = []

def speed_callback(msg):
    global speed
    speed = msg.vehicle_speed
    rospy.loginfo(f"Vehicle Speed: {msg.vehicle_speed} m/s | Valid: {msg.vehicle_speed_valid}")

def parking_space_callback(msg):
    global detected_space
    global endpoints
    detected_space = true
    endpoints = msg
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
    detected_space_sub = rospy.Subscriber('/detected_space', Float32MultiArray, parking_space_callback)

    while not detected_space: # State machine
        if speed >= 1.5: # let off of gas
            accel_cmd.f64_cmd = 0
            accel_pub.publish(accel_cmd)
        if speed <= 1: # accelerate until our speed is 1 m/s
            accel_cmd.f64_cmd = 0.4
            accel_pub.publish(accel_cmd)

    
    while speed > 0:
        brake_cmd.f64_cmd = 0.3
        brake_pub.publish(brake_cmd)

    # Call a* planning logic



if __name__ == "__main__":
    main()

