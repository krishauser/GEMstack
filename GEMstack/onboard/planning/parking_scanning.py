from pacmod_msgs.msg import PositionWithSpeed, PacmodCmd, VehicleSpeedRpt
import rospy
from sensor_msgs.msg import Image,PointCloud2
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Float32MultiArray
import ctypes
import struct
import argparse
import time


class StraightLineMotion():
    def speed_callback(self, msg):
        self.speed = msg.vehicle_speed
        rospy.loginfo(f"Vehicle Speed: {msg.vehicle_speed} m/s | Valid: {msg.vehicle_speed_valid}")
    def __init__(self):
        if not rospy.core.is_initialized():
            rospy.init_node('straight_line', anonymous=True)
        self.start_time = time.time()
        self.accel_cmd = PacmodCmd()
        self.accel_cmd.enable = True
        self.accel_cmd.clear  = False
        self.accel_cmd.ignore = False

        self.brake_cmd = PacmodCmd()
        self.brake_cmd.enable = True
        self.brake_cmd.clear  = False
        self.brake_cmd.ignore = False
        self.brake_cmd.f64_cmd = 1 
        self.speed = 0
        self.speed_sub = rospy.Subscriber('/pacmod/parsed_tx/vehicle_speed_rpt', VehicleSpeedRpt, self.speed_callback)

        self.accel_pub = rospy.Publisher('/pacmod/as_rx/accel_cmd', PacmodCmd, queue_size=1)
        self.brake_pub = rospy.Publisher('/pacmod/as_rx/brake_cmd', PacmodCmd, queue_size=1)

    def update_speed(self):
        print("Updating speed")
        if self.speed >= 1.5: # let off of gas
            self.accel_cmd.f64_cmd = 0
            self.accel_pub.publish(self.accel_cmd)
        if self.speed <= 1: # accelerate until our speed is 1 m/s
            self.accel_cmd.f64_cmd = 0.4
            self.accel_pub.publish(self.accel_cmd)

    def brake(self):
        while self.speed > 0:
            self.brake_cmd.f64_cmd = 0.3
            self.brake_pub.publish(self.brake_cmd)

