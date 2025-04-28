from __future__ import print_function

# Python Headers
import os
import cv2 
import numpy as np

import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

import cv2
import numpy as np

image_path = os.path.join(os.path.dirname(__file__), "highbay_image.pgm")

class OccupancyGrid2:

    global image_path
    
    def __init__(self):

        # Read image in BGR format
        self.map_image = cv2.imread(image_path)

        # Create the cv_bridge object
        self.bridge  = CvBridge()
        self.map_image_pub = rospy.Publisher("/motion_image", Image, queue_size=1) 

        # Subscribe information from sensors
        self.lat     = 0
        self.lon     = 0
        self.heading = 0

        self.lat_start_bt = 40.092722  # 40.09269  
        self.lon_start_l  = -88.236365 # -88.23628
        self.lat_scale    = 0.00062   # 0.0007    
        self.lon_scale    = 0.00136  # 0.00131   

        self.arrow        = 40 
        self.img_width    = 2107
        self.img_height   = 1313

    def image_to_gnss(self, pix_x, pix_y):
        """
        Given image coordinates (pix_x, pix_y), return (latitude, longitude).
        Inverse of your:
            pix_x = img_width * (lon - lon_start_l) / lon_scale
            pix_y = img_height - img_height*(lat - lat_start_bt) / lat_scale
        """
        # longitude:
        lon = self.lon_start_l + (pix_x / float(self.img_width)) * self.lon_scale
        lat = self.lat_start_bt + ((self.img_height - pix_y) / float(self.img_height)) * self.lat_scale

        return lat, lon

    def image_heading(self, lon_x, lat_y, heading):
        
        if(heading >=0 and heading < 90):
            angle  = np.radians(90-heading)
            lon_xd = lon_x + int(self.arrow * np.cos(angle))
            lat_yd = lat_y - int(self.arrow * np.sin(angle))

        elif(heading >= 90 and heading < 180):
            angle  = np.radians(heading-90)
            lon_xd = lon_x + int(self.arrow * np.cos(angle))
            lat_yd = lat_y + int(self.arrow * np.sin(angle))  

        elif(heading >= 180 and heading < 270):
            angle = np.radians(270-heading)
            lon_xd = lon_x - int(self.arrow * np.cos(angle))
            lat_yd = lat_y + int(self.arrow * np.sin(angle))

        else:
            angle = np.radians(heading-270)
            lon_xd = lon_x - int(self.arrow * np.cos(angle))
            lat_yd = lat_y - int(self.arrow * np.sin(angle)) 

        return lon_xd, lat_yd         


    def gnss_to_image_with_heading(self, lon, lat, heading):
        
        lon_x = int(self.img_width*(lon - self.lon_start_l)/self.lon_scale)
        lat_y = int(self.img_height-self.img_height*(lat - self.lat_start_bt)/self.lat_scale)
        lon_xd, lat_yd = self.image_heading(lon_x, lat_y, heading)

        pub_image = np.copy(self.map_image)

        if(lon_x >= 0 and lon_x <= self.img_width and lon_xd >= 0 and lon_xd <= self.img_width and 
            lat_y >= 0 and lat_y <= self.img_height and lat_yd >= 0 and lat_yd <= self.img_height):
            cv2.arrowedLine(pub_image, (lon_x, lat_y), (lon_xd, lat_yd), (0, 0, 255), 2)
            cv2.circle(pub_image, (lon_x, lat_y), 12, (0,0,255), 2)

            ## Debug check if you can convert back to GNSS
            # tmp_lat, tmp_lon = self.image_to_gnss(lon_x, lat_y)
            # print(f'Lat: {self.lat}, Lon: {self.lon}, Converted Lat: {tmp_lat}, Converted Lon: {tmp_lon}')
            ## End Debug check
        try:
            # Convert OpenCV image to ROS image and publish
            self.map_image_pub.publish(self.bridge.cv2_to_imgmsg(pub_image, "bgr8"))
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))

    def gnss_to_image(self, lon, lat):
        
        lon_x = int(self.img_width - self.img_width*(lon - self.lon_start_l) /self.lon_scale)
        lat_y = int(self.img_height  -  self.img_height*(lat - self.lat_start_bt) * -1/self.lat_scale)
        print(f"GNSS ({lat}, {lon}) → pixel ({lon_x}, {lat_y})")
        pub_image = np.copy(self.map_image)

        if(lon_x >= 0 and lon_x <= self.img_width and 
            lat_y >= 0 and lat_y <= self.img_height):
            cv2.circle(pub_image, (lon_x, lat_y), 12, (0,0,255), 2)


        try:
            # Convert OpenCV image to ROS image and publish
            self.map_image_pub.publish(self.bridge.cv2_to_imgmsg(pub_image, "bgr8"))
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))

    def gnss_to_image_coords(self, lon, lat):
        
        lon_x = int(self.img_width - self.img_width*(lon - self.lon_start_l) /self.lon_scale)
        lat_y = int(self.img_height  -  self.img_height*(lat - self.lat_start_bt) * -1/self.lat_scale)
        print(f"GNSS ({lat}, {lon}) → pixel ({lon_x}, {lat_y})")
        return lon_x, lat_y

