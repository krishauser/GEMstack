#!/usr/bin/env python3

import atexit
import rospy
from sensor_msgs.msg import NavSatFix
from septentrio_gnss_driver.msg import INSNavGeod
import pandas as pd
from mathutils import transforms

bag_file_path = "/media/yourmom/Seagate/message_2025-04-01-17-06-29.bag"
xy_csv_path = "/media/yourmom/Seagate/waypoints_xy.csv"
heading_csv_path = "/media/yourmom/Seagate/waypoints_heading.csv"
final_csv_path = "/media/yourmom/Seagate/waypoints.csv"
olat = 40.0928563
olon = -88.2359994
file1 = open(xy_csv_path, mode="w", newline="")
file2 = open(heading_csv_path, mode="w", newline="")
file3 = open("/media/yourmom/Seagate/IRL_sensor_waypoints_lat_lon.csv", mode="w", newline="")

def save_waypoints_xy(data):
    lat_lon_to_x, lat_lon_to_y = wps_to_local_xy(data.longitude, data.latitude)
    file1.write('%f, %f\n' % (lat_lon_to_x, lat_lon_to_y))
    file3.write('%f, %f\n' % (data.longitude, data.latitude))

def save_waypoints_heading(data):
    file2.write('%f\n' % (data.heading))


def wps_to_local_xy(lon_wp, lat_wp):
    # convert GNSS waypoints into local fixed frame reprented in x and y
    lon_wp_x, lat_wp_y = transforms.lat_lon_to_xy(lat_wp,lon_wp,olat,olon)  #la.ll2xy(lat_wp, lon_wp, olat, olon)
    return lon_wp_x, lat_wp_y

def listener():
    rospy.init_node('rosbag_to_waypoints', anonymous=True)
    xy_sub = rospy.Subscriber('/septentrio_gnss/navsatfix', NavSatFix, save_waypoints_xy)
    heading_sub = rospy.Subscriber('/septentrio_gnss/insnavgeod', INSNavGeod, save_waypoints_heading)
    rospy.spin()

def shutdown():
    print("shutting down!")
    file1.close()
    file2.close()
    file3.close()

    df1 = pd.read_csv(xy_csv_path)
    df2 = pd.read_csv(heading_csv_path)

    merged_df = pd.concat([df1, df2], axis=1)
    merged_df.to_csv(final_csv_path, index=False)

if __name__ == '__main__':
    atexit.register(shutdown)
    print('Generating Waypoints')
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
    



# def wps_to_local_xy(lon_wp, lat_wp):
#         # convert GNSS waypoints into local fixed frame reprented in x and y
#         lon_wp_x, lat_wp_y = axy.ll2xy(lat_wp, lon_wp, olat, olon)
#         return lon_wp_x, lat_wp_y   
# # Path to the rosbag file
# bag_file_path = "/home/sanjay/Downloads/2024-10-26-11-03-12.bag"
# olat       = 40.0928563
# olon       = -88.2359994
# sensor_waypoints = []
# # Open the bag file
# with rosbag.Bag(bag_file_path, 'r') as bag:

#     # "/septentrio_gnss/insnavgeod" -> To get heading
#     # "/septentrio_gnss/navsatfix" -> To get lat lon
#     topic_to_read_from = ["/septentrio_gnss/insnavgeod", "/septentrio_gnss/navsatfix"]
#     xy_msgs = bag.read_messages(topics=[topic_to_read_from[1]])
#     heading_msgs = bag.read_messages(topics=[topic_to_read_from[0]])

#     xy_iter = iter(xy_msgs)
#     heading_iter = iter(heading_msgs)

#     xy_msg = next(xy_iter)
#     heading_msg = next(heading_iter)

#     while xy_msg is not None or heading_msg is not None:
#         # print(xy_msg[1].latitude)
#         # print(xy_msg[1].longitude)
#         # print("-----")
#         # print(heading_msg[1].heading)
#         lat_lon_to_x, lat_lon_to_y = wps_to_local_xy(xy_msg[1].latitude, xy_msg[1].latitude)
#         sensor_waypoints.append([lat_lon_to_x, lat_lon_to_y, heading_msg[1].heading])

# with open("IRL_sensor_waypoints.csv", mode="w", newline="") as file:
#      writer = csv.writer(file)
#      writer.writerows(sensor_waypoints)

        
