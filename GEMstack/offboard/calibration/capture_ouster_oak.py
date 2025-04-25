# ROS Headers
import rospy
from sensor_msgs.msg import Image,PointCloud2,NavSatFix
import sensor_msgs.point_cloud2 as pc2
import ctypes
import struct

# OpenCV and cv2 bridge
import cv2
from cv_bridge import CvBridge
import numpy as np
import os
import time
from functools import partial

camera_images = {"oak": None, "fr": None, "fl": None, "rr": None, "rl": None, "fr_rect": None, "fl_rect": None, "rr_rect": None, "rl_rect": None}
lidar_clouds = {"ouster": None, "livox": None}
depth_images = {"depth": None}
gnss_locations = {"nav_fix": None}
lidar_filetype = ".npz"
camera_filetype = ".png"
depth_filetype = ".tif"
gnss_filetype = ".npy"
bridge = CvBridge()

def lidar_callback(scanner, lidar : PointCloud2):
    global lidar_clouds
    lidar_clouds[scanner] = lidar

def camera_callback(camera, img : Image):
    global camera_images
    camera_images[camera] = img

def depth_callback(camera, img : Image):
    global depth_images
    depth_images[camera] = img

def gnss_callback(gnss, sat_fix : NavSatFix):
    global gnss_locations
    gnss_locations[gnss] = sat_fix

def pc2_to_numpy(pc2_msg, want_rgb = False):
    gen = pc2.read_points(pc2_msg, skip_nans=True)
    if want_rgb:
        xyzpack = np.array(list(gen),dtype=np.float32)
        if xyzpack.shape[1] != 4:
            raise ValueError("PointCloud2 does not have points")
        xyzrgb = np.empty((xyzpack.shape[0],6))
        xyzrgb[:,:3] = xyzpack[:,:3]
        for i,x in enumerate(xyzpack):
            rgb = x[3] 
            # cast float32 to int so that bitwise operations are possible
            s = struct.pack('>f' ,rgb)
            i = struct.unpack('>l',s)[0]
            # you can get back the float value by the inverse operations
            pack = ctypes.c_uint32(i).value
            r = (pack & 0x00FF0000)>> 16
            g = (pack & 0x0000FF00)>> 8
            b = (pack & 0x000000FF)
            #r,g,b values in the 0-255 range
            xyzrgb[i,3:] = (r,g,b)
        return xyzrgb
    else:
        return np.array(list(gen),dtype=np.float32)[:,:3]

def clear_scan():
    global camera_images
    for camera in camera_images:
        camera_images[camera] = None
    global lidar_clouds
    for lidar in lidar_clouds:
        lidar_clouds[lidar] = None
    global depth_images
    for camera in depth_images:
        depth_images[camera] = None
    global gnss_locations
    for gnss in gnss_locations:
        gnss_locations[gnss] = None

def save_scan(folder, index):
    print("Saving scan", index)

    for lidar in lidar_clouds:
        lidar_points = lidar_clouds[lidar]
        if lidar_points != None:
            lidar_fn = os.path.join(folder, lidar + str(index) + lidar_filetype)
            pc = pc2_to_numpy(lidar_points, want_rgb=False) # convert lidar feed to numpy
            np.savez(lidar_fn, pc)
    
    for camera in camera_images:
        camera_image = camera_images[camera]
        if camera_image != None:
            camera_fn = os.path.join(folder, camera + str(index) + camera_filetype)
            cv2.imwrite(camera_fn, bridge.imgmsg_to_cv2(camera_image))

    for camera in depth_images:
        depth_image = depth_images[camera]
        if depth_image != None:
            depth_fn = os.path.join(folder, camera + str(index) + depth_filetype)
            dimage = bridge.imgmsg_to_cv2(depth_image)
            dimage_non_nan = dimage[np.isfinite(dimage)]
            dimage = np.nan_to_num(dimage,nan=0,posinf=0,neginf=0)
            dimage = (dimage/4000*0xffff)
            dimage = dimage.astype(np.uint16)
            cv2.imwrite(depth_fn, dimage)
    
    for gnss in gnss_locations:
        location = gnss_locations[gnss]
        if location != None:
            gnss_fn = os.path.join(folder, gnss + str(index) + gnss_filetype)
            coordinates = np.array([location.latitude, location.longitude])
            np.save(gnss_fn + str(index) + gnss_filetype, coordinates)

def main(folder='data',start_index=0):
    # Initialize node and establish subscribers
    rospy.init_node("capture_ouster_oak",disable_signals=True)
    ouster_sub = rospy.Subscriber("/ouster/points", PointCloud2, partial(lidar_callback, "ouster"))
    livox_sub = rospy.Subscriber("/livox/lidar", PointCloud2, partial(lidar_callback, "livox"))
    oak_sub = rospy.Subscriber("/oak/rgb/image_raw", Image, partial(camera_callback, "oak"))
    cam_fl_sub = rospy.Subscriber("/camera_fl/arena_camera_node/image_raw", Image, partial(camera_callback, "fl"))
    cam_fr_sub = rospy.Subscriber("/camera_fr/arena_camera_node/image_raw", Image, partial(camera_callback, "fr"))
    cam_rl_sub = rospy.Subscriber("/camera_rl/arena_camera_node/image_raw", Image, partial(camera_callback, "rl"))
    cam_rr_sub = rospy.Subscriber("/camera_rr/arena_camera_node/image_raw", Image, partial(camera_callback, "rr"))
    cam_fl_rect_sub = rospy.Subscriber("/camera_fl/arena_camera_node/image_rect_color", Image, partial(camera_callback, "fl_rect"))
    cam_fr_rect_sub = rospy.Subscriber("/camera_fr/arena_camera_node/image_rect_color", Image, partial(camera_callback, "fr_rect"))
    cam_rl_rect_sub = rospy.Subscriber("/camera_rl/arena_camera_node/image_rect_color", Image, partial(camera_callback, "rl_rect"))
    cam_rr_rect_sub = rospy.Subscriber("/camera_rr/arena_camera_node/image_rect_color", Image, partial(camera_callback, "rr_rect"))
    depth_sub = rospy.Subscriber("/oak/stereo/image_raw", Image, partial(depth_callback, "depth"))
    gnss_sub = rospy.Subscriber("/septentrio_gnss/navsatfix", NavSatFix, partial(gnss_callback, "nav_fix"))

    # Store scans
    index = start_index
    print(" Storing lidar point clouds as", lidar_filetype)
    print(" Storing color images as", camera_filetype)
    print(" Storing depth images as", depth_filetype)
    print(" Ctrl+C to quit")
    while True:
        if camera_images["oak"]:
            cv2.imshow("result",bridge.imgmsg_to_cv2(camera_images["oak"]))
            save_scan(folder, index)
            clear_scan()
            index += 1
            time.sleep(.5)

if __name__ == '__main__':
    import sys
    folder = 'data'
    start_index = 1
    if len(sys.argv) >= 2:
        folder = sys.argv[1]
    if len(sys.argv) >= 3:
        start_index = int(sys.argv[2])
    main(folder,start_index)
