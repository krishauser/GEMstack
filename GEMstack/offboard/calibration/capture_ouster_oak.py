# ROS Headers
import rospy
from sensor_msgs.msg import Image,PointCloud2,NavSatFix
import sensor_msgs.point_cloud2 as pc2
import ctypes
import struct

# from ...state import VehicleState, ObjectPose
# from ...onboard import Component

# OpenCV and cv2 bridge
import cv2
from cv_bridge import CvBridge
import numpy as np
import os
import time

# TODO: change these arrays to use dictionaries, format eg. {filename: data}
lidar_points = None
camera_image = None
fl_image = None
fr_image = None
rl_image = None
rr_image = None
depth_images = None
gnss_locations = None
frame_position = None
lidar_filetype = ".npz"
camera_filetype = ".png"
depth_filetype = ".tif"
gnss_filetype = ".npy"
bridge = CvBridge()

def lidar_callback(lidar : PointCloud2):
    global lidar_points
    lidar_points = (lidar)

def camera_callback(img : Image):
    global camera_image
    camera_image = (img)

def fl_camera_callback(img : Image):
    global fl_image
    fl_image = (img)

def fr_camera_callback(img : Image):
    global fr_image
    fr_image = (img)

def rl_camera_callback(img : Image):
    global rl_image
    rl_image = (img)

def rr_camera_callback(img : Image):
    global rr_image
    rr_image = (img)

def depth_callback(img : Image):
    global depth_images
    depth_images = (img)

def gnss_callback(sat_fix : NavSatFix):
    global gnss_locations
    gnss_locations = (sat_fix)

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

# def clear_scan():
#     global lidar_points
#     lidar_points = []
#     global camera_image
#     camera_image = []
#     global depth_images
#     depth_images = []
#     global gnss_locations
#     gnss_locations = []

# TODO: when data is stored in dictionaries, change to only save files which exist and change error checking
def save_scan(lidar_fn, camera_fn, fl_fn, fr_fn, rl_fn, rr_fn, depth_fn, gnss_fn, index):
    # if len(lidar_filenames) != len(lidar_points) or len(camera_filenames) != len(camera_image) or len(depth_filenames) != len(depth_images):
    #     print("Missing data, scan", index, "cannot be saved")
    #     print(len(lidar_filenames), len(lidar_points), len(camera_filenames), len(camera_image), len(depth_filenames), len(depth_images))
    #     return
    print("Saving scan", index)

    if frame_position != None:
        np.save("state" + str(index), frame_position)
    else:
        print("No state found")

    # for i in range(len(lidar_points)):
    #     current_lidar = lidar_points[i]
    # lidar_fn = lidar_filename + str(index) + lidar_filetype
    pc = pc2_to_numpy(lidar_points,want_rgb=False) # convert lidar feed to numpy
    np.savez(lidar_fn + str(index) + lidar_filetype,pc)

    # for i in range(len(camera_image)):
    #     current_camera = camera_image[i]
    # camera_fn = camera_filenames[i] + str(index) + camera_filetype
    cv2.imwrite(camera_fn + str(index) + camera_filetype,bridge.imgmsg_to_cv2(camera_image))
    cv2.imwrite(fl_fn + str(index) + camera_filetype,bridge.imgmsg_to_cv2(fl_image))
    cv2.imwrite(fr_fn + str(index) + camera_filetype,bridge.imgmsg_to_cv2(fr_image))
    cv2.imwrite(rl_fn + str(index) + camera_filetype,bridge.imgmsg_to_cv2(rl_image))
    cv2.imwrite(rr_fn + str(index) + camera_filetype,bridge.imgmsg_to_cv2(rr_image))

    # for i in range(len(depth_images)):
    dimage = bridge.imgmsg_to_cv2(depth_images)
    # depth_fn = depth_filenames[i] + str(index) + depth_filetype
    dimage_non_nan = dimage[np.isfinite(dimage)]
    # print("Depth range",np.min(dimage_non_nan),np.max(dimage_non_nan))
    dimage = np.nan_to_num(dimage,nan=0,posinf=0,neginf=0)
    dimage = (dimage/4000*0xffff)
    # print("Depth pixel range",np.min(dimage),np.max(dimage))
    dimage = dimage.astype(np.uint16)
    cv2.imwrite(depth_fn + str(index) + depth_filetype,dimage)
    
    # for i in range(len(gnss_locations)):
    # sat_fix = gnss_locations[i]
    # gnss_fn = gnss_filenames + str(index) + gnss_filetype
    if gnss_locations:
        coordinates = np.array([gnss_locations.latitude, gnss_locations.longitude])
        np.save(gnss_fn + str(index) + gnss_filetype, coordinates)

def main(folder='data',start_index=0):
    # Initialize node and establish subscribers
    rospy.init_node("capture_ouster_oak",disable_signals=True)
    ouster_sub = rospy.Subscriber("/ouster/points", PointCloud2, lidar_callback)
  #  livox_sub = rospy.Subscriber("/livox/lidar", PointCloud2, lidar_callback)
    oak_sub = rospy.Subscriber("/oak/rgb/image_raw", Image, camera_callback)
    cam_fl_sub = rospy.Subscriber("/camera_fl/arena_camera_node/image_raw", Image, fl_camera_callback)
    cam_fr_sub = rospy.Subscriber("/camera_fr/arena_camera_node/image_raw", Image, fr_camera_callback)
    cam_rl_sub = rospy.Subscriber("/camera_rl/arena_camera_node/image_raw", Image, rl_camera_callback)
    cam_rr_sub = rospy.Subscriber("/camera_rr/arena_camera_node/image_raw", Image, rr_camera_callback)
    depth_sub = rospy.Subscriber("/oak/stereo/image_raw", Image, depth_callback)
    gnss_sub = rospy.Subscriber("/septentrio_gnss/navsatfix", NavSatFix, gnss_callback)

    # Store scans
    index = start_index
    print(" Storing lidar point clouds as", lidar_filetype)
    print(" Storing color images as", camera_filetype)
    print(" Storing depth images as", depth_filetype)
    print(" Ctrl+C to quit")
    while True:
        if camera_image:
            cv2.imshow("result",bridge.imgmsg_to_cv2(camera_image))
            time.sleep(.5)
            files = [os.path.join(folder, 'ouster'),os.path.join(folder, 'oak'),
                            os.path.join(folder, 'fl'), os.path.join(folder, 'fr'),
                            os.path.join(folder, 'rl'), os.path.join(folder, 'rr'),
                            os.path.join(folder, 'depth'), os.path.join(folder, 'septentrio')]
            # lidar_files = [os.path.join(folder, 'ouster')]
            # camera_files = [os.path.join(folder, 'oak'),
            #                 os.path.join(folder, 'fl'), os.path.join(folder, 'fr'),
            #                 os.path.join(folder, 'rl'), os.path.join(folder, 'rr')]
            # depth_files = [os.path.join(folder, 'depth')]
            # gnss_files = [os.path.join(folder, 'septentrio')]
            save_scan(*files, index)
            # clear_scan()
            index += 1

# class StateTracker(Component):
#     def rate(self):
#         return 4.0 # Hz
#     def state_inputs(self):
#         return ['vehicle']
#     def update(self, vehicle : VehicleState):
#         global frame_position
#         frame_position = np.array(vehicle.pose.frame, vehicle.pose.t, vehicle.pose.x, vehicle.pose.y)

if __name__ == '__main__':
    import sys
    folder = 'data'
    start_index = 1
    if len(sys.argv) >= 2:
        folder = sys.argv[1]
    if len(sys.argv) >= 3:
        start_index = int(sys.argv[2])
    main(folder,start_index)
