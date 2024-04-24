# ROS Headers
import rospy
from sensor_msgs.msg import Image,PointCloud2
import sensor_msgs.point_cloud2 as pc2
import ctypes
import struct

# OpenCV and cv2 bridge
import cv2
from cv_bridge import CvBridge
import numpy as np
import os

lidar_points = None
camera_image = None
fl_camera_image = None
fr_camera_image = None
depth_image = None
bridge = CvBridge()

def lidar_callback(lidar : PointCloud2):
    global lidar_points
    lidar_points = lidar

def camera_callback(img : Image):
    global camera_image
    camera_image = img

def fl_camera_callback(img : Image):
    global fl_camera_image
    fl_camera_image = img

def fr_camera_callback(img : Image):
    global fr_camera_image
    fr_camera_image = img


def depth_callback(img : Image):
    global depth_image
    depth_image = img

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

def save_scan(lidar_fn,color_fn, fl_color_fn, fr_color_fn, depth_fn):
    print("Saving scan to",lidar_fn,color_fn,depth_fn)
    pc = pc2_to_numpy(lidar_points,want_rgb=False)
    np.savez(lidar_fn,pc)
    cv2.imwrite(color_fn,bridge.imgmsg_to_cv2(camera_image))
    cv2.imwrite(fl_color_fn,bridge.imgmsg_to_cv2(fl_camera_image))
    cv2.imwrite(fr_color_fn,bridge.imgmsg_to_cv2(fr_camera_image))
    dimage = bridge.imgmsg_to_cv2(depth_image)
    dimage_non_nan = dimage[np.isfinite(dimage)]
    print("Depth range",np.min(dimage_non_nan),np.max(dimage_non_nan))
    dimage = np.nan_to_num(dimage,nan=0,posinf=0,neginf=0)
    dimage = (dimage/4000*0xffff)
    print("Depth pixel range",np.min(dimage),np.max(dimage))
    dimage = dimage.astype(np.uint16)
    cv2.imwrite(depth_fn,dimage)

def main(folder='data',start_index=1):
    rospy.init_node("capture_lidar_zed",disable_signals=True)
    lidar_sub = rospy.Subscriber("/ouster/points", PointCloud2, lidar_callback)
    camera_sub = rospy.Subscriber("/oak/rgb/image_raw", Image, camera_callback)
    fl_camera_sub = rospy.Subscriber("/camera_fl/arena_camera_node/image_raw", Image, fl_camera_callback)
    fr_camera_sub = rospy.Subscriber("/camera_fr/arena_camera_node/image_raw", Image, fr_camera_callback)
    depth_sub = rospy.Subscriber("/oak/stereo/image_raw", Image, depth_callback)
    index = start_index
    print("Press any key to:")
    print("  store lidar point clouds as npz")
    print("  store color images as png")
    print("  store depth images (m scaled by 0xffff/4000) as 16-bit tif")
    print("Press Escape or Ctrl+C to quit")
    while True:
        if camera_image:
            cv2.imshow("result",bridge.imgmsg_to_cv2(camera_image))
            key = cv2.waitKey(30)
            if key == -1:
                #nothing
                pass
            elif key == 27:
                #escape
                break
            else:
                if lidar_points is None or camera_image is None or depth_image is None:
                    print (lidar_points is None)
                    print (camera_image is None)
                    print (depth_image is None)
                    print("Missing some messages?")
                else:
                    files = [os.path.join(folder,'lidar{}.npz'.format(index)),
                        os.path.join(folder,'color{}.png'.format(index)),
                        os.path.join(folder,'color_fl{}.png'.format(index)),
                        os.path.join(folder,'color_fr{}.png'.format(index)),
                        os.path.join(folder,'depth{}.tif'.format(index))]
                    save_scan(*files)
                    index += 1

if __name__ == '__main__':
    import sys
    folder = 'data'
    start_index = 1
    if len(sys.argv) >= 2:
        folder = sys.argv[1]
    if len(sys.argv) >= 3:
        start_index = int(sys.argv[2])
    main(folder,start_index)
