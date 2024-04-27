# needed to import GEMstack from top level directory
import sys
import os
import cv2
import argparse
import random
import numpy as np
from typing import Dict, Tuple, List
import pathlib
import os
import cv2
sys.path.append(os.getcwd())
abs_path = os.path.abspath(os.path.dirname(__file__))

from GEMstack.onboard.perception.pixelwise_3D_lidar_coord_handler import PixelWise3DLidarCoordHandler

parser = argparse.ArgumentParser()

parser.add_argument('--src_dir', '-s', type=str, default='./data/gt')
parser.add_argument('--test_target', '-t', type=str, default='frame',
                    choices=['frame', 'rosbag', 'test_conv'])
parser.add_argument('--data_idx', '-i', type=int, default=3)
parser.add_argument('--kernel_size', '-k', type=int, default=5)
args = parser.parse_args()

coord_3d_map = None
mouse_x = None
mouse_y = None

def mouse_callback(event, x, y, flags, param):
    global coord_3d_map
    global mouse_x
    global mouse_y
    mouse_x = x
    mouse_y = y
    # if event == cv2.EVENT_MOUSEMOVE:
    # Display pixel coordinates on the console
    x_3d, y_3d, z_3d = coord_3d_map[y][x]
    
    print(
        f"Pixel coordinates: x={x}, y={y} | 3D xyz coord: x={x_3d:.2f}, y={y_3d:.2f}, z={z_3d:.2f} (in meters)")


if args.test_target == 'rosbag':
    import sensor_msgs.point_cloud2 as pc2
    from cv_bridge import CvBridge
    import rospy
    from sensor_msgs.msg import Image, PointCloud2

    class TestNode:
        def __init__(self, coord_3d_handler):
            self.coord_3d_handler = coord_3d_handler
            self.bridge = CvBridge()
            rospy.Subscriber('/ouster/points', PointCloud2,
                             self.lidar_callback)
            rospy.Subscriber('/oak/rgb/image_raw', Image, self.camera_callback)
            self.image = None
            self.point_cloud = None

        def camera_callback(self, image: Image):
            self.image = image

        def lidar_callback(self, point_cloud):
            self.point_cloud = point_cloud

        def can_run(self):
            if self.image is None or self.point_cloud is None:
                return False
            return True

        def run(self):
            # convert data type
            cv_image = self.bridge.imgmsg_to_cv2(self.image, "bgr8")
            numpy_point_cloud = ros_PointCloud2_to_numpy(
                self.point_cloud, want_rgb=False)

            # get3DCoord
            coord_3d_map = self.coord_3d_handler.get3DCoord(
                cv_image, numpy_point_cloud)
            
            return coord_3d_map, cv_image.copy()


def test_conv_2D():
    # Define a sample image and kernel
    image = np.array([
        [10, 0, 0],
        [0, 20, 0],
        [0, 0, 30]
    ], dtype=np.float32)

    # Do convolution
    kernel = np.array([
        [1, 1],
        [1, 1]
    ], dtype=np.float32)
    kernel = cv2.flip(kernel, flipCode=-1)
    result = cv2.filter2D(image, -1, kernel, borderType=cv2.BORDER_CONSTANT)

    # Derive num_elements matrix
    has_value = image > 0
    has_value = has_value.astype(np.uint8)
    num_elements_mat = cv2.filter2D(
        has_value, -1, kernel, borderType=cv2.BORDER_CONSTANT)

    # divide by num of elements correspond to that particular pixel
    num_elements_mat[num_elements_mat == 0] = 1  # avoid divide by zero
    interpolation_result = result / num_elements_mat

    print("Image:")
    print(image)
    print("\nKernel:")
    print(kernel)
    print("\nResult:")
    print(result)
    print("\nnum_elements_mat:")
    print(num_elements_mat)
    print("\n Interpolation Result:")
    print(interpolation_result)


def ros_PointCloud2_to_numpy(pc2_msg, want_rgb=False):
    if pc2 is None:
        raise ImportError("ROS is not installed")
    # gen = pc2.read_points(pc2_msg, skip_nans=True)
    gen = pc2.read_points(pc2_msg, skip_nans=True, field_names=['x', 'y', 'z'])

    if want_rgb:
        xyzpack = np.array(list(gen), dtype=np.float32)
        if xyzpack.shape[1] != 4:
            raise ValueError(
                "PointCloud2 does not have points with color data.")
        xyzrgb = np.empty((xyzpack.shape[0], 6))
        xyzrgb[:, :3] = xyzpack[:, :3]
        for i, x in enumerate(xyzpack):
            rgb = x[3]
            # cast float32 to int so that bitwise operations are possible
            s = struct.pack('>f', rgb)
            i = struct.unpack('>l', s)[0]
            # you can get back the float value by the inverse operations
            pack = ctypes.c_uint32(i).value
            r = (pack & 0x00FF0000) >> 16
            g = (pack & 0x0000FF00) >> 8
            b = (pack & 0x000000FF)
            # r,g,b values in the 0-255 range
            xyzrgb[i, 3:] = (r, g, b)
        return xyzrgb
    else:
        return np.array(list(gen), dtype=np.float32)[:, :3]


if __name__ == '__main__':

    if args.test_target == 'test_conv':
        ### verify the correctness of opencv filter2D ###
        test_conv_2D()

    elif args.test_target == 'frame':
        # load data
        lidar_fn = os.path.join(args.src_dir, f'lidar{args.data_idx}.npz')
        image_fn = os.path.join(args.src_dir, f'color{args.data_idx}.png')

        point_cloud = np.load(lidar_fn)['arr_0']
        image = cv2.imread(image_fn)

        print('\nStart testing...')

        # init handler
        handler = PixelWise3DLidarCoordHandler(args.kernel_size)
        coord_3d_map = handler.get3DCoord(image, point_cloud)

        # check pixel 3D coords in interactive mode
        cv2.namedWindow('Image')
        cv2.setMouseCallback('Image', mouse_callback)
        while True:
            cv2.imshow('Image', image)

            # Break the loop if 'q' is pressed
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        cv2.destroyAllWindows()

    elif args.test_target == 'rosbag':
        rospy.init_node('test_node')
        rate = rospy.Rate(30)  # Hz

        # init class object
        handler = PixelWise3DLidarCoordHandler(args.kernel_size)
        test_node = TestNode(handler)

        try:
            print('\nStart testing...')
            cv2.namedWindow('Image')
            cv2.setMouseCallback('Image', mouse_callback)
            
            while not rospy.is_shutdown():
                rate.sleep()
                if not test_node.can_run():
                    continue

                coord_3d_map, image = test_node.run()

                # check pixel 3D coords in interactive mode
                cv2.circle(image, center=(mouse_x, mouse_y), radius=5, color=(0, 0, 255), thickness=cv2.FILLED)
                cv2.imshow('Image', image)
                
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    exit(1)
        except rospy.ROSInterruptException:
            print('rospy node error.')

    print('\nDone!')
