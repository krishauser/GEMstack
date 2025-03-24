#!/usr/bin/env python3

import os
import rosbag
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, PointCloud2
import sensor_msgs.point_cloud2 as pc2
import glob
from datetime import datetime
import time
import rospy
import std_msgs.msg
import argparse

def get_matching_files(data_dir):
    """
    Collect and return files that have matching (common) index numbers:
      - camera_fl[index].png
      - camera_fr[index].png
      - camera_rr[index].png
      - camera_rl[index].png
      - front_cam[index].png
      - lidar_front[index].npz
      - lidar_top[index].npz
    """

    # 1) Gather each pattern of files
    camera_fl_files     = sorted(glob.glob(os.path.join(data_dir, 'camera_fl*.png')))
    camera_fr_files     = sorted(glob.glob(os.path.join(data_dir, 'camera_fr*.png')))
    camera_rr_files     = sorted(glob.glob(os.path.join(data_dir, 'camera_rr*.png')))
    camera_rl_files     = sorted(glob.glob(os.path.join(data_dir, 'camera_rl*.png')))
    front_cam_files     = sorted(glob.glob(os.path.join(data_dir, 'front_cam*.png')))
    lidar_front_files   = sorted(glob.glob(os.path.join(data_dir, 'lidar_front*.npz')))
    lidar_top_files     = sorted(glob.glob(os.path.join(data_dir, 'lidar_top*.npz')))

    print(f"Found {len(camera_fl_files)} FL camera PNG files")
    print(f"Found {len(camera_fr_files)} FR camera PNG files")
    print(f"Found {len(camera_rr_files)} RR camera PNG files")
    print(f"Found {len(camera_rl_files)} RL camera PNG files")
    print(f"Found {len(front_cam_files)} front_cam PNG files")
    print(f"Found {len(lidar_front_files)} lidar_front NPZ files")
    print(f"Found {len(lidar_top_files)} lidar_top NPZ files")

    # 2) Helper function to extract index from filename
    def get_number(filename):
        # Example: "camera_fl123.png" => 123
        base = os.path.basename(filename)
        # Filter out digits in the basename, then convert to int
        return int(''.join(filter(str.isdigit, base)))

    # 3) Convert to dictionaries keyed by index number
    fl_dict    = {get_number(f): f for f in camera_fl_files}
    fr_dict    = {get_number(f): f for f in camera_fr_files}
    rr_dict    = {get_number(f): f for f in camera_rr_files}
    rl_dict    = {get_number(f): f for f in camera_rl_files}
    front_dict = {get_number(f): f for f in front_cam_files}

    lidar_front_dict = {get_number(f): f for f in lidar_front_files}
    lidar_top_dict   = {get_number(f): f for f in lidar_top_files}

    # 4) Find common index numbers across all 7 dictionaries
    common_indices = (
        set(fl_dict.keys())
        & set(fr_dict.keys())
        & set(rr_dict.keys())
        & set(rl_dict.keys())
        & set(front_dict.keys())
        & set(lidar_front_dict.keys())
        & set(lidar_top_dict.keys())
    )

    print(f"\nFound {len(common_indices)} matching indices among all file types")

    # 5) For convenience, sort them and pull out matching file paths in the same order
    common_indices = sorted(common_indices)
    matching_fl    = [fl_dict[idx] for idx in common_indices]
    matching_fr    = [fr_dict[idx] for idx in common_indices]
    matching_rr    = [rr_dict[idx] for idx in common_indices]
    matching_rl    = [rl_dict[idx] for idx in common_indices]
    matching_front = [front_dict[idx] for idx in common_indices]

    matching_lidar_front = [lidar_front_dict[idx] for idx in common_indices]
    matching_lidar_top   = [lidar_top_dict[idx]   for idx in common_indices]

    return (
        matching_fl,
        matching_fr,
        matching_rr,
        matching_rl,
        matching_front,
        matching_lidar_front,
        matching_lidar_top
    )

def create_rosbag_from_data(data_dir, output_bag_path, frame_rate):
    """Create a ROS bag with camera (PNG) and LiDAR (NPZ) data at a given frame rate."""

    # Initialize CV bridge and ROS node
    bridge = CvBridge()
    rospy.init_node('bag_creator', anonymous=True)

    # Gather matching files
    (
        fl_files,
        fr_files,
        rr_files,
        rl_files,
        front_files,
        lidar_front_files,
        lidar_top_files
    ) = get_matching_files(data_dir)

    num_frames = len(fl_files)
    print(f"Found {num_frames} total matching sets of files")

    if num_frames == 0:
        raise ValueError("No matching files found!")

    # Create a new bag
    with rosbag.Bag(output_bag_path, 'w') as bag:
        # Start time for the messages
        start_time = rospy.Time.from_sec(time.time())
        message_count = 0

        # Publish each frame
        for idx in range(num_frames):
            # Prepare a timestamp for this frame
            timestamp = rospy.Time.from_sec(start_time.to_sec() + idx * (1.0 / frame_rate))

            try:
                # -------------------------------------------------
                # 1) CAMERA: FL, FR, RR, RL, FRONT
                # -------------------------------------------------
                # Read each .png file
                fl_img = cv2.imread(fl_files[idx])
                fr_img = cv2.imread(fr_files[idx])
                rr_img = cv2.imread(rr_files[idx])
                rl_img = cv2.imread(rl_files[idx])
                front_img = cv2.imread(front_files[idx])

                # Convert to ROS Image messages
                if fl_img is not None:
                    fl_msg = bridge.cv2_to_imgmsg(fl_img, encoding="bgr8")
                    fl_msg.header.stamp = timestamp
                    fl_msg.header.frame_id = "camera_fl"
                    bag.write('/camera/fl/image_raw', fl_msg, timestamp)
                    message_count += 1

                if fr_img is not None:
                    fr_msg = bridge.cv2_to_imgmsg(fr_img, encoding="bgr8")
                    fr_msg.header.stamp = timestamp
                    fr_msg.header.frame_id = "camera_fr"
                    bag.write('/camera/fr/image_raw', fr_msg, timestamp)
                    message_count += 1

                if rr_img is not None:
                    rr_msg = bridge.cv2_to_imgmsg(rr_img, encoding="bgr8")
                    rr_msg.header.stamp = timestamp
                    rr_msg.header.frame_id = "camera_rr"
                    bag.write('/camera/rr/image_raw', rr_msg, timestamp)
                    message_count += 1

                if rl_img is not None:
                    rl_msg = bridge.cv2_to_imgmsg(rl_img, encoding="bgr8")
                    rl_msg.header.stamp = timestamp
                    rl_msg.header.frame_id = "camera_rl"
                    bag.write('/camera/rl/image_raw', rl_msg, timestamp)
                    message_count += 1

                if front_img is not None:
                    front_msg = bridge.cv2_to_imgmsg(front_img, encoding="bgr8")
                    front_msg.header.stamp = timestamp
                    front_msg.header.frame_id = "camera_front"
                    bag.write('/camera/front/image_raw', front_msg, timestamp)
                    message_count += 1

                # -------------------------------------------------
                # 2) LIDAR: FRONT, TOP
                # -------------------------------------------------
                # For .npz, assume data is in 'arr_0' (x, y, z, or however your data is structured)
                front_data = np.load(lidar_front_files[idx])
                front_points = front_data['arr_0']  # e.g., Nx3 array

                top_data = np.load(lidar_top_files[idx])
                top_points = top_data['arr_0']      # e.g., Nx3 array

                # A) Lidar FRONT
                front_header = std_msgs.msg.Header()
                front_header.stamp = timestamp
                front_header.frame_id = "lidar_front"

                # Example: 3 float32 fields for x,y,z
                fields = [
                    pc2.PointField('x', 0, pc2.PointField.FLOAT32, 1),
                    pc2.PointField('y', 4, pc2.PointField.FLOAT32, 1),
                    pc2.PointField('z', 8, pc2.PointField.FLOAT32, 1),
                ]
                front_pc_msg = pc2.create_cloud(front_header, fields, front_points)
                bag.write('/lidar/front/points', front_pc_msg, timestamp)
                message_count += 1

                # B) Lidar TOP
                top_header = std_msgs.msg.Header()
                top_header.stamp = timestamp
                top_header.frame_id = "lidar_top"

                top_pc_msg = pc2.create_cloud(top_header, fields, top_points)
                bag.write('/lidar/top/points', top_pc_msg, timestamp)
                message_count += 1

            except Exception as e:
                print(f"Error processing frame {idx}: {e}")
                continue

            # Log progress every so often
            if idx % 10 == 0:
                print(f"Processed {idx} frames...")

        print(f"Finished writing {message_count} total messages to {output_bag_path}.")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description='Convert multiple camera (PNG) and LiDAR (NPZ) files into a single ROS bag.'
    )

    parser.add_argument(
        'files_directory',
        type=str,
        help='Path to the directory containing the camera and lidar files.'
    )

    parser.add_argument(
        'output_bag',
        type=str,
        help='Full path (including filename) where the output ROS bag will be saved.'
    )

    parser.add_argument(
        'rate',
        type=int,
        help='Playback (or capture) rate in Hz (frames per second).'
    )

    args = parser.parse_args()

    try:
        create_rosbag_from_data(
            data_dir=args.files_directory,
            output_bag_path=args.output_bag,
            frame_rate=args.rate
        )
        print("Successfully created ROS bag file!")

        # Show basic info about the new bag
        print("\nBag file contents:")
        os.system(f"rosbag info {args.output_bag}")

    except Exception as e:
        print(f"Error creating ROS bag: {e}")
