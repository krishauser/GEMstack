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
import std_msgs.msg  # Add this import

def get_matching_files(png_dir, tif_dir, npz_dir):
    """Get files that have corresponding data in all three formats"""
    # Get sorted lists of files
    png_files = sorted(glob.glob(os.path.join(png_dir, 'color*.png')))
    tif_files = sorted(glob.glob(os.path.join(tif_dir, 'depth*.tif')))
    npz_files = sorted(glob.glob(os.path.join(npz_dir, 'lidar*.npz')))
    
    print(f"Found {len(png_files)} PNG files")
    print(f"Found {len(tif_files)} TIF files")
    print(f"Found {len(npz_files)} NPZ files")
    
    # Extract numbers from filenames
    def get_number(filename):
        return int(''.join(filter(str.isdigit, os.path.basename(filename))))
    
    # Create dictionaries with numbers as keys
    png_dict = {get_number(f): f for f in png_files}
    tif_dict = {get_number(f): f for f in tif_files}
    npz_dict = {get_number(f): f for f in npz_files}
    
    # Find common numbers
    common_numbers = set(png_dict.keys()) & set(tif_dict.keys()) & set(npz_dict.keys())
    print(f"\nFound {len(common_numbers)} matching frame numbers")
    
    # Get matching files in sorted order
    common_numbers = sorted(common_numbers)
    matching_pngs = [png_dict[n] for n in common_numbers]
    matching_tifs = [tif_dict[n] for n in common_numbers]
    matching_npzs = [npz_dict[n] for n in common_numbers]
    
    return matching_pngs, matching_tifs, matching_npzs

def create_rosbag_from_data(
    png_dir, 
    tif_dir, 
    npz_dir, 
    output_bag_path, 
    frame_interval=0.5
):
    # Initialize CV bridge
    bridge = CvBridge()
    
    # Initialize ROS node
    rospy.init_node('bag_creator', anonymous=True)
    
    # Get matching files
    png_files, tif_files, npz_files = get_matching_files(png_dir, tif_dir, npz_dir)
    
    print(f"Found {len(png_files)} matching files in all three formats")
    
    if len(png_files) == 0:
        raise ValueError("No matching files found!")
    
    # Create a new bag file
    with rosbag.Bag(output_bag_path, 'w') as bag:
        # Start time for the messages
        start_time = rospy.Time.from_sec(time.time())
        message_count = 0
        
        for idx, (png_file, tif_file, npz_file) in enumerate(zip(png_files, tif_files, npz_files)):
            # Calculate timestamp for this frame
            timestamp = rospy.Time.from_sec(start_time.to_sec() + idx * frame_interval)
            
            try:
                # Process PNG image
                png_img = cv2.imread(png_file)
                if png_img is not None:
                    png_msg = bridge.cv2_to_imgmsg(png_img, encoding="bgr8")
                    png_msg.header.stamp = timestamp
                    png_msg.header.frame_id = "camera_rgb"
                    bag.write('/camera/rgb/image_raw', png_msg, timestamp)
                    message_count += 1
                else:
                    print(f"Warning: Could not read PNG file: {png_file}")
                
                # Process TIF image
                tif_img = cv2.imread(tif_file, -1)  # -1 to preserve original depth
                if tif_img is not None:
                    tif_msg = bridge.cv2_to_imgmsg(tif_img, encoding="passthrough")
                    tif_msg.header.stamp = timestamp
                    tif_msg.header.frame_id = "camera_depth"
                    bag.write('/camera/depth/image_raw', tif_msg, timestamp)
                    message_count += 1
                else:
                    print(f"Warning: Could not read TIF file: {tif_file}")
                
                # Process pointcloud NPZ
                pc_data = np.load(npz_file)
                points = pc_data['arr_0']  # Using 'arr_0' instead of 'points'
                
                # Create pointcloud message
                header = std_msgs.msg.Header()
                header.stamp = timestamp
                header.frame_id = "velodyne"
                
                fields = [
                    pc2.PointField('x', 0, pc2.PointField.FLOAT32, 1),
                    pc2.PointField('y', 4, pc2.PointField.FLOAT32, 1),
                    pc2.PointField('z', 8, pc2.PointField.FLOAT32, 1)
                ]
                
                pc_msg = pc2.create_cloud(header, fields, points)
                bag.write('/velodyne_points', pc_msg, timestamp)
                message_count += 1
                
            except Exception as e:
                print(f"Error processing frame {idx}: {str(e)}")
                continue
            
            if idx % 10 == 0:
                print(f"Processed {idx} frames")
        
        print(f"Total messages written to bag: {message_count}")

if __name__ == "__main__":
    # Define your directories here
    png_directory = "/home/mhmadnour/host/CS588/GEMstack/data/data_sample/data/"
    tif_directory = "/home/mhmadnour/host/CS588/GEMstack/data/data_sample/data/"
    npz_directory = "/home/mhmadnour/host/CS588/GEMstack/data/data_sample/data/"
    output_bag = "/home/mhmadnour/host/CS588/GEMstack/data/data_sample/data/output.bag"
    
    try:
        create_rosbag_from_data(
            png_directory,
            tif_directory,
            npz_directory,
            output_bag
        )
        print("Successfully created ROS bag file!")
        
        # Verify bag contents
        print("\nBag file contents:")
        info_cmd = f"rosbag info {output_bag}"
        os.system(info_cmd)
        
    except Exception as e:
        print(f"Error creating ROS bag: {str(e)}")
