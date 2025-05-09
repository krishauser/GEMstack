#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Copyright 2016 Massachusetts Institute of Technology

# 1-16-2024: Kris Hauser: Modified to extract multiple images from multiple topics

"""Extract images from a rosbag.
"""

import os
import argparse

import cv2

import rosbag
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def main():
    """Extract a folder of images from a rosbag.
    """
    parser = argparse.ArgumentParser(description="Extract images from a ROS bag.")
    parser.add_argument("bag_file", help="Input ROS bag.")
    parser.add_argument("output_dir", help="Output directory.")
    parser.add_argument('-t',"--topics", help="Image topic.", default="/zed2/zed_node/depth/depth_registered,/zed2/zed_node/rgb/image_rect_color")
    parser.add_argument('-o','--output_prefix', help="Output prefix(es).", default="")

    args = parser.parse_args()
    topics = args.topics.split(',')
    output_prefixes = args.output_prefix.split(',')
    if args.output_prefix=='':
        output_prefixes = [topic.split('/')[-1] for topic in topics]
        if len(set(output_prefixes)) != len(output_prefixes):
            print("Got multiple topics with the same name, but no output prefixes specified.") 
            return 1
    if len(topics) != len(output_prefixes):
        print("Got %i topics but %i output prefixes." % (len(topics), len(output_prefixes)))
        return 

    print("Extract images from %s on topics %s into %s" % (args.bag_file,
                                                          ', '.join(topics), args.output_dir))

    bag = rosbag.Bag(args.bag_file, "r")
    bridge = CvBridge()
    count = 0
    for topic, msg, t in bag.read_messages(topics=topics):
        cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

        i = topics.index(topic)
        prefix = output_prefixes[i]
        cv2.imwrite(os.path.join(args.output_dir, "%s%06i.png" % (prefix,count)), cv_img)
        print("Wrote image %s %i" % (prefix,count))

        count += 1

    bag.close()

    return 0

if __name__ == '__main__':
    exit(main())