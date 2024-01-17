#!/usr/bin/env python
# Prints total cumulative serialized msg size in bytes per topic
# Calculation takes approximately 1sec/GB
# Usage:   python rosbag_size_by_topic.py BAG_FILE_PATH
#
#from https://answers.ros.org/question/318667/using-rosbag-to-get-size-of-each-topic/

import rosbag
import sys

topic_size_dict = {}
for topic, msg, time in rosbag.Bag(sys.argv[1], "r").read_messages(raw=True):
    topic_size_dict[topic] = topic_size_dict.get(topic, 0) + len(msg[1])

topic_size = list(topic_size_dict.items())
topic_size.sort(key=lambda x: x[1], reverse=False)

for topic, size in topic_size:
    if size < 1000:
        print("{:7d}B  {}".format(size, topic))
    elif size >= 1000000000000:
        print("{:7.2f}T  {}".format(size/1000000000000, topic))
    elif size >= 1000000000:
        print("{:7.2f}G  {}".format(size/1000000000, topic))
    elif size >= 1000000:
        print("{:7.2f}M  {}".format(size/1000000, topic))
    elif size >= 1000:
        print("{:7.2f}K  {}".format(size/1000, topic))