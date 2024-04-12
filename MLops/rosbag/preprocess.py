import os
import rosbag
import cv2
from cv_bridge import CvBridge
import open3d as o3d
import numpy as np
import gpxpy
import gpxpy.gpx
from datetime import datetime


def get_topics(bag_file):
    bag = rosbag.Bag(bag_file)
    topics = bag.get_type_and_topic_info().topics
    topics = list(topics.keys())
    bag.close()
    return topics


def get_types(bag_file):
    bag = rosbag.Bag(bag_file)
    topics = bag.get_type_and_topic_info().topics
    type_lst = []
    for topic in topics:
        type_lst.append(topics[topic].msg_type.lower())
    bag.close()
    return type_lst


def convert(bag_file, video=False):
    path = os.path.join(os.getcwd(), 'output')
    os.makedirs(path, exist_ok=True)

    bag = rosbag.Bag(bag_file)
    topics = bag.get_type_and_topic_info().topics
    for topic in topics:
        if 'image' in topics[topic].msg_type.lower():
            to_image(bag, topic)
            if video:
                to_video(bag, topic)
        elif 'pointcloud' in topics[topic].msg_type.lower():
            to_pointcloud(bag, topic)
        elif 'gnss' in topics[topic].msg_type.lower():
            to_gnss(bag, topic)

    bag.close()


def to_image(bag, topic):
    bridge = CvBridge()
    path = os.path.join(os.getcwd(), 'output/' + topic[1:].replace('/', '_') + '/images')
    os.makedirs(path, exist_ok=True)

    for _, msg, t in bag.read_messages(topics=[topic]):
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        cv2.imwrite(os.path.join(path, str(t) + '.png'), cv_image)


def to_video(bag, topic):
    bridge = CvBridge()
    path = os.path.join(os.getcwd(), 'output/' + topic[1:].replace('/', '_') + '/video')
    os.makedirs(path, exist_ok=True)

    images = []
    for _, msg, t in bag.read_messages(topics=[topic]):
        cv_image = bridge.imgmsg_to_cv2(msg, msg.encoding)
        images.append(cv_image)

    image_shape = images[0].shape
    h, w = image_shape[0], image_shape[1]
    channel = 1 if len(image_shape) == 2 else image_shape[2]
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    video_writer = cv2.VideoWriter(os.path.join(path, 'video.mp4'), fourcc, 30, (w, h), isColor=channel > 1)
    for image in images:
        video_writer.write(image)
    video_writer.release()


def to_pointcloud(bag, topic):
    path = os.path.join(os.getcwd(), 'output/' + topic[1:].replace('/', '_'))
    os.makedirs(path, exist_ok=True)

    for _, msg, t in bag.read_messages(topics=[topic]):
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(np.frombuffer(msg.data, dtype=np.float32).reshape(-1, 3))
        o3d.io.write_point_cloud(os.path.join(path, str(t) + '.pcd'), pcd)


def to_gnss(bag, topic):
    path = os.path.join(os.getcwd(), 'output/' + topic[1:].replace('/', '_'))
    os.makedirs(path, exist_ok=True)

    gpx = gpxpy.gpx.GPX()
    track = gpxpy.gpx.GPXTrack()
    gpx.tracks.append(track)
    segment = gpxpy.gpx.GPXTrackSegment()
    track.segments.append(segment)

    for _, msg, t in bag.read_messages(topics=[topic]):
        segment.points.append(gpxpy.gpx.GPXTrackPoint(latitude=msg.latitude, longitude=msg.longitude,
                                                      elevation=msg.height, time=datetime.fromtimestamp(t.to_sec())))

    with open(os.path.join(path, 'gnss.gpx'), 'w') as f:
        f.write(gpx.to_xml())


#convert('vehicle_withLidarGNSS.bag', video=True)
