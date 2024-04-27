import os
import rosbag
import cv2
from cv_bridge import CvBridge
import open3d as o3d
import numpy as np
import gpxpy
import gpxpy.gpx
from datetime import datetime
import zipfile
import sensor_msgs.point_cloud2 as pc2

import yaml
import json

DATASET_UPLOAD_FOLDER = '../dataset'


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


def convert(bag_file, video=True, pcd=True):
    file_name = os.path.splitext(os.path.basename(bag_file))[0]
    path = os.path.join(DATASET_UPLOAD_FOLDER, file_name)
    os.makedirs(path, exist_ok=True)

    bag = rosbag.Bag(bag_file)
    topics = bag.get_type_and_topic_info().topics
    for topic in topics:
        if 'sensor_msgs/image' in topics[topic].msg_type.lower():
            to_image_png(bag, topic, file_name)
            if video:
                to_video(bag, topic, file_name, False)
        elif 'pointcloud' in topics[topic].msg_type.lower():
            if pcd:
                to_pointcloud_pcd(bag, topic, file_name)
            to_pointcloud_bin(bag, topic, file_name)
        elif 'gnss' in topics[topic].msg_type.lower():
            to_gnss(bag, topic, file_name)
        elif 'imu' in topics[topic].msg_type.lower():
            to_json(bag, topic, file_name)
        elif 'compressed' in topics[topic].msg_type.lower() and 'Depth' not in topic:
            to_image_jpg(bag, topic, file_name)
            if video:
                to_video(bag, topic, file_name, True)
        elif 'pacmod' in topics[topic].msg_type.lower():
            to_json(bag, topic, file_name)

    bag.close()

def to_image_jpg(bag, topic, file_name):
    path = os.path.join(DATASET_UPLOAD_FOLDER, file_name + '/' + topic[1:].replace('/', '_') + '/images')
    os.makedirs(path, exist_ok=True)

    for _, msg, t in bag.read_messages(topics=[topic]):
        with open(os.path.join(path, str(t) + '.jpg'), 'wb') as f:
            f.write(msg.data)


def to_image_png(bag, topic, file_name):
    bridge = CvBridge()
    path = os.path.join(DATASET_UPLOAD_FOLDER, file_name + '/' + topic[1:].replace('/', '_') + '/images')
    os.makedirs(path, exist_ok=True)

    for _, msg, t in bag.read_messages(topics=[topic]):
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        cv2.imwrite(os.path.join(path, str(t) + '.png'), cv_image)


def to_video(bag, topic, file_name, compressed):
    bridge = CvBridge()
    path = os.path.join(DATASET_UPLOAD_FOLDER, file_name + '/' + topic[1:].replace('/', '_') + '/video')
    os.makedirs(path, exist_ok=True)

    images = []
    timestamp = []
    for _, msg, t in bag.read_messages(topics=[topic]):
        timestamp.append(t.to_sec())
        if compressed:
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        else:
            cv_image = bridge.imgmsg_to_cv2(msg, msg.encoding)
        images.append(cv_image)

    fps = len(timestamp) / (timestamp[-1] - timestamp[0])
    image_shape = images[0].shape
    h, w = image_shape[0], image_shape[1]
    channel = 1 if len(image_shape) == 2 else image_shape[2]
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    video_writer = cv2.VideoWriter(os.path.join(path, 'video.mp4'), fourcc, fps, (w, h), isColor=channel > 1)
    for image in images:
        video_writer.write(image)
    video_writer.release()


def to_pointcloud_pcd(bag, topic, file_name):
    path = os.path.join(DATASET_UPLOAD_FOLDER, file_name + '/' + topic[1:].replace('/', '_') + '/pcd')
    os.makedirs(path, exist_ok=True)

    for _, msg, t in bag.read_messages(topics=[topic]):
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(np.frombuffer(msg.data, dtype=np.float32).reshape(-1, 3))
        o3d.io.write_point_cloud(os.path.join(path, str(t) + '.pcd'), pcd)


def to_pointcloud_bin(bag, topic, file_name):
    path = os.path.join(DATASET_UPLOAD_FOLDER, file_name + '/' + topic[1:].replace('/', '_') + '/bin')
    os.makedirs(path, exist_ok=True)

    for _, msg, t in bag.read_messages(topics=[topic]):
        cloud_data = []
        for point in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            cloud_data.append([point[0], point[1], point[2]])
        cloud_data = np.array(cloud_data, dtype=np.float32)
        with open(os.path.join(path, str(t) + '.bin'), 'wb') as f:
            f.write(cloud_data.tobytes())


def to_gnss(bag, topic, file_name):
    path = os.path.join(DATASET_UPLOAD_FOLDER, file_name + '/' + topic[1:].replace('/', '_'))
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


def to_imu(bag, topic, file_name):
    path = os.path.join(DATASET_UPLOAD_FOLDER, file_name + '/' + topic[1:].replace('/', '_'))
    os.makedirs(path, exist_ok=True)

    imu_data = [['time', 'orientation_x', 'orientation_y', 'orientation_z', 'orientation_w', 'angular_velocity_x',
                 'angular_velocity_y', 'angular_velocity_z', 'linear_acceleration_x', 'linear_acceleration_y',
                 'linear_acceleration_z']]
    for _, msg, t in bag.read_messages(topics=[topic]):
        imu_data.append([t.to_sec(), msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w,
                         msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z,
                         msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])

    np.savetxt(os.path.join(path, 'imu.csv'), imu_data, delimiter=',', fmt='%s')


def to_json(bag, topic, file_name):
    path = os.path.join(DATASET_UPLOAD_FOLDER, file_name + '/' + topic[1:].replace('/', '_'))
    os.makedirs(path, exist_ok=True)

    for _, msg, t in bag.read_messages(topics=[topic]):
        data = yaml.safe_load(str(msg))
        with open(os.path.join(path, str(t) + '.json'), 'w') as f:
            json.dump(data, f)


def create_zip_for_topic(topic_folder):
    # Get the list of subdirectories within the topic folder
    subdirectories = [d for d in os.listdir(topic_folder) if os.path.isdir(os.path.join(topic_folder, d))]
    # Create a list to store the paths of the generated zip files
    zip_file_paths = []

    for subdirectory in subdirectories:
        subdirectory_path = os.path.join(topic_folder, subdirectory)

        # Check if the subdirectory is itself a dataset or contains further subdirectories
        if any(os.path.isdir(os.path.join(subdirectory_path, item)) for item in os.listdir(subdirectory_path)):
            # If it contains subdirectories, create a zip file for each subdirectory
            for item in os.listdir(subdirectory_path):
                item_path = os.path.join(subdirectory_path, item)
                if os.path.isdir(item_path):
                    zip_file_path = os.path.join(topic_folder, f"{subdirectory}@{item}.zip")
                    with zipfile.ZipFile(zip_file_path, 'w') as zipf:
                        for root, _, files in os.walk(item_path):
                            for file in files:
                                zipf.write(os.path.join(root, file),
                                           os.path.relpath(os.path.join(root, file), item_path))
                    zip_file_paths.append(zip_file_path)
        else:
            # If it's a direct dataset, create a zip file for the entire subdirectory
            zip_file_path = os.path.join(topic_folder, f"{subdirectory}.zip")
            with zipfile.ZipFile(zip_file_path, 'w') as zipf:
                for root, _, files in os.walk(subdirectory_path):
                    for file in files:
                        zipf.write(os.path.join(root, file),
                                   os.path.relpath(os.path.join(root, file), subdirectory_path))
            zip_file_paths.append(zip_file_path)

    return zip_file_paths
