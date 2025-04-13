#%%
import rosbag
import os
import bisect
from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs import point_cloud2
import cv2
from cv_bridge import CvBridge
import numpy as np

def write_pcd(pc_msg, filename):
    # Read points from PointCloud2 message
    points = list(point_cloud2.read_points(pc_msg, field_names=("x", "y", "z"), skip_nans=True))
    # Write PCD header
    header = f"""VERSION .7
FIELDS x y z
SIZE 4 4 4
TYPE F F F
COUNT 1 1 1
WIDTH {len(points)}
HEIGHT 1
VIEWPOINT 0 0 0 1 0 0 0
POINTS {len(points)}
DATA ascii
"""
    
    # Write to file
    with open(filename, 'w') as f:
        f.write(header)
        for p in points:
            if p[0] == 0 and p[1] == 0 and p[2] == 0: continue
            f.write(f"{p[0]} {p[1]} {p[2]}\n")

def process_bag(bag_path, output_dir='sync_dataset', time_threshold=0.05):
    bag = rosbag.Bag(bag_path)
    bridge = CvBridge()

    # Topic configuration
    topic_map = {
        '/camera_fl/arena_camera_node/image_raw': 'fl/images',
        '/camera_rl/arena_camera_node/image_raw': 'rl/images',
        '/camera_rr/arena_camera_node/image_raw': 'rr/images',
        '/oak/rgb/image_raw': 'oak/images',
        '/ouster/points': 'pc'
    }

    # Create output directories
    for d in ['fl', 'rl', 'rr', 'oak', 'pc']:
        os.makedirs(os.path.join(output_dir, d), exist_ok=True)

    # Load and sort all messages
    messages = {topic: [] for topic in topic_map.keys()}
    for topic, msg, t in bag.read_messages(topics=topic_map.keys()):
        stamp = msg.header.stamp.to_sec() if hasattr(msg, 'header') else t.to_sec()
        messages[topic].append((stamp, msg))

    # Sort each topic by timestamp
    for topic in messages:
        messages[topic].sort(key=lambda x: x[0])

    # Reference topic (point cloud)
    ref_topic = '/ouster/points'
    group_index = 0

    for pc_stamp, pc_msg in messages[ref_topic]:
        timestamps = {'pc': pc_stamp}
        entry = {}

        # Find closest camera messages
        for cam_topic in topic_map:
            if cam_topic == ref_topic:
                continue
            cam_msgs = messages.get(cam_topic, [])
            if not cam_msgs:
                continue
            stamps = [s for s, m in cam_msgs]
            idx = bisect.bisect_left(stamps, pc_stamp)
            
            closest = None
            closest_stamp = None
            
            # Check next message
            if idx < len(stamps):
                if abs(stamps[idx] - pc_stamp) < time_threshold:
                    closest = cam_msgs[idx][1]
                    closest_stamp = stamps[idx]
            
            # Check previous message if not found
            if not closest and idx > 0:
                if abs(stamps[idx-1] - pc_stamp) < time_threshold:
                    closest = cam_msgs[idx-1][1]
                    closest_stamp = stamps[idx-1]
            
            if closest:
                entry[cam_topic] = (closest_stamp, closest)
                timestamps[topic_map[cam_topic]] = closest_stamp

        # Save only if we have all sensors
        if len(entry) == 4:  # 4 camera topics + point cloud
            # Save images and point cloud
            frame_id = f"{group_index:06d}"
            
            # Print timestamps
            print(f"Saved group {group_index} with timestamps:")
            for sensor, ts in timestamps.items():
                print(f"  {sensor}: {ts:.6f}")
            
            # Save images
            for topic, (stamp, msg) in entry.items():
                output_subdir = topic_map[topic]
                output_path = os.path.join(output_dir, output_subdir, f"{frame_id}.png")
                cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
                cv2.imwrite(output_path, cv_img)
            
            # Save point cloud
            pc_path = os.path.join(output_dir, 'pc', f"{frame_id}.pcd")
            write_pcd(pc_msg, pc_path)
            
            group_index += 1

    bag.close()

# Example usage
process_bag('/mnt/GEMstack/data/mybag_2025-04-01-17-41-55.bag','/mnt/GEMstack/GEMstack/offboard/calibration/calibration_by_SfM/data')