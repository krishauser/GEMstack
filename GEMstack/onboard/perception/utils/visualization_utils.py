from sensor_msgs.msg import PointField
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from .constants import *
import sensor_msgs.point_cloud2 as pc2
import cv2
import rospy
import struct
import math
import tf.transformations as tf_trans


def vis_2d_bbox(image, xywh, box):
    # Setup
    label_text = "Cone "
    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = 0.5
    font_color = (255, 255, 255)
    line_type = 1
    text_thickness = 2

    x, y, w, h = xywh

    if box.id is not None:
        id = box.id.item()
    else:
        id = 0

    # Draw bounding box
    cv2.rectangle(image, (int(x - w / 2), int(y - h / 2)), (int(x + w / 2), int(y + h / 2)), (255, 0, 255), 3)

    # Define text label
    x = int(x - w / 2)
    y = int(y - h / 2)
    label = label_text + str(id) + " : " + str(round(box.conf.item(), 2))

    # Get text size
    text_size, baseline = cv2.getTextSize(label, font, font_scale, line_type)
    text_w, text_h = text_size

    # Position text above the bounding box
    text_x = x
    text_y = y - 10 if y - 10 > 10 else y + h + text_h

    # Draw main text on top of the outline
    cv2.putText(image, label, (text_x, text_y - baseline), font, font_scale, font_color, text_thickness)
    return image

def create_point_cloud(points, color=(255, 0, 0), ref_frame="map"):
    """
    Converts a list of (x, y, z) points into a PointCloud2 message.
    """
    header = rospy.Header()
    header.stamp = rospy.Time.now()
    header.frame_id = ref_frame  # Change to your TF frame

    fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="rgb", offset=12, datatype=PointField.FLOAT32, count=1),
    ]

    # Convert RGB color to packed float32
    r, g, b = color
    packed_color = struct.unpack('f', struct.pack('I', (r << 16) | (g << 8) | b))[0]

    point_cloud_data = [(x, y, z, packed_color) for x, y, z in points]

    return pc2.create_cloud(header, fields, point_cloud_data)

def yaw_to_quaternion(yaw):
    """Convert yaw (in radians) to a normalized quaternion (x, y, z, w)."""
    return tf_trans.quaternion_from_euler(0, 0, yaw)

def create_markers(poses, dimensions, color = (0.0, 1.0, 1.5, 0.2), ns="markers", ref_frame="map"):
    """
    Create 3D bbox markers from centroids and dimensions
    """
    marker_array = MarkerArray()

    for i, (pose, dimension) in enumerate(zip(poses, dimensions)):
        # Skip if no centroid or dimension
        if (pose == None) or (dimension == None):
            continue
            
        marker = Marker()
        marker.header.frame_id = ref_frame  # Reference frame
        marker.header.stamp = rospy.Time.now()
        marker.ns = ns
        marker.id = i  # Unique ID for each marker
        marker.type = Marker.CUBE  # Cube for bounding box
        marker.action = Marker.ADD

        # Position (center of the bounding box)
        c_x, c_y, c_z, yaw = pose
        if (not isinstance(c_x, float)) or (not isinstance(c_y, float)) or (not isinstance(c_z, float)):
            continue

        marker.pose.position.x = c_x
        marker.pose.position.y = c_y
        marker.pose.position.z = c_z

        # Orientation (default, no rotation)
        q = yaw_to_quaternion(yaw)
        marker.pose.orientation.x = q[0]
        marker.pose.orientation.y = q[1]
        marker.pose.orientation.z = q[2]
        marker.pose.orientation.w = q[3]
        # Bounding box dimensions
        d_x, d_y, d_z = dimension
        if (not isinstance(d_x, float)) or (not isinstance(d_y, float)) or (not isinstance(d_z, float)):
            continue

        marker.scale.x = d_x
        marker.scale.y = d_y
        marker.scale.z = d_z

        # Random colors for each bounding box
        r, g, b, a = color
        marker.color.r = r  # Varying colors
        marker.color.g = g
        marker.color.b = b
        marker.color.a = a  # Transparency

        marker.lifetime = rospy.Duration()  # Persistent
        marker_array.markers.append(marker)
    return marker_array


def create_polygon_marker(vertices_2d, ref_frame="map"):
    marker_array = MarkerArray()

    marker = Marker()
    marker.header.frame_id = ref_frame
    marker.header.stamp = rospy.Time.now()
    marker.ns = "polygon"
    marker.id = 0
    marker.type = Marker.LINE_STRIP
    marker.action = Marker.ADD

    # Style
    marker.scale.x = 0.1  # Line width

    # Color (blue)
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    marker.color.a = 1.0

    marker.pose.orientation.w = 1.0

    # Convert 2D vertices into geometry_msgs/Point with z=0.0
    for vx, vy in vertices_2d:
        marker.points.append(Point(x=vx, y=vy, z=0.0))

    # Close the loop by repeating the first point
    marker.points.append(Point(x=vertices_2d[0][0], y=vertices_2d[0][1], z=0.0))

    marker_array.markers.append(marker)
    return marker_array


def create_polygon_markers(grouped_vertices_2d, ref_frame="map"):
    marker_array = MarkerArray()

    for idx, vertices_2d in enumerate(grouped_vertices_2d):
        marker = Marker()
        marker.header.frame_id = ref_frame
        marker.header.stamp = rospy.Time.now()
        marker.ns = "polygon"
        marker.id = idx  # Unique ID for each polygon
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD

        # Style
        marker.scale.x = 0.1  # Line width

        # Color (green)
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        marker.pose.orientation.w = 1.0

        # Convert 2D vertices into geometry_msgs/Point with z=0.0
        for vertex in vertices_2d:
            vx, vy = vertex[0], vertex[1]
            marker.points.append(Point(x=vx, y=vy, z=0.0))

        # Close the loop by repeating the first point
        if vertices_2d:
            marker.points.append(Point(x=vertices_2d[0][0], y=vertices_2d[0][1], z=0.0))

        marker_array.markers.append(marker)

    return marker_array


def create_parking_spot_marker(closest_spot, length=GEM_E4_LENGTH, width=GEM_E4_WIDTH, ref_frame="map"):
    marker_array = MarkerArray()

    marker = Marker()
    marker.header.frame_id = ref_frame
    marker.header.stamp = rospy.Time.now()
    marker.ns = "parking_spot"
    marker.id = 0
    marker.type = Marker.TRIANGLE_LIST
    marker.action = Marker.ADD

    # Transparency and color (green)
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 1.0
    marker.color.a = 0.5

    marker.pose.orientation.w = 1.0

    # Not used in TRIANGLE_LIST, but required
    marker.scale.x = 1.0
    marker.scale.y = 1.0
    marker.scale.z = 1.0

    x, y, theta_deg = closest_spot

    # Convert orientation to radians
    theta = math.radians(theta_deg)

    # Half dimensions
    dx = length / 2.0
    dy = width / 2.0

    # Define rectangle corners (local frame, clockwise)
    local_corners = [
        (-dx, -dy),
        (-dx, dy),
        (dx, dy),
        (dx, -dy)
    ]

    # Transform to global frame
    global_corners = []
    for lx, ly in local_corners:
        gx = x + lx * math.cos(theta) - ly * math.sin(theta)
        gy = y + lx * math.sin(theta) + ly * math.cos(theta)
        global_corners.append(Point(x=gx, y=gy, z=0.0))

    # Define two triangles to fill the rectangle
    marker.points.append(global_corners[0])
    marker.points.append(global_corners[2])
    marker.points.append(global_corners[1])

    marker.points.append(global_corners[0])
    marker.points.append(global_corners[3])
    marker.points.append(global_corners[2])

    marker_array.markers.append(marker)
    return marker_array

def create_parking_goal_marker(x, y, radius=0.3, ref_frame="map", color=(0.7, 1.0, 0.5, 1.0)):
    marker_array = MarkerArray()

    marker = Marker()
    marker.header.frame_id = ref_frame
    marker.header.stamp = rospy.Time.now()
    marker.ns = "parking_goal"
    marker.id = 0
    marker.type = Marker.CYLINDER
    marker.action = Marker.ADD

    # Position at (x, y), ignore yaw
    marker.pose.position.x = x
    marker.pose.position.y = y
    marker.pose.position.z = 0.0  # Flat on ground
    marker.pose.orientation.w = 1.0  # No rotation

    # Scale (diameter in x and y, small height in z to make it flat)
    marker.scale.x = 2 * radius
    marker.scale.y = 2 * radius
    marker.scale.z = 0.05  # Thin circle

    # Color (RGBA)
    marker.color.r = color[0]
    marker.color.g = color[1]
    marker.color.b = color[2]
    marker.color.a = color[3]

    marker.lifetime = rospy.Duration(0)  # 0 means forever

    marker_array.markers.append(marker)
    return marker_array


def delete_markers(ns="markers", max_markers=15):
    marker_array = MarkerArray()
    for i in range(max_markers):
        marker = Marker()
        marker.ns = ns
        marker.id = i
        marker.action = Marker.DELETE
        marker_array.markers.append(marker)
    return marker_array