from sensor_msgs.msg import PointField
from visualization_msgs.msg import Marker, MarkerArray
import sensor_msgs.point_cloud2 as pc2
import cv2
import rospy
import struct

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

def create_point_cloud(points, color=(255, 0, 0)):
    """
    Converts a list of (x, y, z) points into a PointCloud2 message.
    """
    header = rospy.Header()
    header.stamp = rospy.Time.now()
    header.frame_id = "os_sensor"  # Change to your TF frame

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

def create_bbox_marker(centroids, dimensions):
    """
    Create 3D bbox markers from centroids and dimensions
    """
    marker_array = MarkerArray()

    for i, (centroid, dimension) in enumerate(zip(centroids, dimensions)):
        # Skip if no centroid or dimension
        if (centroid == None) or (dimension == None):
            continue
            
        marker = Marker()
        marker.header.frame_id = "top_lidar"  # Reference frame
        marker.header.stamp = rospy.Time.now()
        marker.ns = "bounding_boxes"
        marker.id = i  # Unique ID for each marker
        marker.type = Marker.CUBE  # Cube for bounding box
        marker.action = Marker.ADD

        # Position (center of the bounding box)
        c_x, c_y, c_z = centroid
        if (not isinstance(c_x, float)) or (not isinstance(c_y, float)) or (not isinstance(c_z, float)):
            continue

        marker.pose.position.x = c_x
        marker.pose.position.y = c_y
        marker.pose.position.z = c_z

        # Orientation (default, no rotation)
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        # Bounding box dimensions
        d_x, d_y, d_z = dimension
        if (not isinstance(d_x, float)) or (not isinstance(d_y, float)) or (not isinstance(d_z, float)):
            continue

        marker.scale.x = d_x
        marker.scale.y = d_y
        marker.scale.z = d_z

        # Random colors for each bounding box
        marker.color.r = 0.0  # Varying colors
        marker.color.g = 1.0
        marker.color.b = 1.5
        marker.color.a = 0.2  # Transparency

        marker.lifetime = rospy.Duration()  # Persistent
        marker_array.markers.append(marker)
    return marker_array


def delete_bbox_marker():
    """
    Delete 3D bbox markers given ID ranges
    """
    marker_array = MarkerArray()
    for i in range(15):
        marker = Marker()
        marker.ns = "bounding_boxes"
        marker.id = i
        marker.action = Marker.DELETE
        marker_array.markers.append(marker)
    return marker_array