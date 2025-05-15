from jsk_recognition_msgs.msg import BoundingBox
from scipy.spatial.transform import Rotation as R

# Seperated Sensor Fusion specific utilities into it's own file because package imports are required 
# (to minimize integration problems for groups who aren't using sensor fusion)

# Adds a new bounding box to a BoundingBoxArray
def add_bounding_box(boxes, frame_id, stamp, x, y, z, l, w, h, yaw, conf_score, label):
    box_msg = BoundingBox()
    box_msg.header.frame_id = frame_id
    box_msg.header.stamp = stamp
    
    # Set the pose
    box_msg.pose.position.x = float(x)
    box_msg.pose.position.y = float(y)
    box_msg.pose.position.z = float(z)
    
    # Convert yaw to quaternion
    quat = R.from_euler('z', yaw).as_quat()
    box_msg.pose.orientation.x = float(quat[0])
    box_msg.pose.orientation.y = float(quat[1])
    box_msg.pose.orientation.z = float(quat[2])
    box_msg.pose.orientation.w = float(quat[3])
    
    # Set the dimensions
    # Swapped dims[2] and dims[0]
    box_msg.dimensions.x = float(l)  # length
    box_msg.dimensions.y = float(w)  # width
    box_msg.dimensions.z = float(h)  # height

    # Add confidence score and label
    box_msg.value = float(conf_score)
    box_msg.label = label
    
    boxes.boxes.append(box_msg)
    return boxes
