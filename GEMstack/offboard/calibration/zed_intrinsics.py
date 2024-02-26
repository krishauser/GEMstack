"""To obtain the intrinsics of the zed2 stereo camera

References:
- ROS client API methods: https://docs.ros.org/en/diamondback/api/rospy/html/rospy.client-module.html
- Zed ROS wrapper: https://wiki.ros.org/zed-ros-wrapper
- Message format: http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/CameraInfo.html
"""

import rospy
from sensor_msgs.msg import CameraInfo

import yaml

def get_intrinsics():
	rospy.init_node('zed2_intrinsics', disable_signals=True)
	message = rospy.wait_for_message('/zed2/zed_node/depth/camera_info', CameraInfo)
	return message

def write_intrinsics(message, file):
	camera_info = {
		'height': message.height,
		'width': message.width,
		'K': message.K
	}
	with open(file, 'w') as yaml_file:
		yaml.dump(camera_info, yaml_file)

if __name__ == '__main__':
	message = get_intrinsics()
	write_intrinsics(message, '../../knowledge/calibration/zed_intrinsics.yaml')
