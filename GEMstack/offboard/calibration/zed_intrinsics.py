"""To obtain the intrinsic parameters of the zed2 stereo camera

References:
- ROS client API methods: https://docs.ros.org/en/diamondback/api/rospy/html/rospy.client-module.html
- Zed ROS wrapper: https://wiki.ros.org/zed-ros-wrapper
- Message format: http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/CameraInfo.html
"""

from ...utils import settings

import rospy
from sensor_msgs.msg import CameraInfo

import yaml

def get_intrinsics():
	rospy.init_node('zed2_intrinsics', disable_signals=True)
	message = rospy.wait_for_message('/zed2/zed_node/rgb/camera_info', CameraInfo)
	return message

def write_intrinsics(message, file):
	K = message.K
	camera_info = {
		'height': message.height,
		'width': message.width,
		'K': K.tolist()
	}
	with open(file, 'w') as yaml_file:
		yaml.dump(camera_info, yaml_file)

def main():
	message = get_intrinsics()
	file = settings.get('calibration.zed_intrinsics')
	write_intrinsics(message, file)
