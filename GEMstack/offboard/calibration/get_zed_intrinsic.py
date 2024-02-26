import rospy
from sensor_msgs.msg import CameraInfo

import json

if __name__ == '__main__':
	rospy.init_node('zed2_intrinsics', disable_signals=True)
	message = rospy.wait_for_message('/zed2/zed_node/depth/camera_info', CameraInfo)
	camera_info = {
		'intrinsic_matrix': message.K,
		'height': message.height,
		'width': message.width,
	}
	with open("../../knowledge/calibration/zed_intrinsic.json", 'w') as f:
		json.dump(camera_info, f)