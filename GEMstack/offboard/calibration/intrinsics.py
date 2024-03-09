import rospy
from sensor_msgs.msg import CameraInfo
import pickle

if __name__ == '__main__':
	rospy.init_node('intrinsics', disable_signals=True)
	msg = rospy.wait_for_message('/zed2/zed_node/depth/camera_info', CameraInfo)
	camera_info = {
		'intrinsic_matrix': msg.K,
		'height': msg.height,
		'width': msg.width,
	}

	values = {
		'fx': msg.K[0],
		'fy': msg.K[4],
		'cx': msg.K[2],
		'cy': msg.K[5],
		'height': msg.height,
		'width': msg.width
	}

	with open("../../knowledge/calibration/intrinsic.pickle", 'wb') as f:
		pickle.dump(camera_info, f)

	with open("../../knowledge/calibration/values.pickle", 'wb') as f:
		pickle.dump(values, f)
