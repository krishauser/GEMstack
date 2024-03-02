from ...utils import settings
import rospy
import yaml
from sensor_msgs.msg import CameraInfo

rospy.init_node('zed2_intrinsics', disable_signals=True)
message = rospy.wait_for_message('/zed2/zed_node/rgb/camera_info', CameraInfo)
file = settings.get('calibration.zed_intrinsics')
camera_info = {
		'height': message.height,
		'width': message.width,
		'K': message.K
	}
with open(file, 'w') as yaml_file:
    yaml.dump(camera_info, yaml_file)