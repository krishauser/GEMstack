from cv_bridge import CvBridge
from sensor_msgs.msg import Image, PointCloud2
from ultralytics import YOLO
from fusion_utils import *
import rospy
import message_filters
import os
import tf

class Fusion3D():
    def __init__(self):
        # Setup variables
        self.bridge = CvBridge()
        self.detector = YOLO(os.getcwd()+'/GEMstack/knowledge/detection/yolov8n.pt')
        self.last_person_boxes = [] 
        self.pedestrians = {}
        self.visualization = True
        self.confidence = 0.7
        self.classes_to_detect = 0

        # Load calibration data
        self.R = load_extrinsics(os.getcwd() + '/GEMstack/onboard/perception/calibration/extrinsics/R.npy')
        self.t = load_extrinsics(os.getcwd() + '/GEMstack/onboard/perception/calibration/extrinsics/t.npy')
        self.K = load_intrinsics(os.getcwd()+ '/GEMstack/onboard/perception/calibration/camera_intrinsics.json')

        # Subscribers and sychronizers
        self.rgb_rosbag = message_filters.Subscriber('/oak/rgb/image_raw', Image)
        self.top_lidar_rosbag = message_filters.Subscriber('/ouster/points', PointCloud2)
        self.sync = message_filters.ApproximateTimeSynchronizer([self.rgb_rosbag, self.top_lidar_rosbag], queue_size=10, slop=0.1)
        self.sync.registerCallback(self.fusion_callback)

        # TF listener to get transformation from LiDAR to Camera
        self.tf_listener = tf.TransformListener()

        # Publishers
        if(self.visualization):
            self.pub_image = rospy.Publisher("/camera/image_detection", Image, queue_size=1)



    def fusion_callback(self, image: Image, lidar_pc2_msg: PointCloud2):
        image = self.bridge.imgmsg_to_cv2(image, "bgr8") 
        track_result = self.detector.track(source=image, classes=self.classes_to_detect, persist=True, conf=self.confidence)

        self.last_person_boxes = []
        boxes = track_result[0].boxes

        # Unpacking box dimentions detected into x,y,w,h
        for box in boxes:
            xywh = box.xywh[0].tolist()
            self.last_person_boxes.append(xywh)

            # Used for visualization
            if(self.visualization):
                image = vis_2d_bbox(image, xywh, box)

        # Convert 1D PointCloud2 data to x, y, z coords
        lidar_points = convert_pointcloud2_to_xyz(lidar_pc2_msg)
    
        # Transform LiDAR points into the camera coordinate frame.
        lidar_in_camera = transform_lidar_points(lidar_points, self.R, self.t)
    
        # Project the transformed points into the image plane.
        projected_pts = project_points(lidar_in_camera, self.K)
        
        # Draw projected LiDAR points on the image.
        for pt in projected_pts:
            cv2.circle(image, pt, 2, (0, 0, 255), -1)

        # visualize_point_cloud(p_img_cloud)
        
        # Used for visualization
        if(self.visualization):
            ros_img = self.bridge.cv2_to_imgmsg(image, 'bgr8')
            self.pub_image.publish(ros_img)  



if __name__ == '__main__':
    rospy.init_node('fusion_node', anonymous=True)
    Fusion3D()
    while not rospy.core.is_shutdown():
        rospy.rostime.wallsleep(0.5)

