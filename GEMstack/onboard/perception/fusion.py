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

        # Convert 1D PointCloud2 data to x, y, z coords
        lidar_points = convert_pointcloud2_to_xyz(lidar_pc2_msg)
    
        # Transform LiDAR points into the camera coordinate frame.
        lidar_in_camera = transform_lidar_points(lidar_points, self.R, self.t)
    
        # Project the transformed points into the image plane.
        projected_pts = project_points(lidar_in_camera, self.K)

        # Convert numpy array to Open3D point cloud
        transformed_points = projected_pts
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(transformed_points)

        # Apply voxel grid downsampling
        voxel_size = 0.1  # Adjust for desired resolution
        downsampled_pcd = pcd.voxel_down_sample(voxel_size=voxel_size)

        # Convert back to numpy array
        transformed_points = np.asarray(downsampled_pcd.points)
        print(f"after :{transformed_points.shape}")
        
        # Process bboxes
        self.last_person_boxes = []
        boxes = track_result[0].boxes

        # Unpacking box dimentions detected into x,y,w,h
        all_extracted_pts = []
        for box in boxes:
            xywh = box.xywh[0].tolist()
            self.last_person_boxes.append(xywh)

            # Extracting projected pts
            x, y, w, h = xywh
            left_bound = int(x - w / 2)
            right_bound = int(x + w / 2)
            top_bound = int(y - h / 2)
            bottom_bound = int(y + h / 2)

            if len(projected_pts) > 0:
                pts = np.array(projected_pts)
                extracted_pts = pts[(pts[:, 0] > left_bound) &
                                    (pts[:, 0] < right_bound) &
                                    (pts[:, 1] > top_bound) &
                                    (pts[:, 1] < bottom_bound)
                                    ]
                
                all_extracted_pts = all_extracted_pts + list(extracted_pts)
            
            # Used for visualization
            if(self.visualization):
                image = vis_2d_bbox(image, xywh, box)
        
        # Draw projected LiDAR points on the image.
        for pt in all_extracted_pts:
            cv2.circle(image, pt, 2, (0, 0, 255), -1)
        
        # Used for visualization
        if(self.visualization):
            ros_img = self.bridge.cv2_to_imgmsg(image, 'bgr8')
            self.pub_image.publish(ros_img)  



if __name__ == '__main__':
    rospy.init_node('fusion_node', anonymous=True)
    Fusion3D()
    while not rospy.core.is_shutdown():
        rospy.rostime.wallsleep(0.5)

