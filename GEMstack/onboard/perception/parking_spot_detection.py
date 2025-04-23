import rospy
from ultralytics import YOLO
from ..component import Component
from ...state import VehicleState
from sensor_msgs.msg import Image
from .bev import *

class ParkingSpotDetector3D(Component):
    def __init__(self):
        # Initial variables
        self.bridge = CvBridge()
        self.model_path = os.getcwd() + '/GEMstack/knowledge/detection/yolov8n_fold_0.pt'
        self.model = YOLO(self.model_path)
        self.model.to('cuda')
        # self.model = torch.hub.load('ultralytics/yolov8n', 'custom', path=self.model_path) 
        # self.model.eval()

        # Subscribers
        self.sub_right_cam = rospy.Subscriber("/camera_fr/arena_camera_node/image_raw", Image, self.callback, queue_size=1)

        # Publishers
        self.pub_bev_fr = rospy.Publisher("/bev/front_right", Image, queue_size=1)
        self.pub_detection_bev_fr = rospy.Publisher("/detection/annotated_bev/front_right", Image, queue_size=1)


    # Main sensors callback
    def callback(self, right_cam_msg):
        # Process camera image message
        processed_image = self.process_image_msg(right_cam_msg, 
                                                "FRONT_RIGHT", 
                                                DISTORT_FRONT_RIGHT, 
                                                K_FRONT_RIGHT)
        
        # Apply bird eye transform
        right_cam_bev = self.transform_to_bev(processed_image,
                                                pixelFrontRightCamListXY, 
                                                worldFrontRightCamListXY)
        
        # Detection
        results = self.model(right_cam_bev)[0]
        print(f"results: {results.boxes}")
        right_cam_bev_annotated = right_cam_bev.copy()
        # Draw bounding boxes
        for det in results:
            x1, y1, x2, y2, _, cls = map(int, det[:6])  # Extract values
            label = f'{self.model.names[int(cls)]}'  # Class name + confidence

            # Draw rectangle
            cv2.rectangle(right_cam_bev_annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)  # Green box
            cv2.putText(right_cam_bev_annotated, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

          # Save or display the image
        right_cam_bev_annotated_ros_img = self.bridge.cv2_to_imgmsg(right_cam_bev_annotated, 'bgr8')
        self.pub_detection_bev_fr.publish(right_cam_bev_annotated_ros_img)
 
        
        # Publish BEV
        right_cam_bev_ros_img = self.bridge.cv2_to_imgmsg(right_cam_bev, 'bgr8')
        self.pub_bev_fr.publish( right_cam_bev_ros_img)
    

    # All local helper functions
    def process_image_msg(self, image_msg, maskName, distortionCoeffs=None, intrinsicMat=None):
        # Convert image to cv2
        cv_image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')

        # Remove distortion
        if distortionCoeffs is not None and intrinsicMat is not None:
            cv_image = undistortImage(cv_image, intrinsicMat, distortionCoeffs)[0]

        # Crop image
        maskedImage = maskCamKeyArea(cv_image, maskName)
        return maskedImage

    
    def transform_to_bev(self, image, srcPixelListXY, dstMeterListXY):
        perspectiveTransform = PerspectiveTransform(srcPixelListXY, dstMeterListXY)
        transformedImage = perspectiveTransform.getTransformedImg(image)
        return transformedImage


    def spin(self):
        rospy.spin()

    def rate(self) -> float:
        return 10.0  # Hz

    def state_inputs(self) -> list:
        return ['vehicle']

    def state_outputs(self) -> list:
        return ['agents']

    def update(self, vehicle: VehicleState) -> dict:
        # return dictionary with one key: 'agents'
        return {'agents': {}}  # or real AgentState dict


if __name__ == "__main__":
    node = ParkingSpotDetector3D()
    node.spin()