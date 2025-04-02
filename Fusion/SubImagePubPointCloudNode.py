import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import Header
import sensor_msgs.point_cloud2 as pc2


from stitch import  cvtImg2PointCloudMsg, maskCamKeyArea, \
                    PerspectiveTransform, \
                    srcFrontCamListXY, dstFrontCamListXY


class SubImagePubPointCloudNode:
    def __init__(self):
        rospy.init_node("image_to_pointcloud_node")

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/oak/rgb/image_raw", Image, self.image_callback, queue_size=1)
        self.pc_pub = rospy.Publisher("/image_pointcloud", PointCloud2, queue_size=1)

        self.latest_msg = None

        # 相机参数
        self.src_pts = srcFrontCamListXY
        self.dst_pts = dstFrontCamListXY

        rospy.loginfo("Image to PointCloud2 node started.")

    def image_callback(self, msg):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            rospy.logwarn(f"CV Bridge error: {e}")
            return

        masked_img = maskCamKeyArea(img, "FRONT")
        transformer = PerspectiveTransform(masked_img, self.src_pts.copy(), self.dst_pts.copy())
        transformed_img = transformer.getTransformedImg()

        pc_msg = cvtImg2PointCloudMsg(
            transformed_img, 
            frameId="base_link", 
            pixelSize=0.02, 
            stride=6,
            stamp = msg.header.stamp
        )
        self.pc_pub.publish(pc_msg)



if __name__ == "__main__":
    try:
        node = SubImagePubPointCloudNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


# TODO : try down-sample
