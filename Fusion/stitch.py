import cv2
import numpy as np
import os
import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import PointField, PointCloud2
import sensor_msgs.point_cloud2 as pc2


# canvas shape
STITCH_WIDTH = 1000
STITCH_HEIGHT = 1000



srcFrontCamListXY = np.array([
    [446, 504],  # FL
    [712, 504],  # FR
    [310, 700],  # RL
    [895, 700],  # RR
])  # origin

# dstFrontCamListXY = np.array([
#     [-1.2834, -8.526 ],  # [-1.3/2-0.1-0.5334  , 0.151-1.46-2.794-4.423],  # FL
#     [ 1.766 , -8.3194],  # [1.3/2+0.1+1.016    , 0.151-1.46-2.7432-4.2672],  # FR
#     [-1.2834, -4.103 ],  # [-1.3/2-0.1-0.5334  , 0.151-1.46-2.794],  # RL
#     [ 1.766 , -4.0522],  # [1.3/2+0.1+1.016    , 0.151-1.46-2.7432],  # RR
# ])  # origin


dstFrontCamListXY = np.array([
    [-1.4834, -8.526 ],  # [-1.3/2-0.1-0.5334  , 0.151-1.46-2.794-4.423],  # FL
    [ 1.466 , -8.3194],  # [1.3/2+0.1+1.016    , 0.151-1.46-2.7432-4.2672],  # FR
    [-1.4834, -4.103 ],  # [-1.3/2-0.1-0.5334  , 0.151-1.46-2.794],  # RL
    [ 1.466 , -4.0522],  # [1.3/2+0.1+1.016    , 0.151-1.46-2.7432],  # RR
])  # tune by hand





def maskCamKeyArea(img, whichCam):
    if whichCam == "FRONT":
        mask = np.zeros(img.shape[:2], dtype=np.uint8)
        mask[430:, :] = 255
        maskedImg = cv2.bitwise_and(img, img, mask=mask)

    return maskedImg



class PerspectiveTransform():
    def __init__(self,  img:np.ndarray, 
                        srcPixelListXY:np.ndarray, 
                        dstMeterListXY:np.ndarray):
        self.img = img
        self.imgShape = (self.img).shape
        self.srcPixelListXY = srcPixelListXY  # img pixel
        self.dstMeterListXY = dstMeterListXY  # real world
        
        self.dstPixelListXY = (self.dstMeterListXY * 50)  # 本来 1像素 = 1m，现在 1像素 = 2厘米
        self.dstPixelListXY[:, 0] += STITCH_WIDTH//2
        self.dstPixelListXY[:, 1] += STITCH_HEIGHT//2
        
        # print(self.srcPixelListXY)
        # print(self.dstPixelListXY)
        
        self.perspectiveTransMat = cv2.getPerspectiveTransform(np.float32(self.srcPixelListXY), 
                                                               np.float32(self.dstPixelListXY))
        self.transformedImg = cv2.warpPerspective(self.img, self.perspectiveTransMat, (STITCH_WIDTH, STITCH_HEIGHT))
        
        # cv2.namedWindow("transformedImg", cv2.WINDOW_NORMAL)  # 允许手动调整窗口
        # cv2.resizeWindow("transformedImg", STITCH_WIDTH//2, STITCH_HEIGHT//2)  # 设置窗口大小为 
        # cv2.imshow("transformedImg", self.transformedImg)
        # cv2.waitKey(0)
        # cv2.destroyAllWindows()
    
    def getTransformedImg(self):
        return self.transformedImg


def feet2meter(ft, inch=0.0):
    return (12 * ft + inch) * 0.0254



def cvtImg2PointCloudMsg(imgSrc, frameId="base_footprint", pixelSize=0.02, stride=1, stamp=None):
    img = imgSrc[::stride, ::stride]  # 图像下采样
    height, width, channel = img.shape
    totalPoints = height * width
    
    u, v = np.meshgrid(np.arange(width), np.arange(height))
    xImg = (u.astype(np.float32).reshape(-1) - width // 2) * pixelSize * stride
    yImg = (v.astype(np.float32).reshape(-1) - height // 2) * pixelSize * stride
    zImg = np.zeros_like(xImg)  # 都在地面 高度=0
    
    # x = u.astype(np.float32).reshape(-1) * pixelSize * stride
    # y = v.astype(np.float32).reshape(-1) * pixelSize * stride
    # z = np.zeros_like(x)  # 都在地面 高度=0
    
    x = -yImg
    y = -xImg
    z = zImg
    
    bgr = img.reshape(-1, 3)
    r = bgr[:, 2].astype(np.uint32)
    g = bgr[:, 1].astype(np.uint32)
    b = bgr[:, 0].astype(np.uint32)
    
    rgb = (r << 16) | (g << 8) | b
    rgb_float = rgb.view(np.float32)
    
    points = np.stack([x, y, z, rgb_float], axis=-1)
    
    fields = [
        PointField("x", 0, PointField.FLOAT32, 1),
        PointField("y", 4, PointField.FLOAT32, 1),
        PointField("z", 8, PointField.FLOAT32, 1),
        PointField("rgb", 12, PointField.FLOAT32, 1),
    ]
    
    header = Header()
    if stamp is not None:
        header.stamp = stamp
    else:
        header.stamp = rospy.Time.now()
    header.frame_id = frameId
    
    pc2_msg = pc2.create_cloud(header, fields, points)
    return pc2_msg








if __name__ == "__main__":

    FRONT_CAM_IMG_PATH = os.path.join(os.path.dirname(__file__), "StitchImages", "FrontCam.png")
    imgFrontCam = cv2.imread(FRONT_CAM_IMG_PATH)
    # print(imgFrontCam.shape)  # (720, 1152, 3)

    
    maskedImgFrontCam = maskCamKeyArea(imgFrontCam, "FRONT")
    
    cv2.imshow("imgFrontCam", imgFrontCam)
    cv2.imshow("maskedImgFrontCam", maskedImgFrontCam)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    

    
    print(type(srcFrontCamListXY))
    


    
    frontCamPerspectiveTransform = PerspectiveTransform(
        img=maskedImgFrontCam, 
        srcPixelListXY=srcFrontCamListXY, 
        dstMeterListXY=dstFrontCamListXY, 
    )
    
    transformedImg = frontCamPerspectiveTransform.getTransformedImg()
    
    rospy.init_node("image_to_pointcloud_node")
    pub = rospy.Publisher("/image_pointcloud", PointCloud2, queue_size=1)
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        msg = cvtImg2PointCloudMsg(transformedImg, frameId="base_footprint", pixelSize=0.02)
        pub.publish(msg)
        rate.sleep()










    # src = np.array([
    #     [,],  # FL
    #     [,],  # FR
    #     [,],  # RL
    #     [,],  # RR
    # ])
    
    # dst = np.array([
    #     [,],  # FL
    #     [,],  # FR
    #     [,],  # RL
    #     [,],  # RR
    # ])