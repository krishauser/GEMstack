import cv2
import numpy as np
import os
import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import PointField, PointCloud2, Image
from cv_bridge import CvBridge
import sensor_msgs.point_cloud2 as pc2
import time

# from DistortionRemove import *


# canvas shape
STITCH_WIDTH = 1200
STITCH_HEIGHT = 1800

PIXEL_TO_METER = 0.02
METER_TO_PIXEL = 1 / PIXEL_TO_METER


# srcFrontCamListXY = np.array([
#     [446, 504],  # FL
#     [712, 504],  # FR
#     [310, 700],  # RL
#     [895, 700],  # RR
# ])  # origin

# dstFrontCamListXY = np.array([
#     [-1.2834, -8.526 ],  # [-1.3/2-0.1-0.5334  , 0.151-1.46-2.794-4.423],  # FL
#     [ 1.766 , -8.3194],  # [1.3/2+0.1+1.016    , 0.151-1.46-2.7432-4.2672],  # FR
#     [-1.2834, -4.103 ],  # [-1.3/2-0.1-0.5334  , 0.151-1.46-2.794],  # RL
#     [ 1.766 , -4.0522],  # [1.3/2+0.1+1.016    , 0.151-1.46-2.7432],  # RR
# ])  # origin


# dstFrontCamListXY = np.array([
#     [-1.4834, -8.526 ],  # [-1.3/2-0.1-0.5334  , 0.151-1.46-2.794-4.423],  # FL
#     [ 1.466 , -8.3194],  # [1.3/2+0.1+1.016    , 0.151-1.46-2.7432-4.2672],  # FR
#     [-1.4834, -4.103 ],  # [-1.3/2-0.1-0.5334  , 0.151-1.46-2.794],  # RL
#     [ 1.466 , -4.0522],  # [1.3/2+0.1+1.016    , 0.151-1.46-2.7432],  # RR
# ])  # tune by hand




def cvtFootInch2Meter(ft, inch=0.0):
    return (12 * ft + inch) * 0.0254


pixelFrontCamListXY = np.array([
    [466,466],  # FL
    [668,462],  # FR
    [380,590],  # RL
    [775,585],  # RR
])


pixelFrontLeftCamListXY = np.array([
    [220,700],  # FL
    [1765,585],  # FR
    [420,950],  # RL
    [1640,775],  # RR
])


pixelFrontRightCamListXY = np.array([
    [87,670],  # FL
    [1625,777],  # FR
    [260,855],  # RL
    [1460,966],  # RR
])


"""
backup matrix

worldFrontCamListXY = np.array([
    [cvtFootInch2Meter(0, -25-30    -18), cvtFootInch2Meter(0, -210-170    +10)-1.46+0.151],  # FL
    [cvtFootInch2Meter(0,  35+30    -20), cvtFootInch2Meter(0, -210-170     +6)-1.46+0.151],  # FR
    [cvtFootInch2Meter(0, -25-30    -5), cvtFootInch2Meter(0, -170          +5)-1.46+0.151],  # RL
    [cvtFootInch2Meter(0,  35+30    -8), cvtFootInch2Meter(0, -170          +2)-1.46+0.151],  # RR
])

worldFrontLeftCamListXY = np.array([
    [cvtFootInch2Meter(0, -280-30), cvtFootInch2Meter(0, 0   +8)-1.46+0.151],  # FL
    [cvtFootInch2Meter(0,  -25-30   -20), cvtFootInch2Meter(0, -210-170     +10)-1.46+0.151],  # FR
    [cvtFootInch2Meter(0, -130-30), cvtFootInch2Meter(0, 0   +8)-1.46+0.151],  # RL
    [cvtFootInch2Meter(0,  -25-30   -5),cvtFootInch2Meter(0, -170      +5)-1.46+0.151],  # RR
])


worldFrontRightCamListXY = np.array([
    [cvtFootInch2Meter(0,  35+30    -20), cvtFootInch2Meter(0, -210-170)-1.46+0.151],  # FL
    [cvtFootInch2Meter(0, 250+30), cvtFootInch2Meter(0, 0   -10)-1.46+0.151],  # FR
    [cvtFootInch2Meter(0,  35+30    -10),cvtFootInch2Meter(0, -170)-1.46+0.151],  # RL
    [cvtFootInch2Meter(0, 140+30), cvtFootInch2Meter(0, 0   -5)-1.46+0.151],  # RR
])


"""


worldFrontCamListXY = np.array([
    [cvtFootInch2Meter(0, -25-30    -18), cvtFootInch2Meter(0, -210-170    +10)-1.46+0.151],  # FL
    [cvtFootInch2Meter(0,  35+30    -20), cvtFootInch2Meter(0, -210-170     +6)-1.46+0.151],  # FR
    [cvtFootInch2Meter(0, -25-30    -5), cvtFootInch2Meter(0, -170          +5)-1.46+0.151],  # RL
    [cvtFootInch2Meter(0,  35+30    -8), cvtFootInch2Meter(0, -170          +2)-1.46+0.151],  # RR
])

worldFrontLeftCamListXY = np.array([
    [cvtFootInch2Meter(0, -280-30), cvtFootInch2Meter(0, 0   +8)-1.46+0.151],  # FL
    [cvtFootInch2Meter(0,  -25-30   -20), cvtFootInch2Meter(0, -210-170     +10)-1.46+0.151],  # FR
    [cvtFootInch2Meter(0, -130-30), cvtFootInch2Meter(0, 0   +8)-1.46+0.151],  # RL
    [cvtFootInch2Meter(0,  -25-30   -5),cvtFootInch2Meter(0, -170      +5)-1.46+0.151],  # RR
])


worldFrontRightCamListXY = np.array([
    [cvtFootInch2Meter(0,  35+30    -20), cvtFootInch2Meter(0, -210-170)-1.46+0.151],  # FL
    [cvtFootInch2Meter(0, 250+30), cvtFootInch2Meter(0, 0   -10)-1.46+0.151],  # FR
    [cvtFootInch2Meter(0,  35+30    -10),cvtFootInch2Meter(0, -170)-1.46+0.151],  # RL
    [cvtFootInch2Meter(0, 140+30), cvtFootInch2Meter(0, 0   -5)-1.46+0.151],  # RR
])


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





def maskCamKeyArea(img, whichCam):
    """
    whichCam : FRONT / FRONT_LEFT / FRONT_RIGHT
    """
    if whichCam == "FRONT":
        mask = np.zeros(img.shape[:2], dtype=np.uint8)
        mask[430:, :] = 255
        maskedImg = cv2.bitwise_and(img, img, mask=mask)
    elif whichCam == "FRONT_LEFT":
        mask = np.zeros(img.shape[:2], dtype=np.uint8)
        mask[450:, :] = 255
        maskedImg = cv2.bitwise_and(img, img, mask=mask)
    elif whichCam == "FRONT_RIGHT":
        mask = np.zeros(img.shape[:2], dtype=np.uint8)
        mask[450:, :] = 255
        maskedImg = cv2.bitwise_and(img, img, mask=mask)
    else:
        mask = np.ones(img.shape[:2], dtype=np.uint8)*255
        maskedImg = cv2.bitwise_and(img, img, mask=mask)
    return maskedImg


def maskTransformedKeyArea(img, whichCam):
    """
    whichCam : FRONT / FRONT_LEFT / FRONT_RIGHT
    """
    h, w = img.shape[:2]
    maskTop = np.zeros(img.shape[:2], dtype=np.uint8)
    maskTop[:h//2, :] = 255
    maskDown = np.zeros(img.shape[:2], dtype=np.uint8)
    maskDown[h//2:, :] = 255
    maskLeft = np.zeros(img.shape[:2], dtype=np.uint8)
    maskLeft[:, :w//2] = 255
    maskRight = np.zeros(img.shape[:2], dtype=np.uint8)
    maskRight[:, w//2:] = 255
    
    if whichCam == "FRONT":
        maskedImg = cv2.bitwise_and(img, img, mask=maskTop)
    elif whichCam == "FRONT_LEFT":
        maskedImg = cv2.bitwise_and(img, img, mask=maskTop)
        maskedImg = cv2.bitwise_and(maskedImg, maskedImg, mask=maskLeft)
    elif whichCam == "FRONT_RIGHT":
        maskedImg = cv2.bitwise_and(img, img, mask=maskTop)
        maskedImg = cv2.bitwise_and(maskedImg, maskedImg, mask=maskRight)
    else:
        mask = np.ones(img.shape[:2], dtype=np.uint8)*255
        maskedImg = cv2.bitwise_and(img, img, mask=mask)
    return maskedImg




class PerspectiveTransform():
    def __init__(self,  srcPixelListXY:np.ndarray, 
                        dstMeterListXY:np.ndarray):
        # self.imgShape = (self.img).shape
        self.srcPixelListXY = srcPixelListXY  # img pixel
        self.dstMeterListXY = dstMeterListXY  # real world
        
        self.dstPixelListXY = (self.dstMeterListXY * METER_TO_PIXEL)  # 本来 1像素 = 1m，现在 1像素 = 2厘米
        self.dstPixelListXY[:, 0] += STITCH_WIDTH//2
        self.dstPixelListXY[:, 1] += STITCH_HEIGHT//2
        
        # print(self.srcPixelListXY)
        # print(self.dstPixelListXY)
        
        self.perspectiveTransMat = cv2.getPerspectiveTransform( np.float32(self.srcPixelListXY), 
                                                                np.float32(self.dstPixelListXY))
        # print(self.perspectiveTransMat)
        # exit()
        
        # cv2.namedWindow("transformedImg", cv2.WINDOW_NORMAL)  # can change window size
        # cv2.resizeWindow("transformedImg", STITCH_WIDTH//2, STITCH_HEIGHT//2)  # set window size
        # cv2.imshow("transformedImg", self.transformedImg)
        # cv2.waitKey(0)
        # cv2.destroyAllWindows()
    
    def getTransformedImg(self, img):
        transformedImg = cv2.warpPerspective(img, self.perspectiveTransMat, (STITCH_WIDTH, STITCH_HEIGHT))
        return transformedImg

# pT = PerspectiveTransform(
#     srcPixelListXY=srcFrontCamListXY,
#     dstMeterListXY=dstFrontCamListXY)


def combineImages(imgs):
    h, w, c = imgs[0].shape
    sumImg = np.zeros((h, w, c), dtype=np.float32)
    count = np.zeros((h, w), dtype=np.float32)

    for img in imgs:
        mask = np.any(img != [0, 0, 0], axis=2)
        sumImg[mask] += img[mask].astype(np.float32)
        count[mask] += 1

    count[count == 0] = 1  # avoid division by zero

    avgImg = (sumImg / count[:, :, np.newaxis]).astype(np.uint8)
    
    return avgImg



def cvtImg2PointCloudMsg(imgSrc, frameId="base_footprint", pixelSize=PIXEL_TO_METER, stride=1, stamp=None):
    img = imgSrc[::stride, ::stride]  # down sample
    height, width, channel = img.shape
    totalPoints = height * width
    
    u, v = np.meshgrid(np.arange(width), np.arange(height))
    xImg = (u.astype(np.float32).reshape(-1) - width // 2) * pixelSize * stride
    yImg = (v.astype(np.float32).reshape(-1) - height // 2) * pixelSize * stride
    zImg = np.zeros_like(xImg)  # at the ground
    
    # x = u.astype(np.float32).reshape(-1) * pixelSize * stride
    # y = v.astype(np.float32).reshape(-1) * pixelSize * stride
    # z = np.zeros_like(x)  # at the ground
    
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



class ImageSubscriberAndFunctions():
    def __init__(self, topicName, maskName, srcPixelListXY, dstMeterListXY, distortCoeff=None, intrinsicMat=None):
        self.topicName = topicName
        self.image = None
        self.maskedImage = None
        self.transformedImage = np.zeros((STITCH_HEIGHT, STITCH_WIDTH, 3), dtype=np.uint8)
        self.maskName = maskName
        self.srcPixelListXY = srcPixelListXY
        self.dstMeterListXY = dstMeterListXY
        self.distortionCoeffs = distortCoeff
        self.intrinsicMat = intrinsicMat
        self.bridge = CvBridge()
        self.perspectiveTransform = PerspectiveTransform(srcPixelListXY, dstMeterListXY)
        self.imageSub = rospy.Subscriber(self.topicName, Image, self.callback, queue_size=1)
        
    def callback(self, msg):
        self.image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        if self.distortionCoeffs is not None and self.intrinsicMat is not None:
            self.image = undistortImage(self.image, self.intrinsicMat, self.distortionCoeffs)[0]
        self.maskedImage = maskCamKeyArea(self.image, self.maskName)
        self.transformedImage = self.perspectiveTransform.getTransformedImg(self.maskedImage)

def cvtBirdEyeViewPixel2MeterInVehicleFrame(pixelPosXY, pixelSize=PIXEL_TO_METER):
    """
    From:
    ·---→x
    |
    |
    ↓y
    
    To:
     z   x
      \  ↑
       \ |
        \|
    y←---·
    """
    pixelX, pixelY = pixelPosXY[0] - STITCH_WIDTH/2, pixelPosXY[1] - STITCH_HEIGHT/2
    meterX, meterY = -pixelY * pixelSize + 0.151 + 1.10, -pixelX * pixelSize
    return np.array([meterX, meterY])


def cvtOriginImgPixel2DToVehicleFrameMeter3D(TransMat, pixelPointXY):
    """
    pixelPointXY: (u, v) in original image (pixel)
    TransMat: 3x3 perspective transform matrix (pixel -> real world meter)
    """
    BASE_VEHICLE_DIST = 1.10  # meter
    
    point = np.array([pixelPointXY[0], pixelPointXY[1], 1.0], dtype=np.float32).reshape(3, 1)
    
    # 透视变换
    transformed = TransMat @ point
    transformed /= transformed[2, 0]  # 归一化

    x_real = transformed[0, 0]  # meter
    y_real = transformed[1, 0]  # meter
    z_real = 0.0  # 固定在地面

    # 转 pointcloud 坐标系（和你之前cvtImg2PointCloudMsg一致）
    x_pc = -y_real
    y_pc = -x_real
    z_pc = z_real

    # return np.array([x_pc + BASE_VEHICLE_DIST, y_pc, z_pc], dtype=np.float32)
    return np.array([x_pc + BASE_VEHICLE_DIST, y_pc], dtype=np.float32)


def cvtOriginImgPixels2DToVehicleFrameMeter2D(TransMat, pixelPointXYs):
    """
    pixelPointXYs: N x 2 array of (u, v) in original image
    TransMat: 3x3 perspective transform matrix (pixel -> real world meter)
    return: N x 2 array of (x_vehicle, y_vehicle)
    """
    BASE_VEHICLE_DIST = 1.10  # meter

    # Add homogeneous coordinate: [u, v, 1]
    N = pixelPointXYs.shape[0]
    homogeneous_points = np.hstack([pixelPointXYs, np.ones((N, 1), dtype=np.float32)])  # (N, 3)

    # Apply perspective transform
    transformed = (TransMat @ homogeneous_points.T).T  # (N, 3)

    # Normalize (divide by last coordinate)
    transformed /= transformed[:, [2]]

    x_real = transformed[:, 0]
    y_real = transformed[:, 1]
    z_real = np.zeros_like(x_real)

    # Convert to vehicle pointcloud coordinate
    x_pc = -y_real
    y_pc = -x_real
    z_pc = z_real

    # Return: x + front base distance offset
    return np.stack([x_pc + BASE_VEHICLE_DIST, y_pc], axis=1)


if __name__ == "__main__":
    pixel_points = np.array([
        [48, 672],
        [1691, 791],
        [205, 882],
        [1516, 1003],
    ], dtype=np.float32)

    TransMat = cv2.getPerspectiveTransform(
        np.float32(pixelFrontRightCamListXY),
        np.float32(worldFrontRightCamListXY)
    )

    results = cvtOriginImgPixels2DToVehicleFrameMeter2D(TransMat, pixel_points)
    print(results)
    


if __name__ == "__main__":
    
    # img = cv2.imread(os.path.join(os.path.dirname(__file__), "Pics", "front_right.png"), cv2.IMREAD_COLOR)
    # cv2.imshow("img", img)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()
    # exit(0)
    
    TransMat = cv2.getPerspectiveTransform( np.float32(pixelFrontRightCamListXY), 
                                            np.float32(worldFrontRightCamListXY))
    
    result = cvtOriginImgPixel2DToVehicleFrameMeter3D(TransMat, (48, 672))
    print(result)
    result = cvtOriginImgPixel2DToVehicleFrameMeter3D(TransMat, (1691, 791))
    print(result)
    result = cvtOriginImgPixel2DToVehicleFrameMeter3D(TransMat, (205, 882))
    print(result)
    result = cvtOriginImgPixel2DToVehicleFrameMeter3D(TransMat, (1516, 1003))
    print(result)
    exit(-1)


"""
48, 672

1691, 791

205, 882

1516, 1003
"""



# if __name__ != "__main__":  # test cvt frames
#     print(cvtBirdEyeViewPixel2MeterInVehicleFrame([0,0], pixelSize=PIXEL_TO_METER))
#     print(cvtBirdEyeViewPixel2MeterInVehicleFrame([1200,0], pixelSize=PIXEL_TO_METER))
#     print(cvtBirdEyeViewPixel2MeterInVehicleFrame([0,1800], pixelSize=PIXEL_TO_METER))
#     print(cvtBirdEyeViewPixel2MeterInVehicleFrame([1200,1800], pixelSize=PIXEL_TO_METER))
    
#     exit
    



if __name__ == "__main__":  # sub and pub
    """
    pixelListXY : np.ndarray
        shape = (4, 2)
    """
    frontCamImageSub = ImageSubscriberAndFunctions(
        "/oak/rgb/image_raw", "FRONT", 
        # srcFrontCamListXY, dstFrontCamListXY)
        pixelFrontCamListXY, worldFrontCamListXY,)
    frontLeftCamImageSub = ImageSubscriberAndFunctions(
        "/camera_fl/arena_camera_node/image_raw", "FRONT_LEFT", 
        pixelFrontLeftCamListXY, worldFrontLeftCamListXY,
        DISTORT_FRONT_LEFT, K_FRONT_LEFT)
    frontRightCamImageSub = ImageSubscriberAndFunctions(
        "/camera_fr/arena_camera_node/image_raw", "FRONT_RIGHT", 
        pixelFrontRightCamListXY, worldFrontRightCamListXY,
        DISTORT_FRONT_RIGHT, K_FRONT_RIGHT)

    rospy.init_node("image_to_pointcloud_node")
    pc_pub = rospy.Publisher("/image_pointcloud", PointCloud2, queue_size=1)
    rate = rospy.Rate(1)  # times per second
    
    
    # save_dir = os.path.join(os.path.dirname(__file__), "StitchImages/multiple_slanted_all_valid")
    # if os.path.exists(save_dir) is False:
    #     os.makedirs(save_dir)
    
    while not rospy.is_shutdown():

        stitched_img = combineImages([
            frontCamImageSub.transformedImage,
            frontLeftCamImageSub.transformedImage,
            frontRightCamImageSub.transformedImage
        ])
        
        # now = rospy.Time.now()
        # t_sec = now.to_sec()
        # t_nsec = now.to_nsec()
        
        # t_str = time.strftime("%Y%m%d_%H%M%S", time.localtime(t_sec))
        # save_filename = f"{t_str}_{t_nsec}.png"
        # save_path = os.path.join(save_dir, save_filename)
        # cv2.imwrite(save_path, stitched_img)

        pc_msg = cvtImg2PointCloudMsg(stitched_img, frameId="base_footprint", pixelSize=PIXEL_TO_METER, stride=2)
        pc_pub.publish(pc_msg)

        rate.sleep()  # keep the process rate



# if __name__ == "__main__":  # read fixed img and pub
    
#     FRONT_CAM_IMG_PATH = os.path.join(os.path.dirname(__file__), "Pics", "front.png")
#     imgFrontCam = cv2.imread(FRONT_CAM_IMG_PATH)
#     # imgFrontCam = (imgFrontCam.astype(np.float32)*1.5).astype(np.uint8)
#     maskedImgFrontCam = maskCamKeyArea(imgFrontCam, "Front")
    
    
#     FL_CAM_IMG_PATH = os.path.join(os.path.dirname(__file__), "Pics", "fl.png")
#     imgFrontLeftCam = cv2.imread(FL_CAM_IMG_PATH)
#     imgFrontLeftCam = undistortImage(imgFrontLeftCam, K_FRONT_LEFT, DISTORT_FRONT_LEFT)[0]
#     maskedImgFrontLeftCam = maskCamKeyArea(imgFrontLeftCam, "FRONT_LEFT")
    
    
#     FR_CAM_IMG_PATH = os.path.join(os.path.dirname(__file__), "Pics", "fr.png")
#     imgFrontRightCam = cv2.imread(FR_CAM_IMG_PATH)
#     imgFrontRightCam = undistortImage(imgFrontRightCam, K_FRONT_RIGHT, DISTORT_FRONT_RIGHT)[0]
#     maskedImgFrontRightCam = maskCamKeyArea(imgFrontRightCam, "FRONT_RIGHT")
    
#     # cv2.imshow("maskedImgFrontCam", maskedImgFrontCam)
#     # cv2.imshow("maskedImgFrontLeftCam", maskedImgFrontLeftCam)
#     # cv2.imshow("maskedImgFrontRightCam", maskedImgFrontRightCam)
#     # cv2.waitKey(0)
#     # cv2.destroyAllWindows()


#     FrontCamPerspectiveTransform = PerspectiveTransform(
#         # img=maskedImgFrontCam, 
#         srcPixelListXY=pixelFrontCamListXY, 
#         dstMeterListXY=worldFrontCamListXY, 
#     )
#     transformedFrontImg = FrontCamPerspectiveTransform.getTransformedImg(maskedImgFrontCam)
#     transformedFrontImg = maskTransformedKeyArea(transformedFrontImg, "FRONT")


#     FrontLeftCamPerspectiveTransform = PerspectiveTransform(
#         # img=imgFrontLeftCam, 
#         srcPixelListXY=pixelFrontLeftCamListXY, 
#         dstMeterListXY=worldFrontLeftCamListXY, 
#     )
#     transformedFrontLeftImg = FrontLeftCamPerspectiveTransform.getTransformedImg(imgFrontLeftCam)
#     transformedFrontLeftImg = maskTransformedKeyArea(transformedFrontLeftImg, "FRONT_LEFT")
    

#     FrontRightCamPerspectiveTransform = PerspectiveTransform(
#         # img=imgFrontRightCam, 
#         srcPixelListXY=pixelFrontRightCamListXY, 
#         dstMeterListXY=worldFrontRightCamListXY, 
#     )
#     transformedFrontRightImg = FrontRightCamPerspectiveTransform.getTransformedImg(imgFrontRightCam)
#     transformedFrontRightImg = maskTransformedKeyArea(transformedFrontRightImg, "FRONT_RIGHT")
    

#     cv2.imshow("transformedFrontImg", transformedFrontImg)
#     cv2.imshow("transformedFrontLeftImg", transformedFrontLeftImg)
#     cv2.imshow("transformedFrontRightImg", transformedFrontRightImg)
    
    
#     stitchImg = combineImages([transformedFrontImg, transformedFrontLeftImg, transformedFrontRightImg])
#     cv2.imshow("stitchImg", stitchImg)
    
    
    
#     cv2.waitKey(0)
#     cv2.destroyAllWindows()

#     rospy.init_node("image_to_pointcloud_node")
#     pub = rospy.Publisher("/image_pointcloud", PointCloud2, queue_size=1)
#     rate = rospy.Rate(1)
#     while not rospy.is_shutdown():
#         msg = cvtImg2PointCloudMsg(stitchImg, frameId="base_footprint", pixelSize=0.02, stride=2)
#         pub.publish(msg)
#         rate.sleep()



# if __name__ == "__main__":

#     FRONT_CAM_IMG_PATH = os.path.join(os.path.dirname(__file__), "StitchImages", "FrontCam.png")
#     imgFrontCam = cv2.imread(FRONT_CAM_IMG_PATH)
#     # print(imgFrontCam.shape)  # (720, 1152, 3)

    
#     maskedImgFrontCam = maskCamKeyArea(imgFrontCam, "FRONT")
    
#     cv2.imshow("imgFrontCam", imgFrontCam)
#     cv2.imshow("maskedImgFrontCam", maskedImgFrontCam)
#     cv2.waitKey(0)
#     cv2.destroyAllWindows()
    


#     print(type(srcFrontCamListXY))

    
#     frontCamPerspectiveTransform = PerspectiveTransform(
#         img=maskedImgFrontCam, 
#         srcPixelListXY=srcFrontCamListXY, 
#         dstMeterListXY=dstFrontCamListXY, 
#     )
    
#     transformedImg = frontCamPerspectiveTransform.getTransformedImg()
    
#     rospy.init_node("image_to_pointcloud_node")
#     pub = rospy.Publisher("/image_pointcloud", PointCloud2, queue_size=1)
#     rate = rospy.Rate(1)
#     while not rospy.is_shutdown():
#         msg = cvtImg2PointCloudMsg(transformedImg, frameId="base_footprint", pixelSize=0.02)
#         pub.publish(msg)
#         rate.sleep()










