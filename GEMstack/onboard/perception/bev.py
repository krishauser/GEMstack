import cv2
import numpy as np
import os
import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import PointField, PointCloud2, Image
from cv_bridge import CvBridge
import sensor_msgs.point_cloud2 as pc2


# canvas shape
STITCH_WIDTH = 1200
STITCH_HEIGHT = 1800

PIXEL_TO_METER = 0.02
METER_TO_PIXEL = 1 / PIXEL_TO_METER


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


# front left (distortion & intrinsic)
DISTORT_FRONT_LEFT = np.array([-0.23751890570984993, 0.08452214195986749, -0.00035324203850054794, -0.0003762498910536819, 0.0])
K_FRONT_LEFT = np.array([1230.1440959825889, 0.0, 978.8285075011214, 0.0, 1230.6304235898162, 605.7940335714893, 0.0, 0.0, 1.0]).reshape((3,3))

# front right (distortion & intrinsic)
DISTORT_FRONT_RIGHT = np.array([-0.2448506795091457, 0.08202383880344215, 0.0004294271518916802, -0.0012454354245869965, 0.0])
K_FRONT_RIGHT = np.array([1180.753803927752, 0.0, 934.8594468810093, 0.0, 1177.034929104844, 607.2669740236552, 0.0, 0.0, 1.0]).reshape((3,3))

# rear left (distortion & intrinsic)
DISTORT_REAR_LEFT = np.array([-0.24343640079788503, 0.09125282937288573, 5.21454371097206e-06, -2.2750254391060847e-05, 0.0])
K_REAR_LEFT = np.array([1216.836137433559, 0.0, 955.7290895142494, 0.0, 1216.0457208793944, 599.3150429290699, 0.0, 0.0, 1.0]).reshape((3,3))

# rear right (distortion & intrinsic)
DISTORT_REAR_RIGHT = np.array([-0.24754745389508612, 0.09944435338953724, -3.455420573094537e-05, -0.00022029168609146265, 0.0])
K_REAR_RIGHT = np.array([1210.2941870955246, 0.0, 951.3029561113208, 0.0, 1211.035575904068, 600.5637386855253, 0.0, 0.0, 1.0]).reshape((3,3))


def undistortImage(originImg, cameraMatrix, distCoeffs):
    h,  w = originImg.shape[:2]  # height & width
    # new intrinsic parameter
    newCameraMatrix, roi = cv2.getOptimalNewCameraMatrix(cameraMatrix=cameraMatrix, 
                                                        distCoeffs=distCoeffs, 
                                                        imageSize=(w,h), 
                                                        alpha=1, 
                                                        newImgSize=(w,h))  
    
    undistortImg = cv2.undistort(src=originImg, 
                                cameraMatrix=cameraMatrix, 
                                distCoeffs=distCoeffs, 
                                newCameraMatrix=newCameraMatrix)
    return undistortImg, roi



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

def cvtPixel2MeterInVehicleFrame(pixelPosXY, pixelSize=PIXEL_TO_METER):
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










