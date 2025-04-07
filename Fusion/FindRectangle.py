import cv2
import os
import numpy as np

from ClickPoints import clickPoints



GEM_E4_LENGTH = 3.6  # m
GEM_E4_WIDTH  = 1.5  # m






def judgeRectInPolygon(rectPts:np.ndarray, polygonPts:np.ndarray):
    polygon = polygonPts.reshape((-1, 1, 2)).astype(np.float32)
    for pt in rectPts:
        inside = cv2.pointPolygonTest(polygon, tuple(pt), measureDist=False)
        if inside < 0:
            return False
    return True


def findAllCandidateParkingLot(cornerPts, angleStepDegree=5, positionStrideMeter=0.5):
    
    cornerPts = np.array(cornerPts, dtype=np.float32)
    
    # 先得到 bounding box 范围（缩小一些，避免跑出区域）
    min_x = np.min(cornerPts[:, 0]) + 0.5
    max_x = np.max(cornerPts[:, 0]) - 0.5
    min_y = np.min(cornerPts[:, 1]) + 0.5
    max_y = np.max(cornerPts[:, 1]) - 0.5
    
    candidates = []
    
    for angleDegree in np.arange(-90, 90, angleStepDegree):
        rect = ((0, 0), (GEM_E4_LENGTH, GEM_E4_WIDTH), float(angleDegree))
        carBox = cv2.boxPoints(rect)  # 车体角点，以原点为中心
        
        for cx in np.arange(min_x, max_x, positionStrideMeter):
            for cy in np.arange(min_y, max_y, positionStrideMeter):
                car_box_shifted = carBox + np.array([cx, cy])
                
                if judgeRectInPolygon(car_box_shifted, cornerPts):
                    candidates.append((cx, cy, angleDegree))
    
    return candidates


def drawCarPose(img, center, angleDegree, color=(0, 0, 255), scale=100):
    rect = (center, (GEM_E4_LENGTH, GEM_E4_WIDTH), float(angleDegree))
    box = cv2.boxPoints(rect)  # 四个角点
    box_scaled = np.int32(box * scale)

    cv2.polylines(img, [box_scaled], isClosed=True, color=color, thickness=2)


def visualizeCandidateCarPoses(cornerPts, candidates, scale=100):
    img = np.zeros((1000, 1000, 3), dtype=np.uint8)

    # 画停车位边框
    parking_polygon = np.int32(cornerPts * scale)
    cv2.polylines(img, [parking_polygon], isClosed=True, color=(0, 255, 0), thickness=2)

    # 画所有合法停车姿态
    for (x, y, angle) in candidates:
        drawCarPose(img, (x, y), angle, color=(100, 100, 255), scale=scale)

    cv2.imshow("Parking Candidates", img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


if __name__ == "__main__":
    # cornerPts = np.array([
    #     [8,9],
    #     [4,5],
    #     [1,15],
    #     [5,9],
    # ]).astype(np.float32)  # any order
    
    cornerPts = np.random.randint(
        low=1,
        high=10,
        size=(4,2)
    ).astype(np.float32)
    
    cornerPts = clickPoints(
        imgPath = os.path.join(
            os.path.dirname(__file__),
            r"./Pics/FrontCam.png"
        )
    )/100
    
    print(f"\
        cornerPts = np.array([\n\
            [{cornerPts[0,0]},{cornerPts[0,1]}],\n\
            [{cornerPts[1,0]},{cornerPts[1,1]}],\n\
            [{cornerPts[2,0]},{cornerPts[2,1]}],\n\
            [{cornerPts[3,0]},{cornerPts[3,1]}],\n\
        ]).astype(np.float32)  # any order \
    ")
    
    candidates = findAllCandidateParkingLot(cornerPts)
    
    print(f"find {len(candidates)} candidates")
    for idx, pose in enumerate(candidates[::10]):
        thetaDegree = -(90 - (-pose[2])) if abs(- (90 - (-pose[2]))) < 90 else 180 + (-(90 - (-pose[2])))
        print(f"Candidate {idx}, Pose: centerXY:({pose[0]}, {pose[1]}), thetaDegree:{thetaDegree}")
        # OpenCV (rotation is from the lowest point, range [-90, 0])
        # counter-clock-wise is negative
        # clock-wise is positive


    visualizeCandidateCarPoses(cornerPts, candidates[::10])


















# def checkFitWithMinAreaRect(pts:np.ndarray, rectLongMin=GEM_E4_LENGTH, rectShortMin=GEM_E4_WIDTH):
    
#     def getRotatedRectBox(center, size, angleDegree):
#         """
#         给定中心坐标、尺寸和角度，返回矩形4个顶点
#         center: (x, y)
#         size: (width, height)
#         angleDegree: 逆时针旋转角度（OpenCV风格）
#         """
#         rect = (center, size, angleDegree)
#         box = cv2.boxPoints(rect)  # 得到4个点
#         return box  # 转换为整数像素点或保留float点
    
#     # 外接矩形
#     outRect = cv2.minAreaRect(pts)
#     (centerX, centerY), (width, length), rotation = outRect
#     print(f"rotation: {rotation}")
#     # OpenCV 返回的 rotation 单位是 角度，范围为 [-90°, 0°]
#     # -90°	矩形长边在 Y 轴方向
#     # 0°	长边在 X 轴方向

#     SwitchOrder = False
#     if width > length:
#         outBoxLong, outBoxShort = width, length
#     else:
#         outBoxLong, outBoxShort = length, width
#         SwitchOrder = True
#         print("SwitchOrder")

#     canFit = True
#     if SwitchOrder is True:
#         rotation += 90
    

#     # judge fitting
#     if outBoxLong < rectLongMin:
#         print(f"CAN NOT FIT : outBoxLong{outBoxLong} < rectLongMin{rectLongMin}")
#         canFit = False
#     if outBoxShort < rectShortMin:
#         print(f"CAN NOT FIT : outBoxShort{outBoxShort} < rectShortMin{rectShortMin}")
#         canFit = False

#     centerXY = (centerX, centerY)
    
    
#     outerBoxPts = getRotatedRectBox(centerXY, (outBoxLong, outBoxShort), rotation)
#     innerBoxPts = getRotatedRectBox(centerXY, (rectLongMin, rectShortMin), rotation)
    
#     return canFit, centerXY, rotation, innerBoxPts, outerBoxPts


# if __name__ == "__main__":
    # cornerPts = np.array([
    #     [8,10],
    #     [3,10],
    #     [5,5],
    #     [1,5],

    # ]).astype(np.float32)  # any order

    # cornerPts = np.random.randint(
    #     low=1,
    #     high=10,
    #     size=(4,2)
    # ).astype(np.float32)
    
    # print(cornerPts)


    # canFit, center, rotation, boxInner, boxOuter = checkFitWithMinAreaRect(cornerPts)
    
    # print(canFit, rotation)

    # img = np.zeros((600, 600, 3), dtype=np.uint8)
    # ptsInt = np.int32(cornerPts * 40)  # 放大方便看
    # boxIntInner = np.int32(boxInner * 40)
    # boxIntOuter = np.int32(boxOuter * 40)

    # cv2.polylines(img, [ptsInt], isClosed=True, color=(0,255,0), thickness=2)
    # cv2.polylines(img, [boxIntInner], isClosed=True, color=(255,0,0), thickness=2)
    # cv2.polylines(img, [boxIntOuter], isClosed=True, color=(0,0,255), thickness=2)

    # cv2.imshow("fit check", img)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()
    
    
