import cv2
import os
import numpy as np


GEM_E4_LENGTH = 3.6  # m
GEM_E4_WIDTH  = 1.5  # m


def clickPoints(imgPath=None, numPoints=4):
    clicked_pts = []

    def mouse_callback(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN and len(param) < numPoints:
            param.append((x, y))
            print(f"Point {len(param)}: ({x}, {y})")

    if imgPath:
        img = cv2.imread(imgPath, flags=cv2.IMREAD_COLOR)
    else:
        img = np.ones((600, 800, 3), dtype=np.uint8) * 255

    cv2.namedWindow("Click Points")
    cv2.setMouseCallback("Click Points", mouse_callback, clicked_pts)

    while True:
        img_copy = img.copy()

        for pt in clicked_pts:
            cv2.circle(img_copy, pt, 5, (0, 0, 255), -1)

        if len(clicked_pts) == numPoints:
            cv2.polylines(img_copy, [np.array(clicked_pts)], isClosed=True, color=(0, 255, 0), thickness=2)

        cv2.imshow("Click Points", img_copy)

        key = cv2.waitKey(10)
        if key == 27 or key == ord("q"):  # ESC
            break
        elif key == ord('r'):
            clicked_pts.clear()
        
    cv2.destroyAllWindows()

    return np.array(clicked_pts, dtype=np.float32)



def distPoint2LineAB(p, a, b):
    pa = p - a
    pb = p - b
    ba = b - a
    
    dist = np.abs(np.cross(pa, pb)) / np.linalg.norm(ba)
    return dist


def calculateCandidateScore(pose, cornerPoints):
    position = pose[0:2]
    
    score = np.min([
        distPoint2LineAB(position, cornerPoints[0], cornerPoints[1]),
        distPoint2LineAB(position, cornerPoints[1], cornerPoints[2]),
        distPoint2LineAB(position, cornerPoints[2], cornerPoints[3]),
        distPoint2LineAB(position, cornerPoints[3], cornerPoints[0]),
    ])

    return score


def judgeRectInPolygon(rectPts:np.ndarray, polygonPts:np.ndarray):
    polygon = polygonPts.reshape((-1, 1, 2)).astype(np.float32)
    for pt in rectPts:
        inside = cv2.pointPolygonTest(polygon, tuple(pt), measureDist=False)
        if inside < 0:
            return False
    return True


def cvtPose2CarBox(carPose):
    angleDegree = carPose[-1]
    cx, cy = carPose[0:2]
    rect = ((0, 0), (GEM_E4_LENGTH, GEM_E4_WIDTH), float(angleDegree))
    carBox = cv2.boxPoints(rect)  # 车体角点，以原点为中心
    carBox = carBox + np.array([cx, cy])
    return carBox


def findAllCandidateParkingLot(cornerPts, angleStepDegree=5, positionStrideMeter=0.5):
    
    cornerPts = np.array(cornerPts, dtype=np.float32)
    
    # 先得到 bounding box 范围（缩小一些，避免跑出区域）
    min_x = np.min(cornerPts[:, 0]) + 0.0
    max_x = np.max(cornerPts[:, 0]) - 0.0
    min_y = np.min(cornerPts[:, 1]) + 0.0
    max_y = np.max(cornerPts[:, 1]) - 0.0
    
    candidates = []
    
    refAngleDegree = getMaxLenEdgeAngleDegree(cornerPts)
    
    # for angleDegree in np.arange(-90, 90, angleStepDegree):
    #     rect = ((0, 0), (GEM_E4_LENGTH, GEM_E4_WIDTH), float(angleDegree))
    
    for angleDegree in np.arange(refAngleDegree, refAngleDegree+5, angleStepDegree):
        rect = ((0, 0), (GEM_E4_LENGTH, GEM_E4_WIDTH), float(angleDegree))
        carBox = cv2.boxPoints(rect)  # 车体角点，以原点为中心
        
        for cx in np.arange(min_x, max_x, positionStrideMeter):
            for cy in np.arange(min_y, max_y, positionStrideMeter):
                car_box_shifted = carBox + np.array([cx, cy])
                
                if judgeRectInPolygon(car_box_shifted, cornerPts):
                    candidates.append((cx, cy, angleDegree))
    
    return candidates


def findMaxLenEdgePoints(cornerPts):
    maxLen = 0
    maxPt1, maxPt2 = None, None
    for idx in range(-1, 3):
        pt1, pt2 = cornerPts[idx], cornerPts[idx+1]
        tempLen = np.linalg.norm(pt1 - pt2)
        
        if tempLen > maxLen:
            maxPt1, maxPt2 = pt1, pt2
            maxLen = tempLen
    
    print(f"MaxLen: {maxLen}, pt1: {maxPt1}, pt2: {maxPt2}")
    return maxPt1, maxPt2


def getMaxLenEdgeAngleDegree(cornerPts):
    pt1, pt2 = findMaxLenEdgePoints(cornerPts)
    connectVec = pt1 - pt2
    return - np.arctan(connectVec[0]/connectVec[1]) / np.pi * 180 + 90


def drawCarPose(img, center, angleDegree, color=(0, 0, 255), scale=100):
    rect = (center, (GEM_E4_LENGTH, GEM_E4_WIDTH), float(angleDegree))
    box = cv2.boxPoints(rect)  # 四个角点
    box_scaled = np.int32(box * scale)

    cv2.polylines(img, [box_scaled], isClosed=True, color=color, thickness=2)


def visualizeCandidateCarPoses(cornerPts, candidates, bestPose=None, scale=100):
    img = np.zeros((1000, 1000, 3), dtype=np.uint8)

    parking_polygon = np.int32(cornerPts * scale)
    cv2.polylines(img, [parking_polygon], isClosed=True, color=(0, 255, 0), thickness=2)

    for (x, y, angle) in candidates:
        drawCarPose(img, (x, y), angle, color=(100, 100, 255), scale=scale)
    
    if bestPose is not None:
        drawCarPose(img, bestPose[:2], bestPose[2], color=(0, 255, 0), scale=scale)

    cv2.imshow("Parking Candidates", img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


def scoreAndSortCandidates(candidates, cornerPts):
    scoredCandidates = []
    for pose in candidates:
        score = calculateCandidateScore(pose, cornerPts)
        scoredCandidates.append((pose, score))

    scoredCandidates.sort(key=lambda x: x[1], reverse=True)
    
    top_k = 10
    for idx, (pose, score) in enumerate(scoredCandidates[:top_k]):
        print(f"Top-{idx+1}: center=({pose[0]:.2f}, {pose[1]:.2f}), angle={pose[2]}°, score={score:.4f}")
    
    return scoredCandidates


BASE_VEHICLE_DIST = 1.10

def cvtCenter2VehiclePos(center, cornerPts):
    pt1, pt2 = findMaxLenEdgePoints(cornerPts)
    near, far = (pt1, pt2) if np.linalg.norm(pt1) < np.linalg.norm(pt2) else (pt2, pt1)
    directionNorm = (near - far) / np.linalg.norm(near - far)
    vehicle = center + directionNorm * BASE_VEHICLE_DIST
    return vehicle




if __name__ == "__main__":
    
    cornerPts = np.array([
        [14.52, -3.55],
        [11.10, -3.39],
        [14.56, -8.41],
        [17.82, -8.65],
    ])
    
    
    candidates = findAllCandidateParkingLot(cornerPts)
    
    
    
    print(f"find {len(candidates)} candidates")
    
    scoredCandidates = scoreAndSortCandidates(candidates, cornerPts)
    
    candidates = [scored[0] for scored in scoredCandidates]
    
    print("candidates after score and sort:")
    print(candidates)
    
    
    centerWithScore = candidates[0]
    
    vehiclePos = cvtCenter2VehiclePos(centerWithScore[0:2], cornerPts)
    print(f"Vehicle Position: {vehiclePos}")
    
    
    # for idx, pose in enumerate(candidates[:5:]):
    #     thetaDegree = -(90 - (-pose[2])) if abs(- (90 - (-pose[2]))) < 90 else 180 + (-(90 - (-pose[2])))
    #     print(f"Candidate {idx}, Pose: centerXY:({pose[0]}, {pose[1]}), thetaDegree:{thetaDegree}")
    #     # OpenCV (rotation is from the lowest point, range [-90, 0])
    #     # counter-clock-wise is negative
    #     # clock-wise is positive


    # if len(candidates) != 0:
    #     print(cvtPose2CarBox(candidates[0]))
    #     best_pose = scoredCandidates[0][0]
    #     visualizeCandidateCarPoses(cornerPts, candidates[:5], bestPose=best_pose)

















    
