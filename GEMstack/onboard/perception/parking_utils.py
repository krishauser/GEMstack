import cv2
import numpy as np


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

    min_x = np.min(cornerPts[:, 0]) + 0.5
    max_x = np.max(cornerPts[:, 0]) - 0.5
    min_y = np.min(cornerPts[:, 1]) + 0.5
    max_y = np.max(cornerPts[:, 1]) - 0.5
    
    candidates = []
    
    for angleDegree in np.arange(-90, 90, angleStepDegree):
        rect = ((0, 0), (GEM_E4_LENGTH, GEM_E4_WIDTH), float(angleDegree))
        carBox = cv2.boxPoints(rect)
        
        for cx in np.arange(min_x, max_x, positionStrideMeter):
            for cy in np.arange(min_y, max_y, positionStrideMeter):
                car_box_shifted = carBox + np.array([cx, cy])
                
                if judgeRectInPolygon(car_box_shifted, cornerPts):
                    candidates.append((cx, cy, angleDegree))
    
    return candidates


def drawCarPose(img, center, angleDegree, color=(0, 0, 255), scale=100):
    rect = (center, (GEM_E4_LENGTH, GEM_E4_WIDTH), float(angleDegree))
    box = cv2.boxPoints(rect)
    box_scaled = np.int32(box * scale)

    cv2.polylines(img, [box_scaled], isClosed=True, color=color, thickness=2)


def visualizeCandidateCarPoses(cornerPts, candidates, scale=100):
    img = np.zeros((1000, 1000, 3), dtype=np.uint8)

    parking_polygon = np.int32(cornerPts * scale)
    cv2.polylines(img, [parking_polygon], isClosed=True, color=(0, 255, 0), thickness=2)

    for (x, y, angle) in candidates:
        drawCarPose(img, (x, y), angle, color=(100, 100, 255), scale=scale)

    cv2.imshow("Parking Candidates", img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


if __name__ == "__main__":
    cornerPts = np.array([
        [8,9],
        [4,5],
        [1,15],
        [5,9],
    ]).astype(np.float32)  # any order
    
    
    candidates = findAllCandidateParkingLot(cornerPts)
    
    print(f"find {len(candidates)} candidates")
    print(f"candidates: {candidates[::10]}")
    for idx, pose in enumerate(candidates[::10]):
        thetaDegree = -(90 - (-pose[2])) if abs(- (90 - (-pose[2]))) < 90 else 180 + (-(90 - (-pose[2])))
        print(f"Candidate {idx}, Pose: centerXY:({pose[0]}, {pose[1]}), thetaDegree:{thetaDegree}")
        # OpenCV (rotation is from the lowest point, range [-90, 0])
        # counter-clock-wise is negative
        # clock-wise is positive


    visualizeCandidateCarPoses(cornerPts, candidates[::10])



















    
    