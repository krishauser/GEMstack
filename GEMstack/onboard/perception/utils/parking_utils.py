import cv2
import math
import numpy as np
from scipy.spatial import ConvexHull


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
    carBox = cv2.boxPoints(rect)
    carBox = carBox + np.array([cx, cy])
    return carBox


def find_all_candidate_parking_spots(cornerPts, angleStepDegree=10, positionStrideMeter=0.5):
    cornerPts = np.array(cornerPts, dtype=np.float32)
    min_x = np.min(cornerPts[:, 0]) + 0.5
    max_x = np.max(cornerPts[:, 0]) - 0.5
    min_y = np.min(cornerPts[:, 1]) + 0.5
    max_y = np.max(cornerPts[:, 1]) - 0.5
    
    candidates = []
    
    refAngleDegree = getMaxLenEdgeAngleDegree(cornerPts)
    
    for angleDegree in np.arange(-90, 90, angleStepDegree):
        rect = ((0, 0), (GEM_E4_LENGTH, GEM_E4_WIDTH), float(angleDegree))
    
    for angleDegree in np.arange(refAngleDegree, refAngleDegree+1):
        rect = ((0, 0), (GEM_E4_LENGTH, GEM_E4_WIDTH), float(angleDegree))
        carBox = cv2.boxPoints(rect)
        
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
    return maxPt1, maxPt2


def getMaxLenEdgeAngleDegree(cornerPts):
    pt1, pt2 = findMaxLenEdgePoints(cornerPts)
    connectVec = pt1 - pt2
    return - np.arctan(connectVec[0]/connectVec[1]) / np.pi * 180 + 90


def drawCarPose(img, center, angleDegree, color=(0, 0, 255), scale=100):
    rect = (center, (GEM_E4_LENGTH, GEM_E4_WIDTH), float(angleDegree))
    box = cv2.boxPoints(rect)
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


def select_best_candidate(candidates, cornerPts):
    best_candidate = candidates[0]
    max_score = float('-inf')
    for pose in candidates:
        score = calculateCandidateScore(pose, cornerPts)
        if score > max_score:
            best_candidate = pose
            max_score = score
    return best_candidate


def normalize_yaw(yaw):
    """Normalize yaw to [0, π) for orientation equivalence."""
    return yaw % math.pi


def get_parking_obstacles(vertices):
    vertices = np.array(vertices)
    n = len(vertices)
    segment_info = []

    for i in range(n):
        p1 = vertices[i]
        p2 = vertices[(i + 1) % n]
        center = (p1 + p2) / 2
        delta = p2 - p1
        length = np.linalg.norm(delta)
        yaw = math.atan2(delta[1], delta[0])
        yaw = normalize_yaw(yaw)
        segment_info.append(((center[0], center[1], 0.0, yaw), (length, 0.05, 1.0)))

    # Sort by length and remove the two shortest segments
    sorted_indices = sorted(range(len(segment_info)), key=lambda i: segment_info[i][1][0])
    indices_to_remove = set(sorted_indices[:2])

    filtered_positions = [segment_info[i][0] for i in range(len(segment_info)) if i not in indices_to_remove]
    filtered_dimensions = [segment_info[i][1] for i in range(len(segment_info)) if i not in indices_to_remove]

    return filtered_positions, filtered_dimensions

def order_points_all(points_2d):
    points_np = np.array(points_2d)
    hull = ConvexHull(points_np)
    ordered = [points_np[i] for i in hull.vertices]
    return ordered

def detect_parking_spot(cone_3d_centers):
    # Initial variables
    goal_parking_spot = None
    parking_obstacles_pose = []
    parking_obstacles_dim = []

    # Get 2D cone centers
    cone_ground_centers = np.array(cone_3d_centers)
    cone_ground_centers_2D = cone_ground_centers[:, :2]
    ordered_cone_ground_centers_2D = order_points_all(cone_ground_centers_2D)
    # print(f"-----cone_ground_centers_2D: {len(ordered_cone_ground_centers_2D)}")

    # Check if points can form a polygon, if not then there is no goal parking spot
    if len(cone_3d_centers) != len(ordered_cone_ground_centers_2D):
        return None, [], [], []
    
    # Find the valid candidates
    candidates = find_all_candidate_parking_spots(ordered_cone_ground_centers_2D)
    # print(f"-----candidates: {candidates}")

    # Select the best candidate parking spot
    if len(candidates) > 0:
        parking_obstacles_pose, parking_obstacles_dim = get_parking_obstacles(ordered_cone_ground_centers_2D)
        # print(f"-----parking_obstacles: {self.parking_obstacles_pose}")
        goal_parking_spot = select_best_candidate(candidates, ordered_cone_ground_centers_2D)
        # print(f"-----goal_parking_spot: {self.goal_parking_spot}")
    return goal_parking_spot, parking_obstacles_pose, parking_obstacles_dim, ordered_cone_ground_centers_2D