import cv2
import math
import numpy as np
from scipy.spatial import ConvexHull
from .constants import *


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


def cvtCenter2VehiclePos(center, cornerPts):
    pt1, pt2 = findMaxLenEdgePoints(cornerPts)
    near, far = (pt1, pt2) if np.linalg.norm(pt1) < np.linalg.norm(pt2) else (pt2, pt1)
    directionNorm = (near - far) / np.linalg.norm(near - far)
    vehicle = center + directionNorm * (GEM_E4_LENGTH / 2)
    return vehicle


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
            maxLen = tempLen
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
        distance_to_origin = np.linalg.norm(center)  # Distance from center to origin

        segment_info.append({
            "position": (center[0], center[1], 0.0, yaw),
            "dimension": (length, 0.05, 1.0),
            "length": length,
            "distance_to_origin": distance_to_origin
        })

    if len(segment_info) < 2:
        # Not enough segments to remove one
        return [seg["position"] for seg in segment_info], [seg["dimension"] for seg in segment_info]

    # Find the two shortest segments
    segment_info_sorted = sorted(segment_info, key=lambda s: s["length"])
    shortest = segment_info_sorted[0]
    second_shortest = segment_info_sorted[1]

    # Determine which one is closer to the origin and remove that
    if shortest["distance_to_origin"] < second_shortest["distance_to_origin"]:
        segment_to_remove = shortest
    else:
        segment_to_remove = second_shortest

    # Filter out the selected segment
    filtered_segments = [seg for seg in segment_info if seg != segment_to_remove]

    # Prepare final outputs
    filtered_positions = [seg["position"] for seg in filtered_segments]
    filtered_dimensions = [seg["dimension"] for seg in filtered_segments]

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
        best_candidate = select_best_candidate(candidates, ordered_cone_ground_centers_2D)
        # print(f"-----best_parking_spot: {best_candidate}")
        goal_xy = cvtCenter2VehiclePos(best_candidate[0:2], ordered_cone_ground_centers_2D)
        goal_yaw = best_candidate[2]
        goal_parking_spot = (goal_xy[0], goal_xy[1], goal_yaw)
        # print(f"-----goal_parking_spot: {goal_parking_spot}")
    return goal_parking_spot, parking_obstacles_pose, parking_obstacles_dim, ordered_cone_ground_centers_2D
