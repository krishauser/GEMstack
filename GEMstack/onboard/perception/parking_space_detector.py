import numpy as np
from scipy.spatial import distance
from collections import defaultdict

def order_points(points):
    """Orders points in clockwise order around centroid"""
    centroid = np.mean(points, axis=0)
    vectors = points - centroid
    angles = np.arctan2(vectors[:, 1], vectors[:, 0])
    return points[np.argsort(angles)]

def find_parking_spots(cone_positions, tolerance=0.1):
    """Improved parking spot detection that avoids duplicates"""
    cones_2d = cone_positions[:, :2]
    n = len(cones_2d)
    pairs = []
    
    # Generate all possible cone pairs
    for i in range(n):
        for j in range(i+1, n):
            p1 = cones_2d[i]
            p2 = cones_2d[j]
            midpoint = (round((p1[0]+p2[0])/2/tolerance)*tolerance, 
                        round((p1[1]+p2[1])/2/tolerance)*tolerance)
            dist = round(distance.euclidean(p1, p2)/tolerance)*tolerance
            pairs.append((midpoint, dist, i, j))
    
    # Group pairs by midpoint and distance
    groups = defaultdict(list)
    for p in pairs:
        groups[(p[0][0], p[0][1], p[1])].append((p[2], p[3]))
    
    # Find valid rectangles
    processed = set()
    rectangles = []
    
    for key in groups:
        group_pairs = groups[key]
        
        # We need exactly 2 pairs to form a rectangle (4 unique points)
        if len(group_pairs) >= 2:
            # Combine all possible pairs in this group
            for i in range(len(group_pairs)):
                for j in range(i+1, len(group_pairs)):
                    pair1 = group_pairs[i]
                    pair2 = group_pairs[j]
                    indices = set(pair1 + pair2)
                    
                    # Only proceed if we have exactly 4 unique points
                    if len(indices) == 4:
                        # Check if these points form a valid rectangle
                        points = cones_2d[list(indices)]
                        
                        # Calculate all pairwise distances
                        dists = distance.pdist(points)
                        unique_dists = np.unique(np.round(dists/tolerance)*tolerance)
                        
                        # A rectangle should have exactly 2 unique side lengths (with tolerance)
                        if len(unique_dists) == 2:
                            sorted_indices = tuple(sorted(indices))
                            if sorted_indices not in processed:
                                ordered = order_points(points)
                                rectangles.append(ordered)
                                processed.add(sorted_indices)
    
    return rectangles

def point_in_polygon(point, polygon):
    """Ray casting algorithm for point-in-polygon detection"""
    x, y = point
    n = len(polygon)
    inside = False
    
    for i in range(n):
        xi, yi = polygon[i]
        xj, yj = polygon[(i+1) % n]
        
        # Check if point is on edge
        if (np.isclose(x, xi) and np.isclose(y, yi)) or \
           (np.isclose(x, xj) and np.isclose(y, yj)):
            return True
        
        # Check intersection with edge
        if ((yi > y) != (yj > y)):
            x_intersect = (y - yi) * (xj - xi) / (yj - yi) + xi
            if x < x_intersect:
                inside = not inside
                
    return inside

def detect_parking_spots(cone_positions, object_positions):
    """Main detection function with improved accuracy"""
    rectangles = find_parking_spots(cone_positions)
    parking_spots = []
    
    for rect in rectangles:
        occupied = any(point_in_polygon(obj[:2], rect) for obj in object_positions)
        parking_spots.append({
            "corners": rect,
            "occupied": occupied
        })
    
    return parking_spots

# Example Usage
if __name__ == "__main__":
    cone_positions = np.array([
        [1, 1, 0], [1, 4, 0], [4, 1, 0], [4, 4, 0],  # Spot 1
        [4, 1, 0], [4, 4, 0], [9, 1, 0], [9, 4, 0]   # Spot 2
    ])

    object_positions = np.array([
        [2.5, 2.5, 0]  # Object inside Spot 1
    ])

    spots = detect_parking_spots(cone_positions, object_positions)

    for i, spot in enumerate(spots):
        status = "Occupied" if spot["occupied"] else "Available"
        print(f"Parking Spot {i+1}: {status}")
        print(f"Corners: {spot['corners']}\n")