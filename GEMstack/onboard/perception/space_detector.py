import cv2
import numpy as np
import matplotlib.pyplot as plt

def detect_objects(image_path):
    """
    Detect traffic cones (blue) and foam boxes in an image with improved filtering.
    
    Args:
        image_path (str): Path to the input image
        
    Returns:
        dict: Dictionary containing coordinates of detected cones and boxes
    """
    # Load image
    image = cv2.imread(image_path)
    if image is None:
        raise FileNotFoundError(f"Could not open or find the image: {image_path}")
    
    # Convert to RGB (for visualization) and HSV (for color detection)
    rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    
    # Create copies for visualization
    output_vis = rgb_image.copy()
    
    # Define color ranges for traffic cones (BLUE)
    lower_blue = np.array([100, 150, 120])  # More saturated and brighter blue
    upper_blue = np.array([130, 255, 255])
    blue_mask = cv2.inRange(hsv_image, lower_blue, upper_blue)
    
    # Define color ranges for foam boxes (white/light colored)
    lower_white = np.array([0, 0, 200])
    upper_white = np.array([180, 30, 255])
    white_mask = cv2.inRange(hsv_image, lower_white, upper_white)
    
    # Apply morphological operations to clean up masks
    kernel = np.ones((3, 3), np.uint8)
    
    # Process cone mask with more aggressive cleaning
    cone_mask = cv2.morphologyEx(blue_mask, cv2.MORPH_OPEN, kernel)
    cone_mask = cv2.morphologyEx(cone_mask, cv2.MORPH_CLOSE, kernel)
    
    # Process box mask
    box_mask = cv2.morphologyEx(white_mask, cv2.MORPH_OPEN, kernel)
    box_mask = cv2.morphologyEx(box_mask, cv2.MORPH_CLOSE, kernel)
    
    # Find contours for traffic cones
    cone_contours, _ = cv2.findContours(cone_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    # Find contours for foam boxes
    box_contours, _ = cv2.findContours(box_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    # Store results
    results = {
        'traffic_cones': [],
        'foam_boxes': []
    }
    
    # Process cone contours with stricter filtering
    for contour in cone_contours:
        # Minimum size to filter noise
        area = cv2.contourArea(contour)
        if area < 200 or area > 2000:  # Filter by size range
            continue
            
        # Get bounding rectangle
        x, y, w, h = cv2.boundingRect(contour)
        
        # Calculate aspect ratio
        aspect_ratio = h / w
        
        # Cones should be somewhat taller than wide
        if aspect_ratio < 1.2 or aspect_ratio > 3.0:
            continue
            
        # Check solidity (ratio of contour area to its convex hull area)
        hull = cv2.convexHull(contour)
        hull_area = cv2.contourArea(hull)
        if hull_area > 0:
            solidity = float(area) / hull_area
            if solidity < 0.7:  # Must be mostly solid
                continue
        
        # Calculate center coordinates
        center_x = x + w // 2
        center_y = y + h // 2
        
        # Add to results
        results['traffic_cones'].append({
            'center': (center_x, center_y),
            'top_left': (x, y),
            'width': w,
            'height': h
        })
        
        # Draw for visualization
        cv2.rectangle(output_vis, (x, y), (x + w, y + h), (0, 255, 0), 2)
        cv2.circle(output_vis, (center_x, center_y), 5, (255, 0, 0), -1)
        cv2.putText(output_vis, f"C({center_x},{center_y})", (x, y - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
    
    # Process box contours
    for contour in box_contours:
        area = cv2.contourArea(contour)
        if area < 2000:  # Boxes should be larger
            continue
            
        # Get bounding rectangle
        x, y, w, h = cv2.boundingRect(contour)
        
        # Filter by aspect ratio (boxes are more square-like)
        aspect_ratio = max(w, h) / min(w, h)
        if aspect_ratio > 2.5:  # Not too elongated
            continue
            
        # Calculate center coordinates
        center_x = x + w // 2
        center_y = y + h // 2
        
        # Add to results
        results['foam_boxes'].append({
            'center': (center_x, center_y),
            'top_left': (x, y),
            'width': w,
            'height': h
        })
        
        # Draw for visualization
        cv2.rectangle(output_vis, (x, y), (x + w, y + h), (255, 0, 0), 2)
        cv2.circle(output_vis, (center_x, center_y), 5, (0, 255, 0), -1)
        cv2.putText(output_vis, f"B({center_x},{center_y})", (x, y - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 0, 0), 1)
    
    return results, output_vis

def detect_blue_cones_spatial(image_path, min_area=200, max_area=2000):
    """
    Special detection function optimized for blue traffic cones with spatial constraints.
    
    Args:
        image_path (str): Path to the input image
        min_area (int): Minimum contour area
        max_area (int): Maximum contour area
        
    Returns:
        list: List of detected cone coordinates
    """
    # Load image
    image = cv2.imread(image_path)
    if image is None:
        raise FileNotFoundError(f"Could not open or find the image: {image_path}")
    
    # Convert to RGB and HSV
    rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    
    # Create output image
    output_vis = rgb_image.copy()
    
    # Very specific blue for the cones in this image
    lower_blue = np.array([100, 150, 120])
    upper_blue = np.array([130, 255, 255])
    blue_mask = cv2.inRange(hsv_image, lower_blue, upper_blue)
    
    # Clean mask
    kernel = np.ones((3, 3), np.uint8)
    blue_mask = cv2.morphologyEx(blue_mask, cv2.MORPH_OPEN, kernel)
    blue_mask = cv2.morphologyEx(blue_mask, cv2.MORPH_CLOSE, kernel)
    
    # Find contours
    contours, _ = cv2.findContours(blue_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    # Store valid cone coordinates
    cones = []
    
    # Process each contour with strict filtering
    for contour in contours:
        area = cv2.contourArea(contour)
        if area < min_area or area > max_area:
            continue
            
        # Get bounding rectangle
        x, y, w, h = cv2.boundingRect(contour)
        
        # Filter by aspect ratio (cones are taller than wide)
        aspect_ratio = h / w
        if aspect_ratio < 1.2 or aspect_ratio > 3.0:
            continue
            
        # Check solidity
        hull = cv2.convexHull(contour)
        hull_area = cv2.contourArea(hull)
        if hull_area > 0:
            solidity = float(area) / hull_area
            if solidity < 0.7:
                continue
                
        # Calculate moments to get centroid
        M = cv2.moments(contour)
        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
        else:
            cx = x + w // 2
            cy = y + h // 2
        
        # Add to cones list
        cones.append({
            'center': (cx, cy),
            'top_left': (x, y),
            'width': w,
            'height': h,
            'area': area
        })
        
        # Draw on output image
        cv2.rectangle(output_vis, (x, y), (x + w, y + h), (0, 255, 0), 2)
        cv2.circle(output_vis, (cx, cy), 3, (255, 0, 0), -1)
        label = f"C({cx},{cy})"
        cv2.putText(output_vis, label, (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 
                    0.4, (0, 255, 0), 1)
    
    # Return results and visualization
    return cones, output_vis

# Main function to run on an image
def main(image_path):
    """Run detection on an image and display results"""
    # Run improved cone detection
    print("Running improved cone detection...")
    cones, output_vis = detect_blue_cones_spatial(image_path)
    
    # Print results
    print(f"Detected {len(cones)} traffic cones")
    for i, cone in enumerate(cones):
        print(f"  Cone {i+1}: Center = {cone['center']}")
    
    # Display results
    plt.figure(figsize=(12, 8))
    plt.imshow(output_vis)
    plt.title(f"Detected {len(cones)} Traffic Cones")
    plt.axis("off")
    plt.tight_layout()
    plt.savefig("cone_detection_result.png")
    plt.show()
    
    return cones, output_vis

if __name__ == "__main__":
    image_path = "/home/aadarshhegde/Documents/GEMstack/data/parking_data-20250312T213928Z-001/parking_data/camera_fl95.png"  # Replace with your image path
    
    try:
        cones, _ = main(image_path)
    except Exception as e:
        print(f"Error: {e}")