import cv2
import numpy as np

# Initialize global variables
points = []

def order_points(pts):
    # Sort the points based on their x-coordinates
    x_sorted = pts[np.argsort(pts[:, 0]), :]

    # Grab the left-most and right-most points from the sorted
    # x-coordinate points
    left_most = x_sorted[:2, :]
    right_most = x_sorted[2:, :]

    # Now, sort the left-most coordinates according to their
    # y-coordinates so we can grab the top-left and bottom-left
    # points, respectively
    left_most = left_most[np.argsort(left_most[:, 1]), :]
    (tl, bl) = left_most

    # Now, sort the right-most coordinates according to their
    # y-coordinates so we can grab the top-right and bottom-right
    # points, respectively
    right_most = right_most[np.argsort(right_most[:, 1]), :]
    (tr, br) = right_most

    # Return the coordinates in top-left, top-right,
    # bottom-right, and bottom-left order
    return np.array([tl, tr, br, bl], dtype="float32")

def mouse_callback(event, x, y, flags, param):
    global points
    if event == cv2.EVENT_LBUTTONDOWN:
        if len(points) < 4:
            points.append((x, y))
            print(f"Point {len(points)} selected: ({x}, {y})")
            cv2.circle(param['image'], (x, y), 5, (0, 255, 0), -1)
            cv2.imshow("Select 4 Points", param['image'])
        if len(points) == 4:
            print("All points selected. Press any key to proceed.")
            cv2.waitKey(0)
            cv2.destroyAllWindows()
    elif event == cv2.EVENT_RBUTTONDOWN:
        points = []
        print("Points cleared. Please select 4 points again.")
        param['image'] = param['original_image'].copy()
        cv2.imshow("Select 4 Points", param['image'])

def transform_birdseye(image, src_points, dst_size, camera_in):
    try:
        if image is None or image.size == 0:
            raise ValueError("Invalid image data or failed to load image.")

        # Order points
        src_pts = order_points(np.array(src_points, dtype="float32"))
        
        # Define the output size
        W, H = dst_size
        dst_pts = np.array([
            [0, 0],
            [W - 1, 0],
            [W - 1, H - 1],
            [0, H - 1]
        ], dtype=np.float32)
        
        # Perform undistortion using intrinsics
        undistorted_image = cv2.undistort(image, camera_in, None)
        
        # Compute perspective transform matrix
        M = cv2.getPerspectiveTransform(src_pts, dst_pts)
        print("Perspective Transform Matrix:\n", M)

        # Perform perspective warp
        warped = cv2.warpPerspective(undistorted_image, M, (W, H))
        return warped, M
    
    except Exception as e:
        print(f"Error during transformation: {e}")
        return None, None

def main():
    global points
    points = []  # Reset points list

    # Camera intrinsics
    # camera_in = np.array([
    #     [684.83331299, 0.0, 573.37109375],
    #     [0.0, 684.60968018, 363.70092773],
    #     [0.0, 0.0, 1.0]
    # ], dtype=np.float32)

    camera_in = np.array([
        [1, 0.0, 0],
        [0.0, 1, 0],
        [0.0, 0.0, 1.0]
    ], dtype=np.float32)

    # Load image
    # image_path = "/home/aadarshhegde/Documents/GEMstack/data/parking_data-20250312T213928Z-001/parking_data/front_cam97.png"
    image_path = "/home/aadarshhegde/Documents/GEMstack/data/parking_data-20250312T213928Z-001/parking_data/camera_fl95.png"
    # image_path = '/home/aadarshhegde/Documents/GEMstack/data/parking_others/parking_data-20250312T213928Z-001/parking_data/camera_fr109.png'
    image = cv2.imread(image_path)

    if image is None:
        print("Could not load image. Check the path.")
        return

    # Clone for display
    display_image = image.copy()
    
    # Select points using mouse
    print("Select 4 points for the bird's-eye view. Press 'Esc' to finish.")
    cv2.imshow("Select 4 Points", display_image)
    cv2.setMouseCallback("Select 4 Points", mouse_callback, {'image': display_image, 'original_image': image.copy()})
    cv2.waitKey(0)

    if len(points) != 4:
        print("Error: Please select exactly 4 points.")
        return

    # Output size (adjust to your needs)
    out_width = 800
    out_height = 600

    # Perform the transformation
    warped, M = transform_birdseye(image, points, (out_width, out_height), camera_in)

    if warped is not None:
        # Display the results
        cv2.imshow("Original", image)
        cv2.imshow("Bird's-Eye View", warped)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()