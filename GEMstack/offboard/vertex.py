import cv2

# Initialize a list to store the coordinates
vertex_coordinates = []

def click_event(event, x, y, flags, param):
    # When the left mouse button is clicked, save the coordinates and draw a circle at the position
    if event == cv2.EVENT_LBUTTONDOWN:
        vertex_coordinates.append((x, y))
        cv2.circle(img, (x, y), 5, (255, 0, 0), -1)
        cv2.imshow('image', img)

# Read the image
img = cv2.imread('data/foam_block/color1.png')
cv2.imshow('image', img)

# Set the mouse callback function to 'click_event'
cv2.setMouseCallback('image', click_event)

# Wait until any key is pressed
cv2.waitKey(0)

# Close all OpenCV windows
cv2.destroyAllWindows()

# Write the coordinates to a file
with open('vertex_coordinates.txt', 'w') as file:
    for coord in vertex_coordinates:
        file.write(f'{coord[0]}, {coord[1]}\n')

print("Coordinates have been saved to 'vertex_coordinates.txt'")
