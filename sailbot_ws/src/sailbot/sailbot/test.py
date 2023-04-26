import sys
import time
sys.path.append('c:\python37\lib\site-packages')

import cv2

# Define the lower and upper bounds of the neon orange color in HSV format
orange_lower = (0, 90, 80)
orange_upper = (20, 255, 255)

# Set the minimum area requirement for an object to be considered
min_area = 7500
frame_count = 0
start_time = time.time()

# Open the default camera
cap = cv2.VideoCapture("/dev/video0")


# Set the resolution of the camera
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

while True:
    # Read a frame from the camera
    ret, frame = cap.read()

    # Convert the frame to the HSV color space
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Create a mask for the neon orange color and apply it to the original frame
    neon_orange_mask = cv2.inRange(hsv, orange_lower, orange_upper)

    # Find contours in the mask
    contours, _ = cv2.findContours(neon_orange_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Draw a bounding box around the largest contour (if one exists and its area is greater than min_area)
    if len(contours) > 0:
        largest_contour = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest_contour)
        if area > min_area:
            x, y, w, h = cv2.boundingRect(largest_contour)
            cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)

    # Display the camera view and the neon orange blob detection with a bounding box
    cv2.imshow('Camera View and Neon Orange Blob Detection', frame)

    # Add frame rate calculation and display
    frame_count += 1
    elapsed_time = time.time() - start_time
    if elapsed_time > 1:
        fps = frame_count / elapsed_time
        cv2.putText(frame, f'FPS: {int(fps)}', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        frame_count = 0
        start_time = time.time()

    # Wait for a key press and check if the user pressed the 'q' key
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the camera and close the window
cap.release()
cv2.destroyAllWindows()