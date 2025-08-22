import cv2 as cv
import numpy as np
import pyautogui

# Open webcam
cam = cv.VideoCapture(0)
if not cam.isOpened():
    print(" ERROR: Could not open webcam")
    exit()

print(" Webcam started — press ESC to quit.")

# Color ranges in HSV
lower_yellow = np.array([20, 100, 100])
upper_yellow = np.array([40, 255, 255])
lower_green = np.array([50, 100, 100])
upper_green = np.array([80, 255, 255])

# Main loop
while True:
    ret, frame = cam.read()
    if not ret or frame is None:
        continue  # Skip if no frame captured

    # Flip for mirror effect
    frame = cv.flip(frame, 1)

    # Smooth the image slightly to reduce noise
    img_smooth = cv.GaussianBlur(frame, (7, 7), 0)

    # Create mask (single channel, uint8)
    mask = np.zeros(frame.shape[:2], dtype=np.uint8)
    mask[50:350, 50:350] = 255  # ROI box

    # Apply ROI mask
    img_roi = cv.bitwise_and(img_smooth, img_smooth, mask=mask)

    # Draw ROI guide
    cv.rectangle(frame, (50, 50), (350, 350), (0, 0, 255), 2)
    cv.line(frame, (150, 50), (150, 350), (0, 0, 255), 1)
    cv.line(frame, (250, 50), (250, 350), (0, 0, 255), 1)
    cv.line(frame, (50, 150), (350, 150), (0, 0, 255), 1)
    cv.line(frame, (50, 250), (350, 250), (0, 0, 255), 1)

    # Convert ROI to HSV for color detection
    img_hsv = cv.cvtColor(img_roi, cv.COLOR_BGR2HSV)

    # --- Detect yellow object ---
    img_threshold = cv.inRange(img_hsv, lower_yellow, upper_yellow)
    contours, _ = cv.findContours(img_threshold, cv.RETR_TREE, cv.CHAIN_APPROX_NONE)

    if contours:
        cnt = max(contours, key=cv.contourArea)
        M = cv.moments(cnt)
        if M['m00'] != 0:
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            cv.circle(frame, (cx, cy), 4, (0, 255, 0), -1)

            # Cursor movement
            dist_x = -15 if cx < 150 else 15 if cx > 250 else 0
            dist_y = -15 if cy < 150 else 15 if cy > 250 else 0
            if dist_x != 0 or dist_y != 0:
                pyautogui.moveRel(dist_x, dist_y, duration=0.02)

    # --- Detect green object for click ---
    img_threshold_green = cv.inRange(img_hsv, lower_green, upper_green)
    contours_green, _ = cv.findContours(img_threshold_green, cv.RETR_TREE, cv.CHAIN_APPROX_NONE)
    if contours_green:
        pyautogui.click()

    # Show video feed
    cv.imshow('video', frame)

    # Exit on ESC key
    if cv.waitKey(1) & 0xFF == 27:
        break

# Cleanup
cam.release()
cv.destroyAllWindows()
