# Import required libraries
import cv2 as cv  # OpenCV for video capture & image processing
import mediapipe as mp  # Mediapipe for hand tracking
import numpy as np  # NumPy for mathematical operations
from math import hypot  # For calculating distance between two points
from ctypes import cast, POINTER  # For working with COM objects
from comtypes import CLSCTX_ALL  # COM object context for Windows
from pycaw.pycaw import AudioUtilities, IAudioEndpointVolume  # Pycaw for audio control

# Initialize webcam
cam = cv.VideoCapture(0)  # Capture video from default webcam (index 0)
if not cam.isOpened():  # If camera fails to open
    print("Error: Could not open webcam.")
    exit()  # Stop execution

# Initialize Mediapipe Hands module
mpHands = mp.solutions.hands  # Load the hands module
hands = mpHands.Hands(
    static_image_mode=False,       # False = faster tracking for video stream
    max_num_hands=1,               # Track only 1 hand
    min_detection_confidence=0.7,  # Minimum confidence for detection
    min_tracking_confidence=0.7    # Minimum confidence for tracking
)
mpDraw = mp.solutions.drawing_utils  # Utility for drawing landmarks

# Access the system's audio endpoint (speaker)
devices = AudioUtilities.GetSpeakers()  # Get default audio device
# Activate the audio endpoint interface
interface = devices.Activate(IAudioEndpointVolume._iid_, CLSCTX_ALL, None)
# Cast to IAudioEndpointVolume for controlling volume
volume = cast(interface, POINTER(IAudioEndpointVolume))
# Get volume range (usually min = -63.5 dB, max = 0 dB)
min_Volume, max_Volume = volume.GetVolumeRange()[:2]

# Main loop to capture frames continuously
while True:
    ret, frame = cam.read()  # Read frame from webcam
    if not ret:  # If frame not captured successfully
        print("Error: Failed to grab frame.")
        break  # Exit loop

    frame = cv.flip(frame, 1)  # Flip horizontally for mirror effect
    img_RGB = cv.cvtColor(frame, cv.COLOR_BGR2RGB)  # Convert to RGB (Mediapipe needs RGB)
    results = hands.process(img_RGB)  # Process frame for hand detection

    lmList = []  # List to store landmark positions
    if results.multi_hand_landmarks:  # If hand is detected
        for hand_landmark in results.multi_hand_landmarks:  # Loop through detected hands
            for id, lm in enumerate(hand_landmark.landmark):  # Loop through each landmark
                h, w, c = frame.shape  # Get frame dimensions
                cx, cy = int(lm.x * w), int(lm.y * h)  # Convert normalized coords to pixels
                lmList.append([id, cx, cy])  # Add landmark id & pixel coordinates
            # Draw landmarks and connections on the frame
            mpDraw.draw_landmarks(frame, hand_landmark, mpHands.HAND_CONNECTIONS)

    # If landmarks are detected
    if lmList:
        x1, y1 = lmList[4][1], lmList[4][2]   # Thumb tip coordinates
        x2, y2 = lmList[8][1], lmList[8][2]   # Index finger tip coordinates

        # Draw circles on thumb and index tips
        cv.circle(frame, (x1, y1), 13, (255, 0, 0), -1)
        cv.circle(frame, (x2, y2), 13, (255, 0, 0), -1)
        # Draw line between thumb and index tips
        cv.line(frame, (x1, y1), (x2, y2), (255, 0, 0), 3)

        # Calculate the distance between thumb and index finger tips
        length = hypot(x2 - x1, y2 - y1)

        # Map hand distance range (20–100 pixels) to volume range
        vol = np.interp(length, [20, 100], [min_Volume, max_Volume])
        # Map hand distance to volume bar height
        volbar = np.interp(length, [20, 100], [400, 150])
        # Map hand distance to percentage
        volper = np.interp(length, [20, 100], [0, 100])

        # Set the system volume
        volume.SetMasterVolumeLevel(vol, None)

        # Draw volume bar background
        cv.rectangle(frame, (50, 150), (85, 400), (0, 0, 255), 4)
        # Fill volume bar according to volume level
        cv.rectangle(frame, (50, int(volbar)), (85, 400), (0, 0, 255), -1)
        # Display volume percentage on screen
        cv.putText(frame, f"{int(volper)}%", (10, 40),
                   cv.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 98), 3)

    # Show the processed frame
    cv.imshow('Hand Gesture Volume Control', frame)

    # Wait for key press (1ms), exit if ESC (27) is pressed
    if cv.waitKey(1) & 0xFF == 27:
        break

# Release webcam and close OpenCV windows
cam.release()
cv.destroyAllWindows()
