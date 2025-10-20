#!/usr/bin/env python3
"""
Run Stella VSLAM with Vilib camera input
Gets frames from Vilib.img and feeds them to the SLAM system
"""

import cv2
import numpy as np
import time
from vilib import Vilib

# Initialize Vilib camera
print("Initializing Vilib camera...")
Vilib.camera_start(vflip=False, hflip=False)
Vilib.display(local=True, web=True)

# Wait for camera to initialize
time.sleep(2)

print("Camera initialized. Starting frame capture...")
print("Press 'q' to quit")

frame_count = 0
try:
    while True:
        # Get image from Vilib
        img = Vilib.img

        if img is None:
            print("Waiting for camera frame...")
            time.sleep(0.1)
            continue

        # Convert to grayscale if needed
        if len(img.shape) == 3:
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        else:
            gray = img

        # Display the frame
        cv2.imshow('Vilib Camera - Grayscale', gray)

        frame_count += 1
        if frame_count % 30 == 0:
            print(f"Captured {frame_count} frames, shape: {img.shape}")

        # Check for quit
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

except KeyboardInterrupt:
    print("\nStopping...")

finally:
    print(f"Total frames captured: {frame_count}")
    Vilib.camera_close()
    cv2.destroyAllWindows()
