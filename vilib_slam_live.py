#!/usr/bin/env python3
"""
Real-time SLAM using Vilib camera
Directly processes frames from Vilib.img in grayscale
"""

import cv2
import numpy as np
import time
import sys
import os

# Add vilib to path if not installed system-wide
if os.path.exists('/home/dan/vilib'):
    sys.path.insert(0, '/home/dan/vilib')

from vilib import Vilib

# Configuration
GRAYSCALE = True  # Set to True for B&W processing
DISPLAY = True    # Show live view
SAVE_FRAMES = False  # Save frames to disk for later SLAM processing

def main():
    # Initialize Vilib camera
    print("=" * 60)
    print("Vilib Camera for SLAM - Grayscale Mode")
    print("=" * 60)
    print("\nInitializing Vilib camera...")

    Vilib.camera_start(vflip=False, hflip=False)
    Vilib.display(local=False, web=False)

    # Wait for camera to initialize
    time.sleep(2)

    # Get first frame to check resolution
    while Vilib.img is None:
        print("Waiting for camera...")
        time.sleep(0.1)

    img = Vilib.img
    height, width = img.shape[:2]
    print(f"Camera resolution: {width}x{height}")
    print(f"Grayscale mode: {GRAYSCALE}")
    print(f"Display enabled: {DISPLAY}")

    if SAVE_FRAMES:
        os.makedirs("vilib_frames", exist_ok=True)
        print("Saving frames to: vilib_frames/")

    print("\nPress 'q' to quit, 's' to save current frame")
    print("-" * 60)

    frame_count = 0
    start_time = time.time()
    saved_count = 0

    try:
        while True:
            # Get image from Vilib
            img = Vilib.img

            if img is None:
                time.sleep(0.01)
                continue

            # Convert to grayscale if requested
            if GRAYSCALE:
                if len(img.shape) == 3:
                    processed = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
                else:
                    processed = img.copy()
            else:
                processed = img.copy()

            frame_count += 1

            # Display
            if DISPLAY:
                # Add info overlay
                display_img = processed.copy() if GRAYSCALE else processed
                if len(display_img.shape) == 2:
                    display_img = cv2.cvtColor(display_img, cv2.COLOR_GRAY2BGR)

                # Add FPS counter
                elapsed = time.time() - start_time
                fps = frame_count / elapsed if elapsed > 0 else 0
                cv2.putText(display_img, f"FPS: {fps:.1f}", (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                cv2.putText(display_img, f"Frames: {frame_count}", (10, 60),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                cv2.putText(display_img, "Grayscale: ON" if GRAYSCALE else "Color: ON",
                           (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

                cv2.imshow('Vilib SLAM Camera', display_img)

            # Auto-save frames if enabled
            if SAVE_FRAMES and frame_count % 10 == 0:
                filename = f"vilib_frames/frame_{frame_count:06d}.png"
                cv2.imwrite(filename, processed)

            # Handle keyboard input
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('s'):
                # Manual save
                timestamp = int(time.time() * 1000)
                filename = f"saved_frame_{timestamp}.png"
                cv2.imwrite(filename, processed)
                saved_count += 1
                print(f"Saved: {filename}")

            # Status update every 100 frames
            if frame_count % 100 == 0:
                elapsed = time.time() - start_time
                fps = frame_count / elapsed
                print(f"Frames: {frame_count} | FPS: {fps:.1f} | "
                      f"Mode: {'Grayscale' if GRAYSCALE else 'Color'}")

    except KeyboardInterrupt:
        print("\n\nStopping...")

    finally:
        elapsed = time.time() - start_time
        avg_fps = frame_count / elapsed if elapsed > 0 else 0

        print("\n" + "=" * 60)
        print("Session Summary")
        print("=" * 60)
        print(f"Total frames captured: {frame_count}")
        print(f"Total time: {elapsed:.1f} seconds")
        print(f"Average FPS: {avg_fps:.1f}")
        print(f"Manually saved frames: {saved_count}")
        if SAVE_FRAMES:
            print(f"Auto-saved frames: {frame_count // 10}")
        print("=" * 60)

        Vilib.camera_close()
        cv2.destroyAllWindows()
        print("\nCamera closed successfully.")

if __name__ == "__main__":
    main()
