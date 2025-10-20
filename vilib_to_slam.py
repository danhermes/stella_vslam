#!/usr/bin/env python3
"""
Bridge between Vilib camera and Stella VSLAM
Creates a video stream that can be read by run_video_slam
"""

import cv2
import numpy as np
import time
import sys
from vilib import Vilib

def main():
    # Video output settings
    output_file = '/tmp/vilib_stream.avi'
    fps = 30.0
    width = 640
    height = 480

    # Initialize Vilib camera
    print("Initializing Vilib camera...")
    Vilib.camera_start(vflip=False, hflip=False)
    Vilib.display(local=False, web=False)

    # Wait for camera to initialize
    time.sleep(2)

    # Get first frame to determine size
    while Vilib.img is None:
        print("Waiting for first frame...")
        time.sleep(0.1)

    img = Vilib.img
    height, width = img.shape[:2]
    print(f"Camera resolution: {width}x{height}")

    # Create video writer
    fourcc = cv2.VideoWriter_fourcc(*'MJPG')
    out = cv2.VideoWriter(output_file, fourcc, fps, (width, height), isColor=True)

    if not out.isOpened():
        print(f"Error: Could not create video writer at {output_file}")
        Vilib.camera_close()
        sys.exit(1)

    print(f"Writing video stream to {output_file}")
    print("Press Ctrl+C to stop")
    print(f"\nTo use with SLAM, run in another terminal:")
    print(f"  /home/dan/Documents/stella_vslam_examples/build/run_video_slam \\")
    print(f"    -v ../data/orb_vocab.fbow \\")
    print(f"    -m {output_file} \\")
    print(f"    -c ~/Documents/openvslam/pi_camera.yaml")

    frame_count = 0
    try:
        while True:
            # Get image from Vilib
            img = Vilib.img

            if img is None:
                time.sleep(0.01)
                continue

            # Write frame
            out.write(img)
            frame_count += 1

            if frame_count % 100 == 0:
                print(f"Frames written: {frame_count}")

            # Small delay to maintain fps
            time.sleep(1.0/fps)

    except KeyboardInterrupt:
        print("\nStopping...")

    finally:
        print(f"Total frames written: {frame_count}")
        out.release()
        Vilib.camera_close()

if __name__ == "__main__":
    main()
