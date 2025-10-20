#!/usr/bin/env python3
"""
Direct Vilib to OpenVSLAM Bridge
Saves frames as individual images that can be processed by run_image_slam
"""

import cv2
import numpy as np
import time
import sys
import os
import argparse
from pathlib import Path

# Add vilib to path if not installed system-wide
if os.path.exists('/home/dan/vilib'):
    sys.path.insert(0, '/home/dan/vilib')

from vilib import Vilib


class VilibImageSequence:
    def __init__(self, output_dir='/tmp/vilib_frames', grayscale=True,
                 width=640, height=480, fps=30):
        self.output_dir = Path(output_dir)
        self.grayscale = grayscale
        self.target_width = width
        self.target_height = height
        self.fps = fps
        self.frame_count = 0

        # Create output directory
        self.output_dir.mkdir(parents=True, exist_ok=True)

        # Clear old frames
        for old_file in self.output_dir.glob("*.png"):
            old_file.unlink()

    def start_camera(self):
        """Initialize Vilib camera"""
        print("Initializing Vilib camera...")
        Vilib.camera_start(vflip=False, hflip=False)
        Vilib.display(local=False, web=False)
        time.sleep(2)

        # Wait for first frame
        while Vilib.img is None:
            print("Waiting for camera...")
            time.sleep(0.1)

        print("Camera initialized successfully!")

    def process_frame(self, img):
        """Process frame from Vilib - convert to grayscale and resize"""
        if img is None:
            return None

        # Resize if needed
        if img.shape[1] != self.target_width or img.shape[0] != self.target_height:
            img = cv2.resize(img, (self.target_width, self.target_height))

        # Convert to grayscale if requested
        if self.grayscale:
            if len(img.shape) == 3:
                img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        return img

    def run(self):
        """Main capture loop"""
        print("\n" + "="*60)
        print("Vilib Image Sequence Capture - GRAYSCALE MODE" if self.grayscale else "Vilib Image Sequence Capture")
        print("="*60)
        print(f"Output directory: {self.output_dir}")
        print(f"Resolution: {self.target_width}x{self.target_height}")
        print(f"FPS: {self.fps}")
        print(f"Grayscale: {self.grayscale}")
        print("\nCapturing frames... Press Ctrl+C to stop")
        print("="*60 + "\n")

        self.start_camera()

        # Create timestamps file
        timestamps_file = self.output_dir / "timestamps.txt"
        timestamps_fp = open(timestamps_file, 'w')

        start_time = time.time()
        frame_interval = 1.0 / self.fps

        try:
            while True:
                frame_start = time.time()

                img = Vilib.img

                if img is None:
                    time.sleep(0.01)
                    continue

                # Process frame
                processed = self.process_frame(img)

                if processed is not None:
                    # Save frame
                    frame_filename = f"frame_{self.frame_count:06d}.png"
                    frame_path = self.output_dir / frame_filename
                    cv2.imwrite(str(frame_path), processed)

                    # Write timestamp
                    timestamp = time.time() - start_time
                    timestamps_fp.write(f"{timestamp:.6f}\n")
                    timestamps_fp.flush()

                    self.frame_count += 1

                    # Status update
                    if self.frame_count % 100 == 0:
                        elapsed = time.time() - start_time
                        fps = self.frame_count / elapsed
                        print(f"Frames captured: {self.frame_count} | FPS: {fps:.1f}")

                # Rate limiting
                elapsed = time.time() - frame_start
                sleep_time = frame_interval - elapsed
                if sleep_time > 0:
                    time.sleep(sleep_time)

        except KeyboardInterrupt:
            print("\nStopping...")

        finally:
            timestamps_fp.close()
            elapsed = time.time() - start_time
            avg_fps = self.frame_count / elapsed if elapsed > 0 else 0

            print("\n" + "="*60)
            print("Capture Complete")
            print("="*60)
            print(f"Total frames captured: {self.frame_count}")
            print(f"Total time: {elapsed:.1f}s")
            print(f"Average FPS: {avg_fps:.1f}")
            print(f"Output directory: {self.output_dir}")
            print(f"Timestamps file: {timestamps_file}")
            print("\nTo process with SLAM, run:")
            print(f"\n  run_image_slam \\")
            print(f"    -v ~/vocab/orb_vocab.fbow \\")
            print(f"    -d {self.output_dir} \\")
            print(f"    -c ~/Documents/openvslam/pi_camera.yaml")
            print("="*60)

            Vilib.camera_close()


def main():
    parser = argparse.ArgumentParser(
        description='Vilib Image Sequence Capture - Save frames for SLAM processing'
    )
    parser.add_argument('-o', '--output', default='/tmp/vilib_frames',
                       help='Output directory for frames')
    parser.add_argument('-f', '--fps', type=int, default=30,
                       help='Frames per second (default: 30)')
    parser.add_argument('-g', '--grayscale', action='store_true',
                       help='Capture in grayscale mode')
    parser.add_argument('-W', '--width', type=int, default=640,
                       help='Frame width (default: 640)')
    parser.add_argument('-H', '--height', type=int, default=480,
                       help='Frame height (default: 480)')

    args = parser.parse_args()

    capturer = VilibImageSequence(
        output_dir=args.output,
        grayscale=args.grayscale,
        width=args.width,
        height=args.height,
        fps=args.fps
    )

    capturer.run()


if __name__ == "__main__":
    main()
