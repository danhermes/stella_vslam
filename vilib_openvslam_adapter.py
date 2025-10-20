#!/usr/bin/env python3
"""
Vilib to OpenVSLAM Adapter (Pure Python - No C++ modification needed)

This script creates a virtual video stream from Vilib camera that can be
read by the OpenVSLAM run_camera_slam or run_video_slam executables.

Two modes available:
1. GStreamer pipeline mode (recommended for live streaming)
2. Video file mode (writes to file that SLAM reads)
"""

import cv2
import numpy as np
import time
import sys
import os
import argparse
import subprocess

# Add vilib to path if not installed system-wide
if os.path.exists('/home/dan/vilib'):
    sys.path.insert(0, '/home/dan/vilib')

from vilib import Vilib


class VilibToSLAMAdapter:
    def __init__(self, mode='gstreamer', output_file='/tmp/vilib_slam.avi',
                 fps=30, grayscale=True, width=640, height=480):
        self.mode = mode
        self.output_file = output_file
        self.fps = fps
        self.grayscale = grayscale
        self.target_width = width
        self.target_height = height
        self.frame_count = 0
        self.writer = None

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

    def setup_video_writer(self):
        """Setup video file writer"""
        if self.mode == 'file':
            # Use XVID codec for better compatibility
            fourcc = cv2.VideoWriter_fourcc(*'XVID')
            # Always write as color (3 channels) for compatibility with video readers
            # We'll convert grayscale to BGR before writing
            is_color = True
            self.writer = cv2.VideoWriter(
                self.output_file,
                fourcc,
                self.fps,
                (self.target_width, self.target_height),
                is_color
            )
            if not self.writer.isOpened():
                print("Warning: XVID codec failed, trying MP4V...")
                fourcc = cv2.VideoWriter_fourcc(*'mp4v')
                self.writer = cv2.VideoWriter(
                    self.output_file,
                    fourcc,
                    self.fps,
                    (self.target_width, self.target_height),
                    is_color
                )
            if not self.writer.isOpened():
                raise RuntimeError(f"Failed to create video writer: {self.output_file}")
            print(f"Video writer created: {self.output_file}")

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

    def create_gstreamer_pipeline(self):
        """Create GStreamer pipeline for streaming"""
        # GStreamer pipeline that creates a virtual camera device
        pipeline = (
            f"appsrc ! "
            f"videoconvert ! "
            f"video/x-raw,format=BGR,width={self.target_width},height={self.target_height},framerate={self.fps}/1 ! "
            f"v4l2sink device=/dev/video10"
        )
        return pipeline

    def run_file_mode(self):
        """Run in file mode - write to video file"""
        print("\n" + "="*60)
        print("Vilib to OpenVSLAM Adapter - FILE MODE")
        print("="*60)
        print(f"Output file: {self.output_file}")
        print(f"Resolution: {self.target_width}x{self.target_height}")
        print(f"FPS: {self.fps}")
        print(f"Grayscale: {self.grayscale}")
        print("\nAfter starting, run OpenVSLAM in another terminal:")
        print(f"\n  cd /home/dan/Documents/stella_vslam_examples/build")
        print(f"  ./run_video_slam \\")
        print(f"    -v ~/data/orb_vocab.fbow \\")
        print(f"    -m {self.output_file} \\")
        print(f"    -c ~/Documents/openvslam/pi_camera.yaml")
        print("\nPress Ctrl+C to stop")
        print("="*60 + "\n")

        self.start_camera()
        self.setup_video_writer()

        start_time = time.time()

        try:
            while True:
                img = Vilib.img

                if img is None:
                    time.sleep(0.01)
                    continue

                # Process frame
                processed = self.process_frame(img)

                if processed is not None:
                    # Write to file
                    if self.grayscale:
                        # Convert grayscale to BGR for video writer
                        processed = cv2.cvtColor(processed, cv2.COLOR_GRAY2BGR)

                    self.writer.write(processed)
                    self.frame_count += 1

                    # Status update
                    if self.frame_count % 100 == 0:
                        elapsed = time.time() - start_time
                        fps = self.frame_count / elapsed
                        print(f"Frames written: {self.frame_count} | FPS: {fps:.1f}")

                # Rate limiting
                time.sleep(1.0 / self.fps)

        except KeyboardInterrupt:
            print("\nStopping...")

        finally:
            self.cleanup(start_time)

    def run_udp_mode(self):
        """Run in UDP streaming mode - stream over network"""
        print("\n" + "="*60)
        print("Vilib to OpenVSLAM Adapter - UDP STREAM MODE")
        print("="*60)
        print(f"Resolution: {self.target_width}x{self.target_height}")
        print(f"FPS: {self.fps}")
        print(f"Grayscale: {self.grayscale}")
        print(f"\nStreaming to UDP port 5000")
        print("\nTo receive and process with OpenVSLAM, use:")
        print(f"  gst-launch-1.0 udpsrc port=5000 ! ...")
        print("\nPress Ctrl+C to stop")
        print("="*60 + "\n")

        self.start_camera()

        # GStreamer pipeline for UDP streaming
        gst_pipeline = (
            f"appsrc ! "
            f"videoconvert ! "
            f"video/x-raw,format=I420,width={self.target_width},height={self.target_height} ! "
            f"jpegenc ! "
            f"rtpjpegpay ! "
            f"udpsink host=127.0.0.1 port=5000"
        )

        out = cv2.VideoWriter(gst_pipeline, cv2.CAP_GSTREAMER, 0,
                              self.fps, (self.target_width, self.target_height), True)

        if not out.isOpened():
            print("ERROR: Could not create GStreamer pipeline")
            print("Make sure GStreamer is installed: sudo apt install gstreamer1.0-tools gstreamer1.0-plugins-good")
            return

        start_time = time.time()

        try:
            while True:
                img = Vilib.img

                if img is None:
                    time.sleep(0.01)
                    continue

                processed = self.process_frame(img)

                if processed is not None:
                    if self.grayscale:
                        processed = cv2.cvtColor(processed, cv2.COLOR_GRAY2BGR)

                    out.write(processed)
                    self.frame_count += 1

                    if self.frame_count % 100 == 0:
                        elapsed = time.time() - start_time
                        fps = self.frame_count / elapsed
                        print(f"Frames streamed: {self.frame_count} | FPS: {fps:.1f}")

        except KeyboardInterrupt:
            print("\nStopping...")

        finally:
            out.release()
            self.cleanup(start_time)

    def cleanup(self, start_time=None):
        """Cleanup resources"""
        if self.writer:
            self.writer.release()

        if start_time:
            elapsed = time.time() - start_time
            avg_fps = self.frame_count / elapsed if elapsed > 0 else 0
            print(f"\nTotal frames: {self.frame_count}")
            print(f"Average FPS: {avg_fps:.1f}")
            print(f"Duration: {elapsed:.1f}s")

        Vilib.camera_close()
        print("Camera closed.")


def main():
    parser = argparse.ArgumentParser(
        description='Vilib to OpenVSLAM Adapter - Bridge Vilib camera to SLAM'
    )
    parser.add_argument('-m', '--mode', choices=['file', 'udp'], default='file',
                       help='Streaming mode (default: file)')
    parser.add_argument('-o', '--output', default='/tmp/vilib_slam.avi',
                       help='Output file path for file mode')
    parser.add_argument('-f', '--fps', type=int, default=30,
                       help='Frames per second (default: 30)')
    parser.add_argument('-g', '--grayscale', action='store_true',
                       help='Process in grayscale mode')
    parser.add_argument('-W', '--width', type=int, default=640,
                       help='Frame width (default: 640)')
    parser.add_argument('-H', '--height', type=int, default=480,
                       help='Frame height (default: 480)')

    args = parser.parse_args()

    adapter = VilibToSLAMAdapter(
        mode=args.mode,
        output_file=args.output,
        fps=args.fps,
        grayscale=args.grayscale,
        width=args.width,
        height=args.height
    )

    if args.mode == 'file':
        adapter.run_file_mode()
    elif args.mode == 'udp':
        adapter.run_udp_mode()


if __name__ == "__main__":
    main()
