#!/usr/bin/env python3
"""
Live PiCar SLAM with Keyboard Control
Combines Vilib camera, PiCar controls, and SLAM frame capture
Based on example 11.video_car.py
"""

import sys
import os

# Add paths first before any imports
sys.path.insert(0, '/home/dan/Nevil-picar-v3/v1.0')
sys.path.insert(0, '/home/dan/vilib')

from robot_hat import reset_mcu
from picarlibs.picarx import Picarx
from vilib import Vilib
from time import sleep, time, strftime, localtime
import readchar
import cv2

# Get user info
user = os.getlogin()
user_home = os.path.expanduser(f'~{user}')

# Reset MCU
reset_mcu()
sleep(0.2)

MANUAL = '''
==============================================================
PiCar-X SLAM Live Control
==============================================================
Press keys to control (non-case sensitive):

MOVEMENT:
    W: Forward
    S: Backward
    A: Turn left
    D: Turn right
    F: Stop

SPEED:
    O: Speed up (+10)
    P: Speed down (-10)

SLAM:
    T: Take photo
    R: Start/Stop recording frames for SLAM
    M: Save current SLAM session

    Ctrl+C: Quit

Status will be displayed at bottom.
==============================================================
'''

class PiCarSLAM:
    def __init__(self):
        self.px = Picarx()
        self.speed = 0
        self.status = 'stop'
        self.recording = False
        self.frame_count = 0
        self.session_start_time = None
        self.output_dir = f"/tmp/picar_slam_{strftime('%Y%m%d_%H%M%S', localtime())}"

    def setup_camera(self):
        """Initialize Vilib camera"""
        print("Initializing camera...")
        Vilib.camera_start(vflip=False, hflip=False)
        Vilib.display(local=True, web=True)
        sleep(2)
        print("Camera ready!")

    def take_photo(self):
        """Take a single photo"""
        _time = strftime('%Y-%m-%d-%H-%M-%S', localtime(time()))
        name = f'picar_slam_photo_{_time}'
        path = f"{user_home}/Pictures/picar-x/"
        Vilib.take_photo(name, path)
        print(f'\nPhoto saved: {path}{name}.jpg')

    def toggle_recording(self):
        """Start/stop SLAM frame recording"""
        if not self.recording:
            # Start recording
            self.recording = True
            self.frame_count = 0
            self.session_start_time = time()
            self.output_dir = f"/tmp/picar_slam_{strftime('%Y%m%d_%H%M%S', localtime())}"
            os.makedirs(self.output_dir, exist_ok=True)

            # Create timestamps file
            self.timestamps_file = open(f"{self.output_dir}/timestamps.txt", 'w')
            print(f'\n[SLAM] Recording started: {self.output_dir}')
        else:
            # Stop recording
            self.recording = False
            if hasattr(self, 'timestamps_file'):
                self.timestamps_file.close()
            print(f'\n[SLAM] Recording stopped. Frames: {self.frame_count}')
            print(f'[SLAM] Output: {self.output_dir}')
            print(f'\n[SLAM] To process with SLAM, run:')
            print(f'  run_image_slam \\')
            print(f'    -v ~/vocab/orb_vocab.fbow \\')
            print(f'    -d {self.output_dir} \\')
            print(f'    -c ~/Documents/openvslam/pi_camera_640x480.yaml')

    def save_frame(self):
        """Save current frame if recording"""
        if not self.recording:
            return

        img = Vilib.img
        if img is None:
            return

        # Convert to grayscale
        if len(img.shape) == 3:
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        else:
            gray = img

        # Resize to 640x480 if needed
        if gray.shape != (480, 640):
            gray = cv2.resize(gray, (640, 480))

        # Save frame
        frame_path = f"{self.output_dir}/frame_{self.frame_count:06d}.png"
        cv2.imwrite(frame_path, gray)

        # Write timestamp
        timestamp = time() - self.session_start_time
        self.timestamps_file.write(f"{timestamp:.6f}\n")
        self.timestamps_file.flush()

        self.frame_count += 1

    def move(self, operate: str, speed: int):
        """Control PiCar movement"""
        if operate == 'stop':
            self.px.stop()
        else:
            if operate == 'forward':
                self.px.set_dir_servo_angle(0)
                self.px.forward(speed)
            elif operate == 'backward':
                self.px.set_dir_servo_angle(0)
                self.px.backward(speed)
            elif operate == 'turn left':
                self.px.set_dir_servo_angle(-30)
                self.px.forward(speed)
            elif operate == 'turn right':
                self.px.set_dir_servo_angle(30)
                self.px.forward(speed)

    def run(self):
        """Main control loop"""
        self.setup_camera()
        print(MANUAL)

        last_frame_time = time()
        frame_interval = 1.0 / 30  # 30 FPS

        try:
            while True:
                # Display status
                rec_status = "🔴 REC" if self.recording else "⚫ IDLE"
                print(f"\r{rec_status} | Status: {self.status:12s} | Speed: {self.speed:3d} | Frames: {self.frame_count:5d}    ",
                      end='', flush=True)

                # Save frame if recording (at 30 FPS)
                current_time = time()
                if self.recording and (current_time - last_frame_time) >= frame_interval:
                    self.save_frame()
                    last_frame_time = current_time

                # Read keyboard (non-blocking with timeout)
                key = readchar.readkey()
                key = key.lower() if isinstance(key, str) else key

                # Handle controls
                if key in ('wsadfop'):
                    # Speed control
                    if key == 'o':
                        if self.speed <= 90:
                            self.speed += 10
                    elif key == 'p':
                        if self.speed >= 10:
                            self.speed -= 10
                        if self.speed == 0:
                            self.status = 'stop'

                    # Direction control
                    elif key in ('wsad'):
                        if self.speed == 0:
                            self.speed = 10

                        if key == 'w':
                            # Speed limit when changing direction
                            if self.status != 'forward' and self.speed > 60:
                                self.speed = 60
                            self.status = 'forward'
                        elif key == 'a':
                            self.status = 'turn left'
                        elif key == 's':
                            if self.status != 'backward' and self.speed > 60:
                                self.speed = 60
                            self.status = 'backward'
                        elif key == 'd':
                            self.status = 'turn right'

                    # Stop
                    elif key == 'f':
                        self.status = 'stop'

                    # Execute movement
                    self.move(self.status, self.speed)

                # SLAM controls
                elif key == 't':
                    self.take_photo()
                elif key == 'r':
                    self.toggle_recording()
                elif key == 'm':
                    if self.recording:
                        print('\n[SLAM] Stop recording first (press R)')
                    elif self.frame_count > 0:
                        print(f'\n[SLAM] Session saved: {self.output_dir}')
                        print(f'[SLAM] Total frames: {self.frame_count}')
                    else:
                        print('\n[SLAM] No frames recorded yet')

                # Quit
                elif key == readchar.key.CTRL_C:
                    print('\n\nShutting down...')
                    break

                sleep(0.05)  # Small delay

        except KeyboardInterrupt:
            print('\n\nInterrupted by user')
        finally:
            self.cleanup()

    def cleanup(self):
        """Clean shutdown"""
        if self.recording:
            self.toggle_recording()
        self.px.stop()
        Vilib.camera_close()
        print('Shutdown complete.')


if __name__ == "__main__":
    slam_car = PiCarSLAM()
    try:
        slam_car.run()
    except Exception as e:
        print(f"\nError: {e}")
        import traceback
        traceback.print_exc()
    finally:
        slam_car.cleanup()
