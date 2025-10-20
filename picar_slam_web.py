#!/usr/bin/env python3
"""
Web-based Remote Control for PiCar SLAM
Access from browser at http://PICAR_IP:5001

Features:
- View live camera feed (from Vilib web stream)
- Control robot with keyboard or on-screen buttons
- Start/Stop SLAM recording
- Download recorded sessions
"""

import sys
import os

# Add paths
sys.path.insert(0, '/home/dan/Nevil-picar-v3/v1.0')
sys.path.insert(0, '/home/dan/vilib')

from robot_hat import reset_mcu
from picarlibs.picarx import Picarx
from vilib import Vilib
from time import sleep, time, strftime, localtime
import cv2
from flask import Flask, render_template_string, jsonify, request
import threading
import readchar
import select

# Initialize hardware
# Note: reset_mcu() requires sudo. Run this script with sudo if you get GPIO errors.
try:
    reset_mcu()
    sleep(0.2)
except Exception as e:
    print(f"Warning: reset_mcu failed ({e}). Continuing anyway...")
    print("If robot controls don't work, run with: sudo python3 picar_slam_web.py")

app = Flask(__name__)
px = Picarx()

# State variables
class RobotState:
    def __init__(self):
        self.speed = 0
        self.status = 'stop'
        self.recording = False
        self.frame_count = 0
        self.session_start_time = None
        self.output_dir = None
        self.timestamps_file = None

state = RobotState()

HTML_TEMPLATE = """
<!DOCTYPE html>
<html>
<head>
    <title>PiCar SLAM Remote Control</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
        body {
            font-family: Arial, sans-serif;
            max-width: 1200px;
            margin: 0 auto;
            padding: 20px;
            background: #f0f0f0;
            outline: none;
        }
        body:focus {
            outline: none;
        }
        .container {
            background: white;
            border-radius: 10px;
            padding: 20px;
            margin-bottom: 20px;
            box-shadow: 0 2px 5px rgba(0,0,0,0.1);
        }
        h1 {
            color: #333;
            text-align: center;
        }
        .status-bar {
            display: flex;
            justify-content: space-around;
            padding: 15px;
            background: #e8f4f8;
            border-radius: 5px;
            margin-bottom: 20px;
        }
        .status-item {
            text-align: center;
        }
        .status-label {
            font-size: 12px;
            color: #666;
        }
        .status-value {
            font-size: 24px;
            font-weight: bold;
            color: #333;
        }
        .recording {
            color: red;
            animation: blink 1s infinite;
        }
        @keyframes blink {
            0%, 50% { opacity: 1; }
            51%, 100% { opacity: 0.3; }
        }
        .video-container {
            text-align: center;
            margin-bottom: 20px;
        }
        #camera-feed {
            max-width: 100%;
            border-radius: 5px;
            box-shadow: 0 2px 10px rgba(0,0,0,0.2);
        }
        .controls {
            display: grid;
            grid-template-columns: repeat(3, 1fr);
            gap: 10px;
            max-width: 400px;
            margin: 0 auto 20px;
        }
        .btn {
            padding: 15px;
            font-size: 16px;
            border: none;
            border-radius: 5px;
            cursor: pointer;
            font-weight: bold;
            transition: all 0.2s;
        }
        .btn:active {
            transform: scale(0.95);
        }
        .btn-forward { grid-column: 2; background: #4CAF50; color: white; }
        .btn-left { grid-column: 1; grid-row: 2; background: #2196F3; color: white; }
        .btn-stop { grid-column: 2; grid-row: 2; background: #f44336; color: white; }
        .btn-right { grid-column: 3; grid-row: 2; background: #2196F3; color: white; }
        .btn-backward { grid-column: 2; grid-row: 3; background: #FF9800; color: white; }
        .speed-controls {
            display: flex;
            justify-content: center;
            gap: 10px;
            margin: 20px 0;
        }
        .btn-speed {
            padding: 10px 20px;
            background: #9C27B0;
            color: white;
        }
        .slam-controls {
            display: flex;
            justify-content: center;
            gap: 10px;
            margin-top: 20px;
        }
        .btn-record {
            padding: 15px 30px;
            background: #FF5722;
            color: white;
            font-size: 18px;
        }
        .btn-record.recording {
            background: #d32f2f;
            animation: pulse 1s infinite;
        }
        @keyframes pulse {
            0%, 100% { transform: scale(1); }
            50% { transform: scale(1.05); }
        }
        .instructions {
            background: #fff3cd;
            padding: 15px;
            border-radius: 5px;
            margin-top: 20px;
        }
        .instructions h3 {
            margin-top: 0;
            color: #856404;
        }
        .instructions ul {
            margin: 10px 0;
            padding-left: 20px;
        }
    </style>
</head>
<body>
    <h1>🚗 PiCar SLAM Remote Control</h1>

    <div class="container">
        <div class="status-bar">
            <div class="status-item">
                <div class="status-label">Status</div>
                <div class="status-value" id="status">{{ status }}</div>
            </div>
            <div class="status-item">
                <div class="status-label">Speed</div>
                <div class="status-value" id="speed">{{ speed }}</div>
            </div>
            <div class="status-item">
                <div class="status-label">Recording</div>
                <div class="status-value" id="recording" style="color: gray;">⚫</div>
            </div>
            <div class="status-item">
                <div class="status-label">Frames</div>
                <div class="status-value" id="frames">{{ frames }}</div>
            </div>
        </div>

        <div class="video-container">
            <img id="camera-feed" src="http://{{ camera_host }}:9000/mjpg" alt="Camera Feed">
        </div>

        <div class="controls">
            <button class="btn btn-forward" onmousedown="sendCommand('forward')" onmouseup="sendCommand('stop')">↑<br>FORWARD</button>
            <button class="btn btn-left" onmousedown="sendCommand('left')" onmouseup="sendCommand('stop')">←<br>LEFT</button>
            <button class="btn btn-stop" onclick="sendCommand('stop')">⬛<br>STOP</button>
            <button class="btn btn-right" onmousedown="sendCommand('right')" onmouseup="sendCommand('stop')">→<br>RIGHT</button>
            <button class="btn btn-backward" onmousedown="sendCommand('backward')" onmouseup="sendCommand('stop')">↓<br>BACKWARD</button>
        </div>

        <div class="speed-controls">
            <button class="btn btn-speed" onclick="sendCommand('speed_down')">- SLOWER</button>
            <button class="btn btn-speed" onclick="sendCommand('speed_up')">+ FASTER</button>
        </div>

        <div class="slam-controls">
            <button class="btn btn-record" id="record-btn" onclick="toggleRecording()">
                🔴 START RECORDING
            </button>
        </div>
    </div>

    <div class="container instructions">
        <h3>📝 Instructions</h3>
        <ul>
            <li><strong>Camera:</strong> Live feed shows above (from port 9000)</li>
            <li><strong>Drive:</strong> Use arrow buttons or keyboard <span style="background: #4CAF50; color: white; padding: 2px 6px; border-radius: 3px; font-weight: bold;">W/A/S/D</span></li>
            <li><strong>Speed:</strong> Use +/- buttons or keyboard <span style="background: #9C27B0; color: white; padding: 2px 6px; border-radius: 3px; font-weight: bold;">O/P</span></li>
            <li><strong>Stop:</strong> Press <span style="background: #f44336; color: white; padding: 2px 6px; border-radius: 3px; font-weight: bold;">F</span> or click STOP button</li>
            <li><strong>SLAM:</strong> Press <span style="background: #FF5722; color: white; padding: 2px 6px; border-radius: 3px; font-weight: bold;">R</span> or click button to record</li>
            <li><strong>Processing:</strong> When done, SSH to Pi and run the displayed command</li>
        </ul>
        <p style="background: #e3f2fd; padding: 10px; border-radius: 5px; border-left: 4px solid #2196F3;">
            <strong>💡 Tip:</strong> Keyboard controls work when this browser tab is focused. Just click anywhere on the page and press keys!
        </p>
        <p><strong>SLAM Command (run on Pi after recording):</strong></p>
        <pre id="slam-command" style="background: #f8f9fa; padding: 10px; border-radius: 3px; overflow-x: auto;">
run_image_slam -v ~/vocab/orb_vocab.fbow -d <span id="output-dir">/tmp/picar_slam_TIMESTAMP</span> -c ~/Documents/openvslam/pi_camera_640x480.yaml -o my_map.msg
        </pre>
    </div>

    <script>
        // Update status every 500ms
        setInterval(updateStatus, 500);

        function updateStatus() {
            fetch('/status')
                .then(r => r.json())
                .then(data => {
                    document.getElementById('status').textContent = data.status;
                    document.getElementById('speed').textContent = data.speed;
                    document.getElementById('frames').textContent = data.frames;

                    const recIndicator = document.getElementById('recording');
                    const recBtn = document.getElementById('record-btn');

                    if (data.recording) {
                        recIndicator.innerHTML = '🔴';
                        recIndicator.className = 'status-value recording';
                        recBtn.textContent = '⬛ STOP RECORDING';
                        recBtn.classList.add('recording');
                        document.getElementById('output-dir').textContent = data.output_dir || '/tmp/picar_slam_TIMESTAMP';
                    } else {
                        recIndicator.innerHTML = '⚫';
                        recIndicator.style.color = 'gray';
                        recBtn.textContent = '🔴 START RECORDING';
                        recBtn.classList.remove('recording');
                    }
                });
        }

        function sendCommand(cmd) {
            fetch('/command', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify({command: cmd})
            });
        }

        function toggleRecording() {
            fetch('/toggle_recording', {method: 'POST'})
                .then(r => r.json())
                .then(data => {
                    if (!data.recording && data.output_dir) {
                        alert('Recording stopped!\\n\\nFrames: ' + data.frames + '\\nOutput: ' + data.output_dir + '\\n\\nRun SLAM command shown below.');
                    }
                });
        }

        // Keyboard controls
        let activeKeys = new Set();

        window.addEventListener('keydown', function(e) {
            const key = e.key.toLowerCase();

            console.log('Key pressed:', key);  // Debug

            // Prevent repeated keydown events
            if (activeKeys.has(key)) return;
            activeKeys.add(key);

            let handled = false;

            if (key === 'w') {
                sendCommand('forward');
                handled = true;
            }
            else if (key === 's') {
                sendCommand('backward');
                handled = true;
            }
            else if (key === 'a') {
                sendCommand('left');
                handled = true;
            }
            else if (key === 'd') {
                sendCommand('right');
                handled = true;
            }
            else if (key === 'f') {
                sendCommand('stop');
                handled = true;
            }
            else if (key === 'o') {
                sendCommand('speed_up');
                handled = true;
            }
            else if (key === 'p') {
                sendCommand('speed_down');
                handled = true;
            }
            else if (key === 'r') {
                toggleRecording();
                handled = true;
            }

            if (handled) {
                e.preventDefault();
                e.stopPropagation();
            }
        }, true);

        window.addEventListener('keyup', function(e) {
            const key = e.key.toLowerCase();
            activeKeys.delete(key);

            if (['w', 's', 'a', 'd'].includes(key)) {
                sendCommand('stop');
                e.preventDefault();
            }
        }, true);

        // Ready message
        window.addEventListener('load', function() {
            console.log('✅ Keyboard controls ready! Press W/A/S/D to drive.');
        });
    </script>
</body>
</html>
"""

def move(operate: str, speed: int):
    """Control PiCar movement"""
    if operate == 'stop':
        px.stop()
    elif operate == 'forward':
        px.set_dir_servo_angle(0)
        px.forward(speed)
    elif operate == 'backward':
        px.set_dir_servo_angle(0)
        px.backward(speed)
    elif operate == 'left':
        px.set_dir_servo_angle(-30)
        px.forward(speed)
    elif operate == 'right':
        px.set_dir_servo_angle(30)
        px.forward(speed)

def save_frame_worker():
    """Background thread to save frames"""
    while True:
        if state.recording:
            img = Vilib.img
            if img is not None:
                # Convert to grayscale
                if len(img.shape) == 3:
                    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
                else:
                    gray = img

                # Resize to 640x480
                if gray.shape != (480, 640):
                    gray = cv2.resize(gray, (640, 480))

                # Save frame
                frame_path = f"{state.output_dir}/frame_{state.frame_count:06d}.png"
                cv2.imwrite(frame_path, gray)

                # Write timestamp
                timestamp = time() - state.session_start_time
                state.timestamps_file.write(f"{timestamp:.6f}\n")
                state.timestamps_file.flush()

                state.frame_count += 1

        sleep(1.0 / 30)  # 30 FPS

def keyboard_worker():
    """Background thread for keyboard controls"""
    import sys
    print("\n[Keyboard] Terminal keyboard controls active!")
    print("           W/S=fwd/back, A/D=left/right, F=stop")
    print("           O/P=speed up/down, R=record, Ctrl+C=quit\n")

    while True:
        try:
            # Non-blocking check for keyboard input
            if select.select([sys.stdin], [], [], 0.1)[0]:
                key = readchar.readkey().lower()

                # Movement controls
                if key == 'w':
                    if state.speed == 0:
                        state.speed = 10
                    if state.status != 'forward' and state.speed > 60:
                        state.speed = 60
                    state.status = 'forward'
                    move(state.status, state.speed)
                elif key == 's':
                    if state.speed == 0:
                        state.speed = 10
                    if state.status != 'backward' and state.speed > 60:
                        state.speed = 60
                    state.status = 'backward'
                    move(state.status, state.speed)
                elif key == 'a':
                    if state.speed == 0:
                        state.speed = 10
                    state.status = 'left'
                    move(state.status, state.speed)
                elif key == 'd':
                    if state.speed == 0:
                        state.speed = 10
                    state.status = 'right'
                    move(state.status, state.speed)
                elif key == 'f':
                    state.status = 'stop'
                    move(state.status, state.speed)

                # Speed controls
                elif key == 'o':
                    if state.speed <= 90:
                        state.speed += 10
                        print(f"\n[Keyboard] Speed: {state.speed}")
                elif key == 'p':
                    if state.speed >= 10:
                        state.speed -= 10
                    if state.speed == 0:
                        state.status = 'stop'
                    print(f"\n[Keyboard] Speed: {state.speed}")

                # SLAM recording
                elif key == 'r':
                    if not state.recording:
                        state.recording = True
                        state.frame_count = 0
                        state.session_start_time = time()
                        state.output_dir = f"/tmp/picar_slam_{strftime('%Y%m%d_%H%M%S', localtime())}"
                        os.makedirs(state.output_dir, exist_ok=True)
                        state.timestamps_file = open(f"{state.output_dir}/timestamps.txt", 'w')
                        print(f"\n[Keyboard] Recording started: {state.output_dir}")
                    else:
                        state.recording = False
                        if state.timestamps_file:
                            state.timestamps_file.close()
                        print(f"\n[Keyboard] Recording stopped. Frames: {state.frame_count}")
                        print(f"[Keyboard] Output: {state.output_dir}")
        except:
            pass  # Ignore errors in keyboard thread

        sleep(0.05)

@app.route('/')
def index():
    camera_host = request.host.split(':')[0]  # Get Pi's IP
    return render_template_string(
        HTML_TEMPLATE,
        status=state.status,
        speed=state.speed,
        frames=state.frame_count,
        camera_host=camera_host
    )

@app.route('/status')
def get_status():
    return jsonify({
        'status': state.status,
        'speed': state.speed,
        'recording': state.recording,
        'frames': state.frame_count,
        'output_dir': state.output_dir
    })

@app.route('/command', methods=['POST'])
def command():
    data = request.json
    cmd = data.get('command')

    if cmd == 'forward':
        if state.speed == 0:
            state.speed = 10
        if state.status != 'forward' and state.speed > 60:
            state.speed = 60
        state.status = 'forward'
    elif cmd == 'backward':
        if state.speed == 0:
            state.speed = 10
        if state.status != 'backward' and state.speed > 60:
            state.speed = 60
        state.status = 'backward'
    elif cmd == 'left':
        if state.speed == 0:
            state.speed = 10
        state.status = 'left'
    elif cmd == 'right':
        if state.speed == 0:
            state.speed = 10
        state.status = 'right'
    elif cmd == 'stop':
        state.status = 'stop'
    elif cmd == 'speed_up':
        if state.speed <= 90:
            state.speed += 10
    elif cmd == 'speed_down':
        if state.speed >= 10:
            state.speed -= 10
        if state.speed == 0:
            state.status = 'stop'

    move(state.status, state.speed)
    return jsonify({'ok': True})

@app.route('/toggle_recording', methods=['POST'])
def toggle_recording():
    if not state.recording:
        # Start recording
        state.recording = True
        state.frame_count = 0
        state.session_start_time = time()
        state.output_dir = f"/tmp/picar_slam_{strftime('%Y%m%d_%H%M%S', localtime())}"
        os.makedirs(state.output_dir, exist_ok=True)
        state.timestamps_file = open(f"{state.output_dir}/timestamps.txt", 'w')
        print(f"\n[SLAM] Recording started: {state.output_dir}")
    else:
        # Stop recording
        state.recording = False
        if state.timestamps_file:
            state.timestamps_file.close()
        print(f"\n[SLAM] Recording stopped. Frames: {state.frame_count}")
        print(f"[SLAM] Output: {state.output_dir}")

    return jsonify({
        'recording': state.recording,
        'frames': state.frame_count,
        'output_dir': state.output_dir
    })

def main():
    print("Initializing PiCar SLAM Web Control...")
    print("Starting camera...")
    Vilib.camera_start(vflip=False, hflip=False)
    Vilib.display(local=False, web=True)
    sleep(2)
    print("Camera ready!")

    # Start frame saving thread
    frame_thread = threading.Thread(target=save_frame_worker, daemon=True)
    frame_thread.start()

    # Start keyboard control thread
    keyboard_thread = threading.Thread(target=keyboard_worker, daemon=True)
    keyboard_thread.start()

    print("\n" + "="*60)
    print("PiCar SLAM Web Control Ready!")
    print("="*60)
    print(f"\n🌐 Web Control:")
    print(f"   Browser: http://10.0.0.162:5001")
    print(f"   Camera:  http://10.0.0.162:9000/mjpg")
    print(f"\n⌨️  Keyboard Control:")
    print(f"   W/A/S/D - Move, F - Stop, O/P - Speed")
    print(f"   R - Start/Stop Recording")
    print(f"\nPress Ctrl+C to quit")
    print("="*60)

    try:
        app.run(host='0.0.0.0', port=5001, debug=False, use_reloader=False)
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        px.stop()
        Vilib.camera_close()

if __name__ == "__main__":
    main()
