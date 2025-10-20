# Vilib Camera SLAM - Quick Start Guide

## Run SLAM with Vilib Camera in Grayscale (B&W)

### Prerequisites
- Raspberry Pi with camera connected
- Vilib installed (automatically installed from `/home/dan/vilib`)
- OpenVSLAM installed system-wide
- ORB vocabulary file downloaded

---

## Step 1: Download ORB Vocabulary (One-time setup)

```bash
mkdir -p ~/vocab
cd ~/vocab
wget https://github.com/stella-cv/FBoW_orb_vocab/raw/main/orb_vocab.fbow
```

---

## Step 2: Capture Frames from Vilib Camera

Run this command and **let it capture for 10-20 seconds**, then press **Ctrl+C**:

```bash
cd /home/dan/Documents/openvslam
python3 vilib_direct_slam.py --grayscale --width 640 --height 480
```

**What this does:**
- Captures frames from `Vilib.img` (your Raspberry Pi camera)
- Converts to grayscale (B&W)
- Saves each frame as PNG in `/tmp/vilib_frames/`
- Creates timestamps file

**You should see:**
```
Initializing Vilib camera...
Camera initialized successfully!

============================================================
Vilib Image Sequence Capture - GRAYSCALE MODE
============================================================
Output directory: /tmp/vilib_frames
Resolution: 640x480
Grayscale: True

Capturing frames... Press Ctrl+C to stop
Frames captured: 100 | FPS: 29.8
Frames captured: 200 | FPS: 29.5
```

**Important:**
- Capture at least 300-600 frames (10-20 seconds)
- Move the camera smoothly around your environment
- Aim at textured surfaces (not blank walls)
- Press **Ctrl+C** when done capturing

---

## Step 3: Run SLAM on the Captured Frames

After stopping the capture (Ctrl+C), run:

```bash
run_image_slam \
  -v ~/vocab/orb_vocab.fbow \
  -d /tmp/vilib_frames \
  -c ~/Documents/openvslam/pi_camera_640x480.yaml
```

**What this does:**
- Reads the saved PNG frames from disk
- Performs SLAM in grayscale
- Shows 3D map visualization
- Tracks camera position

**You should see:**
- SLAM viewer window with camera trajectory
- Feature points being tracked
- Map being built in real-time
- Console output showing tracking status

---

## Stopping

**To stop SLAM:**
- Press `Ctrl+C` or close the viewer window

**To capture new footage:**
- Run Step 2 again (it will overwrite old frames)

---

## Common Options

### Adapter Options

**Different resolution:**
```bash
python3 vilib_openvslam_adapter.py --grayscale --width 1280 --height 720
```

**Lower FPS (better performance on slower devices):**
```bash
python3 vilib_openvslam_adapter.py --grayscale --fps 15
```

**Custom output file:**
```bash
python3 vilib_openvslam_adapter.py --grayscale -o /tmp/my_slam.avi
```

**See all options:**
```bash
python3 vilib_openvslam_adapter.py --help
```

### SLAM Options

**Save the map (add -o flag):**
```bash
run_image_slam \
  -v ~/vocab/orb_vocab.fbow \
  -d /tmp/picar_slam_TIMESTAMP \
  -c ~/Documents/openvslam/pi_camera_640x480.yaml \
  -o living_room.msg

# Or with descriptive names and timestamp:
-o ~/slam_maps/living_room_$(date +%Y%m%d_%H%M).msg
```

**Load and use an existing map (localization only):**
```bash
# Capture new footage, then:
run_image_slam \
  -v ~/vocab/orb_vocab.fbow \
  -d /tmp/picar_slam_TIMESTAMP \
  -c ~/Documents/openvslam/pi_camera_640x480.yaml \
  -i living_room.msg \
  --disable-mapping
```
This loads the saved map and only localizes (finds position) without building new map.

**No viewer (headless mode for SSH):**
```bash
run_image_slam \
  -v ~/vocab/orb_vocab.fbow \
  -d /tmp/vilib_frames \
  -c ~/Documents/openvslam/pi_camera_640x480.yaml \
  --viewer none
```

**Higher resolution (capture at 1280x720 and use original config):**
```bash
# Capture at higher resolution
python3 vilib_direct_slam.py --grayscale --width 1280 --height 720

# Process with original config
run_image_slam \
  -v ~/vocab/orb_vocab.fbow \
  -d /tmp/vilib_frames \
  -c ~/Documents/openvslam/pi_camera.yaml
```

---

## Testing Just the Camera

If you want to test the camera feed before running SLAM:

```bash
python3 vilib_slam_live.py
```

This shows:
- Live grayscale feed from Vilib
- FPS counter
- Frame statistics
- Press `q` to quit, `s` to save a frame

---

## How It Works (Technical Details)

1. **Vilib** captures frames from Raspberry Pi camera
2. **Python capture script** reads from `Vilib.img`
3. Frames are converted to **grayscale** (B&W)
4. Each grayscale frame is saved as a PNG file in `/tmp/vilib_frames/`
5. Timestamps are recorded for each frame
6. **OpenVSLAM** reads the PNG files from disk
7. ORB feature extractor processes in grayscale
8. **All SLAM processing happens in B&W**

### Why Grayscale?
- ORB features work better in grayscale
- Faster processing (less data)
- More robust to lighting changes
- Standard practice for visual SLAM

---

## Troubleshooting

### "cannot import name 'Vilib'"
The adapter automatically adds `/home/dan/vilib` to Python path. If you still see this error:
```bash
cd /home/dan/vilib
sudo python3 install.py
```

### "FFmpeg: Failed to write frame" or "expected 1 channel but got 3"
This was fixed in the latest version of the adapter. Make sure you're using the updated script.

### Camera not detected
Check camera connection:
```bash
libcamera-hello --list-cameras
```

### SLAM tracking fails
Try:
- Better lighting
- Slower camera movement
- More textured environment (avoid blank walls)
- Lower resolution: `--width 320 --height 240`

### Low FPS
- Reduce resolution: `--width 320 --height 240`
- Reduce camera FPS: `--fps 15`
- Close other applications

---

## Working with Multiple Maps

### Understanding Map Files

**SLAM can only load ONE map at a time.** You cannot merge or use multiple `.msg` files simultaneously in stella_vslam.

### Use Cases for Multiple Maps:

**1. Different locations (one map per room/area):**
```bash
# Build separate maps
living_room.msg
kitchen.msg
bedroom.msg

# Use whichever map matches your current location
run_image_slam ... -i living_room.msg --disable-mapping
```

**2. Versions of the same area:**
```bash
# Keep different versions
office_v1.msg        # Initial mapping
office_v2.msg        # After furniture moved
office_final.msg     # Best version

# Use the one that works best
run_image_slam ... -i office_final.msg --disable-mapping
```

**3. Different times/lighting conditions:**
```bash
house_daytime.msg
house_nighttime.msg

# Pick the map that matches current conditions
```

### Multi-Room Strategy:

**Option A: One big map (recommended)**
```bash
# Drive through all rooms in one session
python3 picar_slam_live.py
# (press R, drive through: living room → hallway → kitchen → back to living room)
# (press R to stop)

# Build one unified map
run_image_slam ... -o whole_house.msg

# Use this single map for entire house
run_image_slam ... -i whole_house.msg --disable-mapping
```

**Option B: Separate maps per room**
```bash
# Map living room
python3 picar_slam_live.py  # drive in living room only
run_image_slam ... -o living_room.msg

# Map kitchen
python3 picar_slam_live.py  # drive in kitchen only
run_image_slam ... -o kitchen.msg

# Use appropriate map based on where you are
# (Your code needs to know which room you're in and load that map)
```

### Switching Between Maps:

You **cannot** switch maps while SLAM is running. To use a different map:

```bash
# 1. Stop current SLAM
# 2. Capture new footage
python3 picar_slam_live.py

# 3. Run with different map
run_image_slam ... -i different_room.msg --disable-mapping
```

### Map Management:

```bash
# Organize your maps
mkdir -p ~/slam_maps
ls ~/slam_maps/

living_room_20251019.msg
kitchen_20251019.msg
whole_house_20251020.msg

# List map files with sizes
ls -lh ~/slam_maps/
```

### Important Notes:

- ⚠️ **Cannot merge maps** - stella_vslam doesn't support combining multiple .msg files
- ⚠️ **One map at a time** - use `-i` to load ONE map file only
- ✅ **Build large maps** - better to create one map covering multiple rooms than many small maps
- ✅ **Keep backups** - save multiple versions as you iterate

---

## Files Created by This Setup

**Adapter scripts:**
- `/home/dan/Documents/openvslam/vilib_openvslam_adapter.py` - Main adapter
- `/home/dan/Documents/openvslam/vilib_slam_live.py` - Camera test tool
- `/home/dan/Documents/openvslam/run_vilib_slam.py` - Simple test

**Runtime files:**
- `/tmp/vilib_slam.avi` - Video stream file (temporary)

**Config files:**
- `/home/dan/Documents/openvslam/pi_camera.yaml` - Raspberry Pi camera config
- `/home/dan/Documents/openvslam/pi_camera_small.yaml` - Lower resolution config

---

## Next Steps

Once SLAM is working:

1. **Save maps:**
   ```bash
   run_video_slam -v ~/vocab/orb_vocab.fbow -m /tmp/vilib_slam.avi \
     -c ~/Documents/openvslam/pi_camera.yaml -o my_map.msg
   ```

2. **Use saved maps for localization:**
   ```bash
   run_video_slam -v ~/vocab/orb_vocab.fbow -m /tmp/vilib_slam.avi \
     -c ~/Documents/openvslam/pi_camera.yaml -i my_map.msg --disable-mapping
   ```

3. **Integrate with robotics:**
   - Use map for navigation
   - Get camera pose for path planning
   - Export trajectory for analysis

---

## Reference

**Full documentation:** `/home/dan/Documents/openvslam/USAGE_GUIDE.md`

**OpenVSLAM docs:** https://stella-cv.readthedocs.io/

**Camera configs:** `/home/dan/Documents/openvslam/example/`

**Help:**
```bash
run_video_slam --help
python3 vilib_openvslam_adapter.py --help
```
