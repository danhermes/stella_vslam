# stella_vslam - Complete Usage Guide

## 🎥 LIVE CAMERA WITH VILIB (GRAYSCALE MODE)

### Quick Start - Run SLAM with Vilib Camera in B&W

**Step 1: Capture frames from Vilib camera**
```bash
cd /home/dan/Documents/openvslam
python3 vilib_direct_slam.py --grayscale --width 640 --height 480
```
Let it run for **10-20 seconds**, then press **Ctrl+C**

**Step 2: Run SLAM on the captured frames**
```bash
run_image_slam \
  -v ~/vocab/orb_vocab.fbow \
  -d /tmp/vilib_frames \
  -c ~/Documents/openvslam/pi_camera_640x480.yaml
```

### How It Works
1. **Python capture script** reads from `Vilib.img` (Raspberry Pi camera)
2. Converts frames to **grayscale** (B&W)
3. Saves each frame as PNG in `/tmp/vilib_frames/`
4. **OpenVSLAM** reads the PNG files and processes in grayscale
5. **All feature extraction happens in grayscale** for optimal SLAM performance

### Available Vilib Scripts
- `vilib_direct_slam.py` - Capture frames for SLAM (recommended - **WORKING**)
- `vilib_slam_live.py` - Test camera feed in grayscale
- `vilib_openvslam_adapter.py` - Video streaming adapter (experimental)

### Capture Script Options
```bash
python3 vilib_direct_slam.py --help

Options:
  --grayscale           Process in grayscale/B&W mode
  --width WIDTH         Frame width (default: 640)
  --height HEIGHT       Frame height (default: 480)
  --fps FPS             Frames per second (default: 30)
  -o OUTPUT             Output directory for frames
```

**Note:** Use `pi_camera_640x480.yaml` for 640x480 captures, or `pi_camera.yaml` for 1280x720.

---

## ✅ Success! Everything is Installed System-Wide

### 📍 WHERE IS EVERYTHING?

**SLAM Executables (System-Wide - Run from Anywhere):**
```
/usr/local/bin/run_video_slam
/usr/local/bin/run_camera_slam
/usr/local/bin/run_image_slam
/usr/local/bin/run_kitti_slam
/usr/local/bin/run_euroc_slam
/usr/local/bin/run_tum_rgbd_slam
/usr/local/bin/run_loop_closure
```
✅ These work from **any directory** - no need to navigate anywhere!

**SLAM Library:**
```
/usr/local/lib/libstella_vslam.so
/usr/local/include/stella_vslam/
```

**Camera Configuration Files:**
```
360° cameras:    /home/dan/Documents/openvslam/example/aist/equirectangular.yaml
Regular cameras: /home/dan/Documents/openvslam/example/kitti/KITTI_mono_*.yaml
Fisheye:         /home/dan/Documents/openvslam/example/euroc/*.yaml
RGB-D:           /home/dan/Documents/openvslam/example/tum_rgbd/*.yaml
```

**What You Still Need to Download:**
```
ORB Vocabulary: https://github.com/stella-cv/FBoW_orb_vocab/raw/main/orb_vocab.fbow
Save it to:     ~/vocab/orb_vocab.fbow
```

---

## ⚡ Quick Test

```bash
# Verify executables are accessible
which run_video_slam
# Should show: /usr/local/bin/run_video_slam

# Show help
run_video_slam --help
```

---

## 🎬 How to Run with Viewer and Mapping

### Step 1: Download Vocabulary File (One Time Only)

```bash
mkdir -p ~/vocab
cd ~/vocab
wget https://github.com/stella-cv/FBoW_orb_vocab/raw/main/orb_vocab.fbow
```

### Step 2: Run SLAM

## 🎥 Live Camera SLAM (Real-Time)

**For 360° camera:**
```bash
run_camera_slam \
  -v ~/vocab/orb_vocab.fbow \
  -c ~/Documents/openvslam/example/aist/equirectangular.yaml \
  -n 0
```

**For regular camera (webcam, USB camera):**
```bash
run_camera_slam \
  -v ~/vocab/orb_vocab.fbow \
  -c ~/Documents/openvslam/example/kitti/KITTI_mono_00-02.yaml \
  -n 0
```

**For fisheye camera:**
```bash
run_camera_slam \
  -v ~/vocab/orb_vocab.fbow \
  -c ~/Documents/openvslam/example/euroc/EuRoC_mono.yaml \
  -n 0
```

**Note:** Change `-n 0` to 1, 2, 3, etc. if your camera is on a different device.

---

## 📹 Video File SLAM

**For 360° videos:**
```bash
run_video_slam \
  -v ~/vocab/orb_vocab.fbow \
  -m /path/to/your_video.mp4 \
  -c ~/Documents/openvslam/example/aist/equirectangular.yaml
```

**For regular camera videos:**
```bash
run_video_slam \
  -v ~/vocab/orb_vocab.fbow \
  -m /path/to/your_video.mp4 \
  -c ~/Documents/openvslam/example/kitti/KITTI_mono_00-02.yaml
```

**Save the map after processing:**
```bash
run_video_slam \
  -v ~/vocab/orb_vocab.fbow \
  -m /path/to/your_video.mp4 \
  -c ~/Documents/openvslam/example/aist/equirectangular.yaml \
  -o my_map.msg
```

### What You'll See

The viewer will show:
- 🗺️ 3D map points being created
- 📹 Camera trajectory (path the camera took)
- 🖼️ Current video frame
- 📊 Real-time statistics

**Note:** Viewer and mapping are **enabled by default**. The viewer window will appear automatically.

## Available Executables

All the following are ready to use:

- **run_video_slam** - Run SLAM on video files (MP4, etc.)
- **run_camera_slam** - Run SLAM with live camera
- **run_image_slam** - Run SLAM on image sequences
- **run_kitti_slam** - Run SLAM on KITTI dataset
- **run_euroc_slam** - Run SLAM on EuRoC MAV dataset
- **run_tum_rgbd_slam** - Run SLAM on TUM RGB-D dataset
- **run_loop_closure** - Run loop closure detection

---

## Quick Start: run_video_slam

### Command Syntax

```bash
cd ~/Documents/stella_vslam_examples/build

./run_video_slam \
  -v <vocab_file> \
  -m <video_file> \
  -c <config_file> \
  [options]
```

### Required Arguments

1. **`-v, --vocab`** - Vocabulary file path
   - Download from: https://github.com/stella-cv/FBoW_orb_vocab/raw/main/orb_vocab.fbow
   - Or use `--without-vocab` to skip (not recommended)

2. **`-m, --video`** - Video file path (MP4, AVI, etc.)

3. **`-c, --config`** - Camera configuration YAML file
   - Examples in: `/home/dan/Documents/openvslam/example/`

### Optional Arguments

- `--mask arg` - Mask image path
- `--frame-skip arg` - Interval of frame skip (default: 1)
- `-s, --start-time arg` - Start time in milliseconds (default: 0)
- `--no-sleep` - Don't wait for next frame in real time (faster processing)
- `--wait-loop-ba` - Wait until loop BA is finished
- `--auto-term` - Automatically terminate the viewer
- `--log-level arg` - Log level (default: info)
- `--eval-log-dir arg` - Store trajectory and tracking times
- `-i, --map-db-in arg` - Load a pre-built map from this path
- `-o, --map-db-out arg` - Save map database after SLAM
- `--disable-mapping` - Disable mapping (localization-only mode)
- `--temporal-mapping` - Enable temporal mapping
- `-t, --start-timestamp arg` - Timestamp of video start
- `--viewer arg` - Viewer type: iridescence_viewer, pangolin_viewer, socket_publisher, none

---

## Step-by-Step Example

### 1. Download ORB Vocabulary File

```bash
cd ~/Documents/stella_vslam_examples
mkdir -p data
cd data
wget https://github.com/stella-cv/FBoW_orb_vocab/raw/main/orb_vocab.fbow
```

This will download a ~50-80MB file needed for feature matching.

### 2. Prepare Your Video

Place your video file somewhere accessible, e.g.:
```bash
~/Videos/my_video.mp4
```

### 3. Choose/Create Configuration File

**Option A: Use existing config**

For an equirectangular (360°) camera:
```bash
cp ~/Documents/openvslam/example/aist/equirectangular.yaml ~/Documents/stella_vslam_examples/data/my_config.yaml
```

For a perspective camera:
```bash
cp ~/Documents/openvslam/example/kitti/KITTI_mono_00-02.yaml ~/Documents/stella_vslam_examples/data/my_config.yaml
```

**Option B: Edit config for your camera**

Edit the YAML file to match your camera's:
- Resolution (width, height)
- Focal length (fx, fy)
- Principal point (cx, cy)
- Distortion coefficients

### 4. Run SLAM

**Basic run (with viewer):**
```bash
cd ~/Documents/stella_vslam_examples/build

./run_video_slam \
  -v ../data/orb_vocab.fbow \
  -m ~/Videos/my_video.mp4 \
  -c ~/Documents/openvslam/example/aist/equirectangular.yaml
```

**Faster processing (no real-time waiting):**
```bash
./run_video_slam \
  -v ../data/orb_vocab.fbow \
  -m ~/Videos/my_video.mp4 \
  -c ~/Documents/openvslam/example/aist/equirectangular.yaml \
  --frame-skip 3 \
  --no-sleep
```

**Save map for later localization:**
```bash
./run_video_slam \
  -v ../data/orb_vocab.fbow \
  -m ~/Videos/my_video.mp4 \
  -c ~/Documents/openvslam/example/aist/equirectangular.yaml \
  --frame-skip 3 \
  --no-sleep \
  --map-db-out map.msg
```

**Localization-only mode (using pre-built map):**
```bash
./run_video_slam \
  --disable-mapping \
  -v ../data/orb_vocab.fbow \
  -m ~/Videos/my_video_2.mp4 \
  -c ~/Documents/openvslam/example/aist/equirectangular.yaml \
  --frame-skip 3 \
  --no-sleep \
  --map-db-in map.msg
```

---

## Example Configurations Available

### 1. Equirectangular (360° cameras)
- **File**: `~/Documents/openvslam/example/aist/equirectangular.yaml`
- **Use for**: RICOH THETA, Insta360, etc.

### 2. Perspective (Standard cameras)
- **Files**: `~/Documents/openvslam/example/kitti/*.yaml`
- **Use for**: Regular webcams, phone cameras

### 3. Fisheye
- **Files**: `~/Documents/openvslam/example/euroc/*.yaml`
- **Use for**: Wide-angle fisheye lenses

### 4. RGB-D
- **Files**: `~/Documents/openvslam/example/tum_rgbd/*.yaml`
- **Use for**: Depth cameras (RealSense, Kinect, etc.)

---

## Typical Workflow

### 1. Mapping Phase (First Video)
```bash
# Build the map
./run_video_slam \
  -v data/orb_vocab.fbow \
  -m video1.mp4 \
  -c config.yaml \
  --no-sleep \
  -o my_map.msg
```

### 2. Localization Phase (Second Video)
```bash
# Localize in the pre-built map
./run_video_slam \
  --disable-mapping \
  -v data/orb_vocab.fbow \
  -m video2.mp4 \
  -c config.yaml \
  --no-sleep \
  -i my_map.msg
```

---

## Other Executables

### run_camera_slam - Live Camera
```bash
./run_camera_slam \
  -v ../data/orb_vocab.fbow \
  -c <config.yaml> \
  -n 0  # Camera device ID
```

### run_image_slam - Image Sequence
```bash
./run_image_slam \
  -v ../data/orb_vocab.fbow \
  -i <image_directory> \
  -c <config.yaml>
```

### run_kitti_slam - KITTI Dataset
```bash
# Download KITTI dataset first
./run_kitti_slam \
  -v ../data/orb_vocab.fbow \
  -d <kitti_sequence_directory> \
  -c ~/Documents/openvslam/example/kitti/KITTI_stereo_00-02.yaml
```

### run_tum_rgbd_slam - TUM RGB-D Dataset
```bash
# Download TUM RGB-D dataset first
./run_tum_rgbd_slam \
  -v ../data/orb_vocab.fbow \
  -d <tum_dataset_directory> \
  -c ~/Documents/openvslam/example/tum_rgbd/TUM_RGBD_rgbd_1.yaml
```

---

## Troubleshooting

### "command not found" Error

If you get `bash: run_video_slam: command not found`, the executable is not in your PATH.

**Solution 1: Navigate to the build directory first**
```bash
cd ~/Documents/stella_vslam_examples/build
./run_video_slam -v ../data/orb_vocab.fbow -m video.mp4 -c config.yaml
```

**Solution 2: Use the full path**
```bash
~/Documents/stella_vslam_examples/build/run_video_slam -v ~/Documents/stella_vslam_examples/data/orb_vocab.fbow -m video.mp4 -c config.yaml
```

**Solution 3: Add to PATH (permanent)**
```bash
# Add to ~/.bashrc
echo 'export PATH="$HOME/Documents/stella_vslam_examples/build:$PATH"' >> ~/.bashrc
source ~/.bashrc

# Now you can run from anywhere:
run_video_slam -v ~/Documents/stella_vslam_examples/data/orb_vocab.fbow -m video.mp4 -c config.yaml
```

**Solution 4: Create a symbolic link**
```bash
sudo ln -s ~/Documents/stella_vslam_examples/build/run_video_slam /usr/local/bin/run_video_slam
sudo ln -s ~/Documents/stella_vslam_examples/build/run_camera_slam /usr/local/bin/run_camera_slam
sudo ln -s ~/Documents/stella_vslam_examples/build/run_image_slam /usr/local/bin/run_image_slam

# Now you can run from anywhere:
run_video_slam -v ~/Documents/stella_vslam_examples/data/orb_vocab.fbow -m video.mp4 -c config.yaml
```

### Library Not Found Error
If you see "error while loading shared libraries":
```bash
sudo ldconfig
```

### No Viewer Displayed
If running headless or without display, use:
```bash
./run_video_slam \
  -v ~/vocab/orb_vocab.fbow \
  -m video.mp4 \
  -c config.yaml \
  --viewer none
```

### Slow Performance
- Use `--frame-skip 2` or `--frame-skip 3` to process every 2nd/3rd frame
- Use `--no-sleep` to process as fast as possible
- Reduce video resolution in config file

### Memory Issues
- Process shorter video segments
- Use `--temporal-mapping` for long sequences
- Reduce ORB features in config (if you create custom config)

---

## Configuration File Format

Example YAML structure:
```yaml
Camera:
  name: "My Camera"
  setup: "monocular"  # or "stereo", "rgbd"
  model: "perspective"  # or "fisheye", "equirectangular"

  # Image size
  fps: 30.0
  cols: 1920
  rows: 1080

  # Intrinsic parameters
  fx: 1000.0
  fy: 1000.0
  cx: 960.0
  cy: 540.0

  # Distortion parameters (for perspective)
  k1: 0.0
  k2: 0.0
  p1: 0.0
  p2: 0.0
  k3: 0.0

# ORB feature parameters
Feature:
  max_num_keypoints: 2000
  scale_factor: 1.2
  num_levels: 8
  ini_fast_threshold: 20
  min_fast_threshold: 7
```

---

## Downloads

### Required
- **ORB Vocabulary**: https://github.com/stella-cv/FBoW_orb_vocab/raw/main/orb_vocab.fbow

### Optional Datasets
- **Sample Videos**: https://drive.google.com/open?id=1A_gq8LYuENePhNHsuscLZQPhbJJwzAq4
- **KITTI Odometry**: http://www.cvlibs.net/datasets/kitti/eval_odometry.php
- **TUM RGB-D**: https://vision.in.tum.de/data/datasets/rgbd-dataset
- **EuRoC MAV**: https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets

---

## Documentation

- **stella_vslam**: https://stella-cv.readthedocs.io/
- **GitHub**: https://github.com/stella-cv/stella_vslam
- **Examples**: https://github.com/stella-cv/stella_vslam_examples
- **Discussions**: https://github.com/stella-cv/stella_vslam/discussions

---

## Summary

✅ **All executables built successfully**
📂 **Location**: `/home/dan/Documents/stella_vslam_examples/build/`
📥 **Next step**: Download vocabulary file
🎬 **Ready to run**: Use `./run_video_slam` with your video!

---

## Build History and System Setup

This installation was completed on **2025-10-19** on a **Raspberry Pi** (ARM64 architecture).

### What Was Built and Installed

1. **g2o (General Graph Optimization) v0.2.0**
   - Built from source: https://github.com/RainerKuemmerle/g2o.git
   - Installed to: `/usr/local/`
   - Used for: Graph-based optimization in SLAM
   - See [BUILD_INFO.md](BUILD_INFO.md) for full details

2. **stella_vslam library**
   - Built from: `/home/dan/Documents/openvslam/`
   - Installed to: `/usr/local/lib/libstella_vslam.so`
   - Core SLAM library with support for multiple camera models

3. **stella_vslam_examples**
   - Cloned from: https://github.com/stella-cv/stella_vslam_examples.git
   - Built in: `/home/dan/Documents/stella_vslam_examples/build/`
   - Contains all executable programs (run_video_slam, etc.)

### Git Submodules Initialized

- **FBoW** (Fast Bag of Words) - For loop closure detection
- **tinycolormap** - For visualization
- **filesystem** (in examples) - For cross-platform file operations

### Dependencies Installed

The following packages were installed to support the build:
- libsuitesparse-dev (sparse matrix libraries)
- qtdeclarative5-dev, qt5-qmake (Qt5 for visualization)
- libqglviewer-dev-qt5 (OpenGL viewer)
- Existing: OpenCV 4.6.0, Eigen3, yaml-cpp, SQLite3

### Platform Details

- **OS**: Linux 6.12.47+rpt-rpi-2712 (Raspberry Pi)
- **Architecture**: ARM64 (aarch64)
- **Compiler**: GCC 12.2.0
- **Build Type**: Release (optimized)

---

## Complete Installation Steps (For Reference)

If you need to reproduce this setup on another machine:

```bash
# 1. Install dependencies
sudo apt install libsuitesparse-dev qtdeclarative5-dev qt5-qmake libqglviewer-dev-qt5

# 2. Build and install g2o
cd ~/Documents
git clone https://github.com/RainerKuemmerle/g2o.git
cd g2o
mkdir build && cd build
cmake ..
make -j$(nproc)
sudo make install

# 3. Clone stella_vslam and initialize submodules
cd ~/Documents
git clone --recursive https://github.com/stella-cv/stella_vslam.git openvslam
cd openvslam
git submodule update --init --recursive

# 4. Build and install stella_vslam
mkdir build && cd build
cmake ..
make -j$(nproc)
sudo make install

# 5. Clone and build stella_vslam_examples
cd ~/Documents
git clone --recursive https://github.com/stella-cv/stella_vslam_examples.git
cd stella_vslam_examples
mkdir build && cd build
cmake ..
make -j$(nproc)

# 6. Update library cache
sudo ldconfig

# 7. Download vocabulary file
mkdir -p ../data
cd ../data
wget https://github.com/stella-cv/FBoW_orb_vocab/raw/main/orb_vocab.fbow

# 8. Ready to run!
cd ../build
./run_video_slam --help
```

---

## Tips for Raspberry Pi Users

Since you're running on a Raspberry Pi:

### Performance Optimization

1. **Use frame skipping** for smoother performance:
   ```bash
   ./run_video_slam \
     -v ~/vocab/orb_vocab.fbow \
     -m video.mp4 \
     -c config.yaml \
     --frame-skip 3
   ```

2. **Process offline** (no real-time waiting):
   ```bash
   ./run_video_slam \
     -v ~/vocab/orb_vocab.fbow \
     -m video.mp4 \
     -c config.yaml \
     --no-sleep
   ```

3. **Reduce video resolution** in your config YAML:
   ```yaml
   Camera:
     cols: 1280  # Instead of 1920
     rows: 720   # Instead of 1080
   ```

4. **Lower ORB features** in config:
   ```yaml
   Feature:
     max_num_keypoints: 1000  # Instead of 2000
   ```

### Memory Management

- Process shorter video segments if you encounter memory issues
- Use `--temporal-mapping` for long sequences
- Consider using swap space for large datasets

### GPU Acceleration

- The build on Raspberry Pi doesn't use GPU acceleration (CUDA)
- For faster processing, consider using frame-skip and temporal mapping
- OpenMP is available but was disabled in this build (can be re-enabled)

---

## Viewer Options

stella_vslam supports multiple viewers:

1. **pangolin_viewer** (default) - 3D visualization
   - Requires OpenGL support
   - Shows map points, camera trajectory, and current view

2. **iridescence_viewer** - Web-based viewer
   - Can be accessed remotely via browser
   - Good for headless/SSH setups

3. **socket_publisher** - Stream data to external viewer
   - For custom visualization solutions

4. **none** - No viewer (headless mode)
   ```bash
   ./run_video_slam \
     -v ~/vocab/orb_vocab.fbow \
     -m video.mp4 \
     -c config.yaml \
     --viewer none
   ```

For Raspberry Pi remote access, try:
```bash
./run_video_slam \
  -v ~/vocab/orb_vocab.fbow \
  -m video.mp4 \
  -c config.yaml \
  --viewer iridescence_viewer
# Then access via browser at http://<pi-ip-address>:8080
```

---

## Common Camera Configurations

### Raspberry Pi Camera Module v2
```yaml
Camera:
  name: "Raspberry Pi Camera v2"
  setup: "monocular"
  model: "perspective"
  fps: 30.0
  cols: 1920
  rows: 1080
  fx: 1112.0
  fy: 1112.0
  cx: 960.0
  cy: 540.0
  k1: 0.0
  k2: 0.0
  p1: 0.0
  p2: 0.0
  k3: 0.0
```

### USB Webcam (typical)
```yaml
Camera:
  name: "USB Webcam"
  setup: "monocular"
  model: "perspective"
  fps: 30.0
  cols: 1280
  rows: 720
  fx: 800.0
  fy: 800.0
  cx: 640.0
  cy: 360.0
  k1: 0.0
  k2: 0.0
  p1: 0.0
  p2: 0.0
  k3: 0.0
```

**Note**: You'll need to calibrate your camera to get accurate intrinsic parameters. Use OpenCV's camera calibration tools or online calibration tools.

---

## Additional Resources

### Camera Calibration
- **OpenCV Tutorial**: https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html
- **Online Tool**: http://calib.io/

### Sample Videos for Testing
- **stella_vslam samples**: https://drive.google.com/open?id=1A_gq8LYuENePhNHsuscLZQPhbJJwzAq4

### Public SLAM Datasets
- **KITTI Odometry**: http://www.cvlibs.net/datasets/kitti/eval_odometry.php
- **TUM RGB-D**: https://vision.in.tum.de/data/datasets/rgbd-dataset
- **EuRoC MAV**: https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets

### Community Support
- **GitHub Issues**: https://github.com/stella-cv/stella_vslam/issues
- **Discussions**: https://github.com/stella-cv/stella_vslam/discussions
- **Documentation**: https://stella-cv.readthedocs.io/

---

## Important Notes

### Why "command not found"?

The executables are built locally and are **not installed system-wide**. They remain in:
```
~/Documents/stella_vslam_examples/build/
```

This is normal! You have three options:

1. **Always run from the build directory**:
   ```bash
   cd ~/Documents/stella_vslam_examples/build
   ./run_video_slam -v ../data/orb_vocab.fbow -m video.mp4 -c config.yaml
   ```

2. **Add to PATH** (one time, permanent):
   ```bash
   cd ~/Documents/stella_vslam_examples
   ./setup_path.sh
   source ~/.bashrc
   ```

3. **Create symlinks** (requires sudo):
   ```bash
   sudo ln -s ~/Documents/stella_vslam_examples/build/run_video_slam /usr/local/bin/
   sudo ln -s ~/Documents/stella_vslam_examples/build/run_camera_slam /usr/local/bin/
   sudo ln -s ~/Documents/stella_vslam_examples/build/run_image_slam /usr/local/bin/
   ```

### Vocabulary File Location

Remember to download the vocabulary file:
```bash
cd ~/Documents/stella_vslam_examples
mkdir -p data
cd data
wget https://github.com/stella-cv/FBoW_orb_vocab/raw/main/orb_vocab.fbow
```

Then reference it as:
```bash
# If in build directory
./run_video_slam -v ../data/orb_vocab.fbow -m video.mp4 -c config.yaml

# OR if in PATH
run_video_slam -v ~/Documents/stella_vslam_examples/data/orb_vocab.fbow -m video.mp4 -c config.yaml
```

---

Enjoy running stella_vslam! 🚀
