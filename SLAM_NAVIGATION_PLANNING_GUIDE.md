# SLAM, Navigation & Planning Guide
## Complete Tool Documentation for stella_vslam

**Author:** System Documentation
**Date:** 2025-10-19
**Purpose:** Comprehensive guide to using stella_vslam tools for SLAM, navigation, and path planning

---

## Table of Contents

1. [Overview](#overview)
2. [SLAM Executables](#slam-executables)
3. [Map Tools & Manipulation](#map-tools--manipulation)
4. [Trajectory Export/Import](#trajectory-exportimport)
5. [Data Formats](#data-formats)
6. [SLAM-to-Navigation Pipeline](#slam-to-navigation-pipeline)
7. [Practical Examples](#practical-examples)
8. [Tool Reference](#tool-reference)

---

## Overview

### What stella_vslam Provides

stella_vslam is a **localization and mapping system** that outputs:
- **Pose data** - Robot position and orientation (6 DOF)
- **Sparse 3D map** - Feature point cloud
- **Trajectory** - Path history
- **Tracking state** - Quality metrics

### What You Need to Add for Navigation

- **Path Planner** - Calculates route from start to goal
- **Motion Controller** - Executes the path
- **Obstacle Detector** - Converts SLAM landmarks to obstacles
- **Goal Manager** - Sets navigation targets

---

## SLAM Executables

All executables are located in: `/home/dan/Documents/stella_vslam_examples/build/`

### 1. run_image_slam (Image Sequences)

**Purpose:** Process directories of images for SLAM

**Usage:**
```bash
./run_image_slam \
  -v ~/vocab/orb_vocab.fbow \
  -d /tmp/picar_slam_20251019_153000 \
  -c ~/Documents/openvslam/pi_camera_640x480.yaml \
  -o ~/slam_maps/living_room.msg
```

**Parameters:**
- `-v` - Vocabulary file (ORB features)
- `-d` - Directory containing images
- `-c` - Camera calibration YAML
- `-o` - Output map file (optional)
- `--map-db-in` - Load existing map for localization
- `--disable-mapping` - Localization-only mode

**Output Files:**
- Map database (`.msg` or `.db`)
- `frame_trajectory.txt` - All frame poses
- `keyframe_trajectory.txt` - Keyframe poses only

**When to Use:**
- Processing recorded image sequences
- Building maps from PiCar recordings
- Batch SLAM processing

---

### 2. run_video_slam (Video Files)

**Purpose:** Process video files (MP4, AVI, etc.)

**Usage:**
```bash
./run_video_slam \
  -v ~/vocab/orb_vocab.fbow \
  -m ~/Videos/picar_run.mp4 \
  -c ~/Documents/openvslam/pi_camera_640x480.yaml \
  -o ~/slam_maps/kitchen.msg \
  --frame-skip 2
```

**Parameters:**
- `-m` - Video file path
- `--frame-skip N` - Process every Nth frame
- `--no-sleep` - Process as fast as possible
- Other parameters same as `run_image_slam`

**When to Use:**
- Processing pre-recorded video
- Converting video to SLAM map
- Offline SLAM processing

---

### 3. run_camera_slam (Live Camera)

**Purpose:** Real-time SLAM with camera feed

**Usage:**
```bash
./run_camera_slam \
  -v ~/vocab/orb_vocab.fbow \
  -c ~/Documents/openvslam/pi_camera_640x480.yaml \
  -n 0 \
  -o ~/slam_maps/realtime_map.msg
```

**Parameters:**
- `-n` - Camera device ID (0 = /dev/video0)
- Other parameters same as above

**When to Use:**
- Real-time SLAM
- Live mapping
- Active navigation

---

### 4. run_loop_closure (Loop Closure Testing)

**Purpose:** Test loop closure detection between keyframes

**Usage:**
```bash
./run_loop_closure \
  -v ~/vocab/orb_vocab.fbow \
  -p ~/slam_maps/living_room.msg \
  -c ~/Documents/openvslam/pi_camera_640x480.yaml \
  -a 10 \
  -b 150
```

**Parameters:**
- `-p` - Map database path
- `-a` - First keyframe ID
- `-b` - Second keyframe ID

**When to Use:**
- Debugging loop closure
- Testing map consistency
- Validating SLAM quality

---

### 5. Dataset-Specific Executables

**run_kitti_slam** - KITTI Odometry dataset
**run_euroc_slam** - EuRoC MAV dataset (stereo + IMU)
**run_tum_rgbd_slam** - TUM RGB-D benchmark

These are primarily for benchmarking and evaluation.

---

## Map Tools & Manipulation

### Map Database I/O System

Location: `/home/dan/Documents/openvslam/src/stella_vslam/io/`

#### Supported Formats

| Format | Extension | Description | Use Case |
|--------|-----------|-------------|----------|
| **MessagePack** | `.msg` | Binary serialization | Fast, compact, default |
| **SQLite3** | `.db` | Relational database | Queryable, structured |

#### Format Auto-Detection

The factory pattern automatically selects the correct format based on file extension:

```cpp
// In C++ code
auto io_handler = map_database_io_factory::create("msgpack");
io_handler->save(path, cam_db, orb_params_db, map_db);

// Or for SQLite
auto io_handler = map_database_io_factory::create("sqlite3");
```

File: [src/stella_vslam/io/map_database_io_factory.h:23](src/stella_vslam/io/map_database_io_factory.h#L23)

---

### Map Database Contents

A saved map contains:

1. **Camera Database**
   - Intrinsic parameters (fx, fy, cx, cy)
   - Distortion coefficients
   - Camera model type

2. **Keyframes**
   - Pose (4x4 transformation matrix)
   - Feature descriptors
   - Timestamp
   - Graph connections (covisibility)

3. **Landmarks (3D Points)**
   - World coordinates (x, y, z)
   - Observation count
   - Descriptor
   - Observing keyframe IDs

4. **ORB Parameters**
   - Scale factor
   - Number of levels
   - Feature extraction settings

5. **BoW Vocabulary** (if included)
   - Bag-of-Words database for place recognition

---

### Helper Script: save_slam_map.sh

Location: `/home/dan/Documents/openvslam/save_slam_map.sh`

**Purpose:** Automated map building from frame directories

**Usage:**
```bash
./save_slam_map.sh living_room /tmp/picar_slam_20251019_153000
```

**What it does:**
1. Validates frame directory exists
2. Counts frames
3. Runs `run_image_slam` with proper settings
4. Saves map to `~/slam_maps/living_room_TIMESTAMP.msg`
5. Shows localization command for reuse

**Output:**
```
Map saved to: /home/dan/slam_maps/living_room_20251019_155230.msg

To use this map for localization:
  run_image_slam -v ~/vocab/orb_vocab.fbow \
    -d <new_frame_dir> \
    -c ~/Documents/openvslam/pi_camera_640x480.yaml \
    -i /home/dan/slam_maps/living_room_20251019_155230.msg \
    --disable-mapping
```

File: [save_slam_map.sh:1](save_slam_map.sh#L1)

---

## Trajectory Export/Import

### Trajectory I/O Module

Location: `/home/dan/Documents/openvslam/src/stella_vslam/io/trajectory_io.h`

### Two Types of Trajectories

#### 1. Frame Trajectory
- **Every frame** processed by SLAM
- Higher resolution (~30 Hz)
- Includes intermediate poses between keyframes

#### 2. Keyframe Trajectory
- **Keyframes only** (typically 1-5 Hz)
- Lower resolution but more reliable
- Better for evaluation and planning

### Supported Export Formats

#### KITTI Format

**Structure:** 3x4 pose matrix (space-separated)

```
r11 r12 r13 tx r21 r22 r23 ty r31 r32 r33 tz
```

**Example:**
```
0.999998 -0.001767 0.000887 1.234567 0.001768 0.999998 -0.000567 2.345678 -0.000886 0.000568 0.999999 0.012345
```

**Use Case:** KITTI benchmark evaluation, robotics research

#### TUM Format

**Structure:** Timestamp + translation + quaternion (space-separated)

```
timestamp tx ty tz qx qy qz qw
```

**Example:**
```
1634567890.123456 1.234567 2.345678 0.012345 0.001234 0.002345 0.003456 0.999994
```

**Use Case:** TUM benchmark, time-synchronized data, ROS integration

### How to Export Trajectories

#### Method 1: Automatic (from executables)

All SLAM executables automatically save trajectories in the working directory:
- `frame_trajectory.txt` (KITTI format by default)
- `keyframe_trajectory.txt` (KITTI format by default)

#### Method 2: Manual (from C++ API)

```cpp
// In your C++ code
#include "stella_vslam/system.h"

stella_vslam::system SLAM(cfg, vocab);
// ... run SLAM ...

// Save trajectories
SLAM.save_frame_trajectory("path/to/frame_traj.txt", "TUM");
SLAM.save_keyframe_trajectory("path/to/keyframe_traj.txt", "KITTI");
```

File: [src/stella_vslam/io/trajectory_io.cc:20](src/stella_vslam/io/trajectory_io.cc#L20)

---

### Trajectory Data Structure

Each pose in the trajectory contains:

**4x4 Transformation Matrix (SE(3) group):**
```
| R11  R12  R13  tx |
| R21  R22  R23  ty |
| R31  R32  R33  tz |
|  0    0    0   1  |
```

Where:
- **R** (3x3) = Rotation matrix (camera orientation)
- **t** (3x1) = Translation vector (camera position)

**Conversion utilities available in:**
File: [src/stella_vslam/util/converter.h](src/stella_vslam/util/converter.h)

---

## Data Formats

### Camera Configuration (YAML)

Location: `/home/dan/Documents/openvslam/*.yaml`

**Example: pi_camera_640x480.yaml**
```yaml
Camera:
  name: "Raspberry Pi Camera"
  setup: "monocular"
  model: "perspective"

  fx: 452.8
  fy: 451.2
  cx: 320.0
  cy: 240.0

  k1: 0.1
  k2: -0.05
  p1: 0.001
  p2: 0.002

  fps: 30.0
  cols: 640
  rows: 480
  color_order: "Gray"

Feature:
  max_num_keypoints: 2000
  scale_factor: 1.2
  num_levels: 8
  ini_fast_threshold: 20
  min_fast_threshold: 7
```

### Image Sequence Format

**Directory structure:**
```
/tmp/picar_slam_20251019_153000/
├── frame_000000.png
├── frame_000001.png
├── frame_000002.png
├── ...
└── timestamps.txt
```

**timestamps.txt format:**
```
0.000000
0.033333
0.066666
0.100000
...
```

Each line = timestamp in seconds for corresponding frame

---

## SLAM-to-Navigation Pipeline

### Complete Workflow

```
┌─────────────────────────────────────────────────────────────┐
│                    PHASE 1: MAPPING                         │
└─────────────────────────────────────────────────────────────┘

1. CAPTURE FRAMES
   PiCar with camera → Record frames → /tmp/picar_slam_TIMESTAMP/

   Script: picar_slam_live.py
   Press 'R' to start recording
   Drive around environment
   Press 'R' to stop recording

2. BUILD MAP
   Frames → run_image_slam → Map database (.msg)

   Command:
   ./save_slam_map.sh living_room /tmp/picar_slam_20251019_153000

   Output:
   - ~/slam_maps/living_room_TIMESTAMP.msg
   - frame_trajectory.txt
   - keyframe_trajectory.txt

3. VALIDATE MAP
   Check map quality:
   - Number of keyframes (>50 for good coverage)
   - Number of landmarks (>500 for reliable tracking)
   - Trajectory consistency

┌─────────────────────────────────────────────────────────────┐
│              PHASE 2: LOCALIZATION (Real-time)              │
└─────────────────────────────────────────────────────────────┘

4. LOAD MAP FOR LOCALIZATION
   New frames → run_image_slam --disable-mapping → Current pose

   Command:
   run_image_slam \
     -v ~/vocab/orb_vocab.fbow \
     -d <current_frame_dir> \
     -c ~/Documents/openvslam/pi_camera_640x480.yaml \
     -i ~/slam_maps/living_room_TIMESTAMP.msg \
     --disable-mapping

5. EXTRACT CURRENT POSE
   From SLAM publishers (C++ API):

   auto pose = map_pub_->get_current_cam_pose();  // Mat44_t

   Pose format:
   - x, y, z: position in meters
   - Rotation matrix: orientation
   - Update rate: ~30 Hz

┌─────────────────────────────────────────────────────────────┐
│              PHASE 3: PATH PLANNING (Your Code)             │
└─────────────────────────────────────────────────────────────┘

6. EXTRACT MAP LANDMARKS (Obstacles)
   auto landmarks = map_pub_->get_landmarks();

   For each landmark:
   - World position (x, y, z)
   - Reliability (observation count)
   - Filter: Keep only high-confidence landmarks (obs_count > 10)

7. SET NAVIGATION GOAL
   User input → Goal position (x_goal, y_goal)

   Example:
   Goal: (5.0, 3.0) meters from map origin

8. RUN PATH PLANNER
   Input:
   - Current pose from SLAM
   - Goal position
   - Obstacle landmarks

   Algorithm (e.g., A*):
   - Create occupancy grid from landmarks
   - Search for collision-free path
   - Output waypoint sequence

   Output:
   path = [(1.5, 2.3), (2.0, 2.0), (3.5, 2.0), (5.0, 3.0)]

┌─────────────────────────────────────────────────────────────┐
│            PHASE 4: NAVIGATION (Control Loop)               │
└─────────────────────────────────────────────────────────────┘

9. MOTION CONTROLLER
   Loop at 10-50 Hz:

   while not at goal:
       # Get current state from SLAM
       current_pose = slam.get_current_pose()

       # Get next waypoint
       target = path[current_waypoint]

       # Calculate control
       distance = compute_distance(current_pose, target)
       angle_error = compute_heading_error(current_pose, target)

       # PID control
       steering = Kp * angle_error
       speed = min(MAX_SPEED, distance * K_speed)

       # Send to motors
       picar.set_dir_servo_angle(steering)
       picar.forward(speed)

       # Check if waypoint reached
       if distance < WAYPOINT_THRESHOLD:
           current_waypoint += 1

10. DYNAMIC REPLANNING (Optional)
    If new obstacles detected or tracking lost:
    - Recompute path
    - Update controller
    - Continue navigation
```

---

## Practical Examples

### Example 1: Build a Map of Your Living Room

**Step 1: Record frames**
```bash
cd /home/dan/Documents/openvslam
python3 picar_slam_live.py
```

- Drive PiCar around living room
- Press 'R' to start recording
- Drive for 1-2 minutes (smooth, slow movements)
- Press 'R' to stop
- Note the output directory (e.g., `/tmp/picar_slam_20251019_153000`)

**Step 2: Build map**
```bash
./save_slam_map.sh living_room /tmp/picar_slam_20251019_153000
```

**Step 3: Check results**
```bash
ls -lh ~/slam_maps/
# Should see: living_room_TIMESTAMP.msg

# Check trajectory files
head frame_trajectory.txt
head keyframe_trajectory.txt
```

---

### Example 2: Localize in an Existing Map

**Step 1: Load map for localization only**
```bash
cd /home/dan/Documents/stella_vslam_examples/build

./run_image_slam \
  -v ~/vocab/orb_vocab.fbow \
  -d /tmp/new_frames \
  -c ~/Documents/openvslam/pi_camera_640x480.yaml \
  -i ~/slam_maps/living_room_20251019_155230.msg \
  --disable-mapping
```

**What happens:**
- SLAM loads existing map
- Localizes camera in known map
- Does NOT create new landmarks
- Outputs current pose at ~30 Hz

**Output:**
- Real-time pose estimates
- Tracking state ("Tracking", "Lost", "Initializing")

---

### Example 3: Extract Trajectory for Path Planning

**Step 1: Export trajectory in TUM format**
```bash
# Trajectories are auto-saved after SLAM run
# Or manually from C++ code:
SLAM.save_keyframe_trajectory("path.txt", "TUM");
```

**Step 2: Parse trajectory in Python**
```python
import numpy as np

def load_tum_trajectory(filepath):
    """Load TUM format trajectory"""
    data = np.loadtxt(filepath)
    # Columns: timestamp, tx, ty, tz, qx, qy, qz, qw
    timestamps = data[:, 0]
    positions = data[:, 1:4]  # tx, ty, tz
    orientations = data[:, 4:8]  # qx, qy, qz, qw (quaternion)
    return timestamps, positions, orientations

# Load trajectory
timestamps, positions, orientations = load_tum_trajectory('keyframe_trajectory.txt')

# Extract 2D path (for ground robot)
path_2d = positions[:, 0:2]  # x, y only

print(f"Path has {len(path_2d)} waypoints")
print(f"Start: {path_2d[0]}")
print(f"End: {path_2d[-1]}")
```

**Step 3: Use for path planning**
```python
# Use as reference path
# Or compute goal from last position
goal_x, goal_y = path_2d[-1]
```

---

### Example 4: Convert Landmarks to Occupancy Grid

**Step 1: Extract landmarks from map (C++ API)**
```cpp
#include "stella_vslam/publish/map_publisher.h"

// Get all landmarks
auto landmarks = map_pub_->get_landmarks();

// Filter reliable landmarks
std::vector<Vec3_t> obstacle_points;
for (const auto& lm_id : landmarks) {
    auto lm = map_db_->get_landmark(lm_id);
    if (lm && lm->num_observations() > 10) {  // Reliable only
        obstacle_points.push_back(lm->get_pos_in_world());
    }
}
```

**Step 2: Create occupancy grid**
```cpp
// Grid parameters
const float grid_resolution = 0.05;  // 5cm cells
const int grid_width = 200;  // 10m x 10m
const int grid_height = 200;

// Initialize grid (0 = free, 1 = occupied)
std::vector<std::vector<int>> grid(grid_height,
                                    std::vector<int>(grid_width, 0));

// Mark obstacles
for (const auto& point : obstacle_points) {
    int grid_x = static_cast<int>(point.x() / grid_resolution) + grid_width/2;
    int grid_y = static_cast<int>(point.y() / grid_resolution) + grid_height/2;

    if (grid_x >= 0 && grid_x < grid_width &&
        grid_y >= 0 && grid_y < grid_height) {
        grid[grid_y][grid_x] = 1;  // Mark as occupied
    }
}

// Inflate obstacles (robot radius)
const int inflate_radius = 3;  // cells
inflateObstacles(grid, inflate_radius);
```

**Step 3: Use with A* planner**
```cpp
// Run A* on occupancy grid
std::vector<Vec2i> path = astar_planner(grid, start_pos, goal_pos);
```

---

### Example 5: Real-time Navigation Loop (Pseudocode)

```python
#!/usr/bin/env python3
"""
Real-time navigation using SLAM localization
"""

import time
from picarx import Picarx
from slam_interface import SLAMLocalizer  # Your C++ binding
from path_planner import AStarPlanner

# Initialize
picar = Picarx()
slam = SLAMLocalizer(map_path="~/slam_maps/living_room.msg",
                     config="~/Documents/openvslam/pi_camera_640x480.yaml",
                     vocab="~/vocab/orb_vocab.fbow")

planner = AStarPlanner(grid_resolution=0.05)

# Set goal
goal_x, goal_y = 5.0, 3.0  # meters

# Wait for SLAM to initialize
while slam.get_tracking_state() != "Tracking":
    time.sleep(0.1)

# Get current position
current_pose = slam.get_current_pose()  # {x, y, theta}
start_x, start_y = current_pose['x'], current_pose['y']

# Get obstacles from SLAM map
landmarks = slam.get_landmarks()
obstacles = [(lm.x, lm.y) for lm in landmarks if lm.obs_count > 10]

# Plan path
path = planner.plan(start=(start_x, start_y),
                   goal=(goal_x, goal_y),
                   obstacles=obstacles)

if not path:
    print("No path found!")
    exit(1)

print(f"Path has {len(path)} waypoints")

# Navigation control loop
current_waypoint = 0
WAYPOINT_THRESHOLD = 0.2  # meters
Kp_angle = 30.0  # Proportional gain for steering
Kp_speed = 50.0  # Proportional gain for speed

while current_waypoint < len(path):
    # Get current state from SLAM
    pose = slam.get_current_pose()
    current_x, current_y, current_theta = pose['x'], pose['y'], pose['theta']

    # Check tracking quality
    if slam.get_tracking_state() != "Tracking":
        print("Tracking lost! Stopping.")
        picar.stop()
        time.sleep(1.0)
        continue

    # Get target waypoint
    target_x, target_y = path[current_waypoint]

    # Calculate errors
    dx = target_x - current_x
    dy = target_y - current_y
    distance = (dx**2 + dy**2)**0.5

    target_angle = math.atan2(dy, dx)
    angle_error = target_angle - current_theta

    # Normalize angle to [-pi, pi]
    while angle_error > math.pi:
        angle_error -= 2*math.pi
    while angle_error < -math.pi:
        angle_error += 2*math.pi

    # Check if waypoint reached
    if distance < WAYPOINT_THRESHOLD:
        print(f"Reached waypoint {current_waypoint}/{len(path)}")
        current_waypoint += 1
        continue

    # Compute control commands
    steering_angle = Kp_angle * angle_error
    steering_angle = max(-30, min(30, steering_angle))  # Clamp to [-30, 30]

    speed = min(30, Kp_speed * distance)  # Max speed 30

    # Send to motors
    picar.set_dir_servo_angle(steering_angle)
    picar.forward(int(speed))

    # Control rate
    time.sleep(0.05)  # 20 Hz

# Goal reached
print("Goal reached!")
picar.stop()
slam.shutdown()
```

---

## Tool Reference

### SLAM Executables Quick Reference

| Executable | Primary Use | Input | Output |
|------------|-------------|-------|--------|
| `run_image_slam` | Image sequences | Frame directory | Map + trajectory |
| `run_video_slam` | Video files | MP4/AVI file | Map + trajectory |
| `run_camera_slam` | Live camera | Camera device | Map + trajectory |
| `run_loop_closure` | Testing | Map + keyframe IDs | Loop closure result |

### Map Tools Quick Reference

| Tool | Function | Input | Output |
|------|----------|-------|--------|
| `save_slam_map.sh` | Automated mapping | Frame directory | Map file (.msg) |
| `map_database_io_msgpack` | Binary serialization | Map object | .msg file |
| `map_database_io_sqlite3` | Database storage | Map object | .db file |
| `trajectory_io` | Export poses | Map database | KITTI/TUM format |

### Python Scripts Quick Reference

| Script | Purpose | When to Use |
|--------|---------|-------------|
| `picar_slam_live.py` | PiCar frame capture | Recording SLAM data |
| `vilib_direct_slam.py` | Direct frame save | Raspberry Pi camera |
| `vilib_openvslam_adapter.py` | Video streaming | Real-time processing |

### Data Formats Quick Reference

| Format | Extension | Structure | Use Case |
|--------|-----------|-----------|----------|
| MessagePack | `.msg` | Binary | Fast, compact |
| SQLite3 | `.db` | Database | Queryable |
| KITTI | `.txt` | 3x4 matrix | Benchmark |
| TUM | `.txt` | timestamp + pose | ROS integration |

---

## File Locations Reference

**SLAM Executables:**
```
/home/dan/Documents/stella_vslam_examples/build/
├── run_image_slam
├── run_video_slam
├── run_camera_slam
└── run_loop_closure
```

**Core Library Source:**
```
/home/dan/Documents/openvslam/src/stella_vslam/
├── io/
│   ├── trajectory_io.h/cc
│   ├── map_database_io_msgpack.h/cc
│   ├── map_database_io_sqlite3.h/cc
│   └── map_database_io_factory.h
├── publish/
│   ├── map_publisher.h/cc
│   └── frame_publisher.h/cc
└── util/
    └── converter.h
```

**Camera Configurations:**
```
/home/dan/Documents/openvslam/
├── pi_camera_640x480.yaml
├── pi_camera_small.yaml
└── example/
    ├── kitti/
    ├── euroc/
    └── tum_rgbd/
```

**Helper Scripts:**
```
/home/dan/Documents/openvslam/
├── save_slam_map.sh
├── picar_slam_live.py
└── vilib_openvslam_adapter.py
```

---

## Next Steps

### For Your PiCar Navigation Project

1. **Complete Mapping Phase**
   - Use `picar_slam_live.py` to record environment
   - Run `save_slam_map.sh` to build map
   - Validate map quality

2. **Implement Path Planner**
   - Extract landmarks from map
   - Create occupancy grid
   - Implement A* or RRT algorithm

3. **Build Navigation Controller**
   - Create SLAM C++/Python interface
   - Implement control loop (20-50 Hz)
   - Add safety features (obstacle avoidance)

4. **Integrate Complete System**
   - SLAM localization → Planner → Controller → Motors
   - Add error handling and recovery
   - Test in real environment

---

## Additional Resources

**stella_vslam Documentation:**
- GitHub: https://github.com/stella-cv/stella_vslam

**Path Planning Algorithms:**
- A* (A-star): Grid-based shortest path
- RRT (Rapidly-exploring Random Tree): Sampling-based
- DWA (Dynamic Window Approach): Velocity space planning

**Motion Control:**
- Pure Pursuit: Geometric path tracking
- PID Control: Angle and distance correction
- Model Predictive Control (MPC): Advanced control

**ROS Integration:**
- `stella_vslam_ros`: Official ROS wrapper
- `nav_core`: ROS navigation stack
- `move_base`: ROS motion planning

---

**End of Guide**
