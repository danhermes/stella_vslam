# stella_vslam Documentation Index
## Complete Guide to SLAM, Navigation, and Planning

**Project:** PiCar SLAM Navigation
**Date:** 2025-10-19
**Status:** Comprehensive documentation complete

---

## 📚 Documentation Overview

This documentation package provides complete coverage of:
- ✅ SLAM concepts and architecture
- ✅ Navigation and planning fundamentals
- ✅ Tools, scripts, and utilities
- ✅ Visual mapper library internals
- ✅ Practical examples and usage

---

## 📖 Documentation Files

### 1. [SLAM_NAVIGATION_PLANNING_GUIDE.md](SLAM_NAVIGATION_PLANNING_GUIDE.md)
**Complete Tool Documentation for stella_vslam**

**Contents:**
- Overview of SLAM vs Navigation vs Planning
- All SLAM executables (run_image_slam, run_video_slam, etc.)
- Map tools and manipulation (MessagePack, SQLite3)
- Trajectory export/import (KITTI, TUM formats)
- Data formats and file structures
- Complete SLAM-to-navigation pipeline (4 phases)
- Practical examples with code
- Tool reference tables

**When to use:**
- Learning how to use stella_vslam tools
- Building and saving maps
- Exporting trajectories for analysis
- Understanding the complete workflow
- Setting up your PiCar for SLAM

**Key sections:**
- SLAM Executables Quick Reference
- Map Database Contents
- Trajectory Export Methods
- Complete 4-Phase Pipeline Workflow
- 5 Practical Examples with Commands

---

### 2. [VISUAL_MAPPER_LIBRARY_GUIDE.md](VISUAL_MAPPER_LIBRARY_GUIDE.md)
**Simple Visual Mapping Library Guide**

**Contents:**
- What is a visual mapper?
- Core component architecture
- System core (entry point)
- Feature extraction (ORB)
- Tracking module (pose estimation)
- Mapping module (3D reconstruction)
- Data structures (frame, keyframe, landmark)
- How landmarks are created (triangulation)
- Complete data flow
- Integration examples (C++, Python)

**When to use:**
- Understanding how stella_vslam works internally
- Learning about visual features (ORB)
- Understanding tracking vs mapping
- Integrating SLAM into custom code
- Debugging SLAM issues

**Key sections:**
- ORB Feature Extraction Explained
- Tracking Pipeline (6 steps)
- Mapping Pipeline (background thread)
- Data Structure Definitions
- Minimal C++ Integration Example

---

### 3. [USAGE_GUIDE.md](USAGE_GUIDE.md) *(if exists)*
**Practical usage instructions**

---

### 4. [VILIB_QUICKSTART.md](VILIB_QUICKSTART.md) *(if exists)*
**Raspberry Pi camera (Vilib) integration**

---

## 🎯 Quick Navigation by Topic

### I Want To...

#### **Understand SLAM Concepts**
→ Read: [SLAM_NAVIGATION_PLANNING_GUIDE.md](SLAM_NAVIGATION_PLANNING_GUIDE.md) - Overview section
- What SLAM provides (pose, map, tracking state)
- What you need to add (planner, controller)
- How they work together

#### **Build a Map with PiCar**
→ Read: [SLAM_NAVIGATION_PLANNING_GUIDE.md](SLAM_NAVIGATION_PLANNING_GUIDE.md) - Example 1
```bash
# Step 1: Record frames
python3 picar_slam_live.py

# Step 2: Build map
./save_slam_map.sh living_room /tmp/picar_slam_20251019_153000
```

#### **Use an Existing Map for Localization**
→ Read: [SLAM_NAVIGATION_PLANNING_GUIDE.md](SLAM_NAVIGATION_PLANNING_GUIDE.md) - Example 2
```bash
run_image_slam -v ~/vocab/orb_vocab.fbow \
  -d /tmp/new_frames \
  -c ~/Documents/openvslam/pi_camera_640x480.yaml \
  -i ~/slam_maps/living_room.msg \
  --disable-mapping
```

#### **Understand How the Visual Mapper Works**
→ Read: [VISUAL_MAPPER_LIBRARY_GUIDE.md](VISUAL_MAPPER_LIBRARY_GUIDE.md) - All sections
- System architecture
- Feature extraction (ORB)
- Tracking and mapping modules
- Data structures

#### **Integrate SLAM into My Code**
→ Read: [VISUAL_MAPPER_LIBRARY_GUIDE.md](VISUAL_MAPPER_LIBRARY_GUIDE.md) - Section 6 & 8
```cpp
stella_vslam::system SLAM(cfg, "vocab.fbow");
SLAM.startup();
auto pose = SLAM.feed_monocular_frame(frame, timestamp);
```

#### **Export Trajectory Data**
→ Read: [SLAM_NAVIGATION_PLANNING_GUIDE.md](SLAM_NAVIGATION_PLANNING_GUIDE.md) - Trajectory Export section
```cpp
SLAM.save_frame_trajectory("path.txt", "TUM");
SLAM.save_keyframe_trajectory("path.txt", "KITTI");
```

#### **Use Landmarks for Obstacle Detection**
→ Read: [SLAM_NAVIGATION_PLANNING_GUIDE.md](SLAM_NAVIGATION_PLANNING_GUIDE.md) - Example 4
```cpp
auto landmarks = map_pub_->get_landmarks();
// Convert to occupancy grid
```

#### **Implement Path Planning**
→ Read: [SLAM_NAVIGATION_PLANNING_GUIDE.md](SLAM_NAVIGATION_PLANNING_GUIDE.md) - Phase 3 of pipeline
- Extract current pose from SLAM
- Extract landmarks as obstacles
- Run A* or RRT planner
- Generate waypoint sequence

#### **Build a Navigation Controller**
→ Read: [SLAM_NAVIGATION_PLANNING_GUIDE.md](SLAM_NAVIGATION_PLANNING_GUIDE.md) - Example 5
```python
while not_at_goal:
    pose = slam.get_current_pose()
    steering = calculate_steering(pose, waypoint)
    picar.set_dir_servo_angle(steering)
```

---

## 🗂️ File Structure Reference

### Configuration Files
```
/home/dan/Documents/openvslam/
├── pi_camera_640x480.yaml      # PiCar camera config
├── pi_camera_small.yaml        # Smaller resolution
└── example/                     # Example configs
    ├── kitti/
    ├── euroc/
    └── tum_rgbd/
```

### Executables
```
/home/dan/Documents/stella_vslam_examples/build/
├── run_image_slam       # Process image sequences
├── run_video_slam       # Process video files
├── run_camera_slam      # Live camera SLAM
└── run_loop_closure     # Loop closure testing
```

### Python Scripts
```
/home/dan/Documents/openvslam/
├── picar_slam_live.py              # PiCar recording + control
├── vilib_direct_slam.py            # Direct frame capture
├── vilib_openvslam_adapter.py      # Video streaming adapter
└── save_slam_map.sh                # Automated map building
```

### Core Library Source
```
/home/dan/Documents/openvslam/src/stella_vslam/
├── system.h/cc                     # Main SLAM system
├── tracking_module.h/cc            # Real-time tracking
├── mapping_module.h/cc             # Background mapping
├── feature/
│   └── orb_extractor.h/cc          # ORB feature extraction
├── data/
│   ├── frame.h/cc                  # Frame data structure
│   ├── keyframe.h/cc               # Keyframe storage
│   ├── landmark.h/cc               # 3D point storage
│   └── map_database.h/cc           # Map storage
├── io/
│   ├── trajectory_io.h/cc          # Trajectory export
│   ├── map_database_io_msgpack.h/cc # Map save/load (binary)
│   └── map_database_io_sqlite3.h/cc # Map save/load (database)
├── publish/
│   ├── map_publisher.h/cc          # Map data access
│   └── frame_publisher.h/cc        # Frame data access
└── module/
    ├── two_view_triangulator.h/cc  # 3D point creation
    ├── local_map_updater.h/cc      # Local map management
    └── local_map_cleaner.h/cc      # Redundancy removal
```

---

## 🔧 Common Workflows

### Workflow 1: Build a Map from Scratch

```bash
# 1. Record frames with PiCar
python3 picar_slam_live.py
# Press 'R' to start recording
# Drive around slowly
# Press 'R' to stop

# 2. Build map from frames
./save_slam_map.sh my_environment /tmp/picar_slam_TIMESTAMP

# 3. Check output
ls -lh ~/slam_maps/
# Output: my_environment_TIMESTAMP.msg
```

**Result:** Reusable map file for localization

---

### Workflow 2: Localize in Existing Map

```bash
# 1. Capture new frames
python3 vilib_direct_slam.py -o /tmp/new_run

# 2. Localize (no new mapping)
run_image_slam \
  -v ~/vocab/orb_vocab.fbow \
  -d /tmp/new_run \
  -c ~/Documents/openvslam/pi_camera_640x480.yaml \
  -i ~/slam_maps/my_environment.msg \
  --disable-mapping

# 3. Check trajectory
cat frame_trajectory.txt
```

**Result:** Camera poses in known map

---

### Workflow 3: Export Data for Analysis

```bash
# 1. Run SLAM (creates trajectory files automatically)
run_image_slam -v ~/vocab/orb_vocab.fbow \
  -d /tmp/frames \
  -c pi_camera_640x480.yaml

# 2. Trajectory files created:
#    - frame_trajectory.txt (all frames)
#    - keyframe_trajectory.txt (keyframes only)

# 3. Parse in Python
python3
>>> import numpy as np
>>> data = np.loadtxt('frame_trajectory.txt')  # KITTI format
>>> positions = data[:, [3, 7, 11]]  # Extract x, y, z columns
```

**Result:** Trajectory data for plotting/analysis

---

### Workflow 4: Real-time Navigation (Conceptual)

```python
# 1. Initialize SLAM
slam = SLAMSystem(config, vocab)
slam.startup()

# 2. Set goal
goal = (5.0, 3.0)  # meters

# 3. Plan path
obstacles = slam.get_landmarks()
path = planner.plan(start=slam.get_pose(), goal=goal, obstacles=obstacles)

# 4. Navigate
for waypoint in path:
    while not_at_waypoint(waypoint):
        pose = slam.get_current_pose()  # Real-time from SLAM
        control = controller.compute(pose, waypoint)
        picar.drive(control)
```

**Result:** Autonomous navigation using SLAM

---

## 📊 Key Concepts Summary

### SLAM Outputs

| Output | Type | Rate | Description |
|--------|------|------|-------------|
| **Pose** | Mat44_t (4x4) | ~30 Hz | Camera position + orientation |
| **Map** | Keyframes + Landmarks | Growing | 3D environment representation |
| **State** | String | ~30 Hz | "Tracking", "Lost", "Initializing" |

### Data Formats

| Format | Extension | Use Case |
|--------|-----------|----------|
| MessagePack | `.msg` | Fast binary map storage |
| SQLite3 | `.db` | Queryable map database |
| KITTI | `.txt` | 3x4 pose matrices |
| TUM | `.txt` | Timestamp + pose (ROS-friendly) |

### SLAM vs Planning vs Navigation

| Component | Frequency | Purpose | Output |
|-----------|-----------|---------|--------|
| **SLAM** | 30 Hz | Localization + Mapping | Pose, Map |
| **Planner** | 0.1-1 Hz | Path calculation | Waypoint sequence |
| **Navigator** | 10-50 Hz | Path following | Motor commands |

---

## 🎓 Learning Path

### Beginner (Getting Started)

1. Read: Overview in [SLAM_NAVIGATION_PLANNING_GUIDE.md](SLAM_NAVIGATION_PLANNING_GUIDE.md)
2. Try: Example 1 - Build a map
3. Try: Example 2 - Localize in existing map
4. Understand: Data formats and file structures

### Intermediate (Understanding Internals)

1. Read: [VISUAL_MAPPER_LIBRARY_GUIDE.md](VISUAL_MAPPER_LIBRARY_GUIDE.md) - Full guide
2. Study: Tracking module (pose estimation)
3. Study: Mapping module (3D reconstruction)
4. Understand: Data structures (frame, keyframe, landmark)

### Advanced (Custom Integration)

1. Study: System API in [system.h:1](src/stella_vslam/system.h#L1)
2. Study: Publisher interfaces in [map_publisher.h](src/stella_vslam/publish/map_publisher.h)
3. Implement: Custom path planner with SLAM data
4. Implement: Navigation controller with real-time feedback

---

## 🚀 Next Steps for PiCar Project

### Phase 1: SLAM Setup (Current)
- ✅ Documentation complete
- ✅ Understanding SLAM system
- 🔲 Test map building with PiCar
- 🔲 Validate map quality

### Phase 2: Planning
- 🔲 Extract landmarks from map
- 🔲 Convert to occupancy grid
- 🔲 Implement A* path planner
- 🔲 Test with waypoint goals

### Phase 3: Navigation
- 🔲 Create SLAM Python/C++ interface
- 🔲 Implement control loop (PID)
- 🔲 Integrate planner + controller
- 🔲 Test autonomous navigation

### Phase 4: Integration
- 🔲 Add obstacle avoidance
- 🔲 Add error handling
- 🔲 Add web interface (optional)
- 🔲 Field testing

---

## 📞 Quick Reference

### Common Commands

```bash
# Build map from images
run_image_slam -v vocab.fbow -d /frames -c config.yaml -o map.msg

# Localize only (no mapping)
run_image_slam -v vocab.fbow -d /frames -c config.yaml -i map.msg --disable-mapping

# Real-time camera SLAM
run_camera_slam -v vocab.fbow -c config.yaml -n 0

# Automated map building
./save_slam_map.sh map_name /tmp/frame_directory
```

### Key File Paths

```bash
# Vocabulary (required)
~/vocab/orb_vocab.fbow

# Camera config
~/Documents/openvslam/pi_camera_640x480.yaml

# Executables
/home/dan/Documents/stella_vslam_examples/build/

# Maps output
~/slam_maps/

# Python scripts
/home/dan/Documents/openvslam/*.py
```

---

## 📝 Additional Notes

### Performance Tips

1. **Feature count:** 2000 features works well for most environments
2. **Frame rate:** 30 FPS is ideal, but 15-20 FPS acceptable
3. **Movement:** Smooth, slow movements produce better maps
4. **Lighting:** Avoid rapid lighting changes
5. **Texture:** Feature-rich environments work better than plain walls

### Troubleshooting

**"Tracking Lost"**
- Move slower
- Ensure good lighting
- Check feature count (should be >500)
- Verify camera calibration

**"Map has few landmarks"**
- Ensure textured environment
- Check feature extraction settings
- Increase max_num_keypoints in config

**"Localization fails in existing map"**
- Ensure same camera calibration
- Check if environment changed
- Try relocalization mode

---

## 🔗 External Resources

**stella_vslam GitHub:**
https://github.com/stella-cv/stella_vslam

**Original ORB-SLAM2 Paper:**
"ORB-SLAM2: an Open-Source SLAM System for Monocular, Stereo and RGB-D cameras"

**Path Planning Algorithms:**
- A* (A-star): Shortest path on grid
- RRT: Rapidly-exploring Random Tree
- DWA: Dynamic Window Approach

**ROS Integration:**
- stella_vslam_ros package for ROS wrapper
- nav_core for navigation stack

---

## ✅ Documentation Checklist

- ✅ SLAM concepts explained
- ✅ Navigation vs Planning clarified
- ✅ All tools documented
- ✅ Map formats explained
- ✅ Trajectory export/import covered
- ✅ Visual mapper internals documented
- ✅ Data structures defined
- ✅ Practical examples provided
- ✅ Integration examples shown
- ✅ Common workflows documented
- ✅ File reference complete
- ✅ Quick reference created

---

## 📄 Document Versions

| Document | Version | Last Updated | Status |
|----------|---------|--------------|--------|
| SLAM_NAVIGATION_PLANNING_GUIDE.md | 1.0 | 2025-10-19 | Complete |
| VISUAL_MAPPER_LIBRARY_GUIDE.md | 1.0 | 2025-10-19 | Complete |
| README_DOCUMENTATION.md | 1.0 | 2025-10-19 | Complete |

---

**All documentation complete! Ready for PiCar SLAM navigation development.**

---

**End of Documentation Index**
