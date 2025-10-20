# Simple Visual Mapper Library Guide
## stella_vslam Core Mapping System

**Purpose:** Understanding the simple visual mapping library in stella_vslam
**Date:** 2025-10-19
**For:** PiCar SLAM Navigation Project

---

## Overview: What is a Visual Mapper?

A **visual mapper** is a library that:
1. Takes **camera images** as input
2. Extracts **visual features** (distinctive points)
3. Tracks features **frame-to-frame**
4. Creates **3D landmarks** from 2D features
5. Outputs **camera pose** (position + orientation) and **3D map**

stella_vslam is a **monocular/stereo/RGBD visual SLAM** library that does exactly this.

---

## Core Components (The Simple Visual Mapper)

### Architecture Diagram

```
┌────────────────────────────────────────────────────────┐
│                    SYSTEM (Orchestrator)               │
│                   system.h / system.cc                 │
└────────────────┬───────────────────────┬───────────────┘
                 │                       │
     ┌───────────▼──────────┐   ┌────────▼─────────────┐
     │  TRACKING MODULE     │   │  MAPPING MODULE      │
     │  tracking_module.h   │   │  mapping_module.h    │
     │                      │   │                      │
     │ • Track frames       │   │ • Create landmarks   │
     │ • Estimate pose      │   │ • Optimize map       │
     │ • Decide keyframes   │   │ • Clean redundancy   │
     └──────────┬───────────┘   └──────────┬───────────┘
                │                          │
                │                          │
     ┌──────────▼──────────────────────────▼───────────┐
     │           DATA LAYER (Map Database)             │
     │                                                  │
     │  • Keyframes (selected frames)                  │
     │  • Landmarks (3D points)                        │
     │  • BoW Database (place recognition)             │
     └──────────────────────────────────────────────────┘
```

---

## 1. System Core (Entry Point)

**File:** [src/stella_vslam/system.h](src/stella_vslam/system.h)

**Purpose:** Main orchestrator - manages tracking, mapping, and data I/O

### Key Methods

```cpp
// Initialize system
system(const std::shared_ptr<config>& cfg,
       const std::string& vocab_file_path);

// Start SLAM
void startup(const bool need_initialize = true);

// Feed a monocular frame - MAIN ENTRY POINT
std::shared_ptr<Mat44_t> feed_monocular_frame(
    const cv::Mat& img,           // Input image
    const double timestamp,        // Time in seconds
    const cv::Mat& mask = cv::Mat{} // Optional mask
);

// Feed stereo frames
std::shared_ptr<Mat44_t> feed_stereo_frame(
    const cv::Mat& left_img,
    const cv::Mat& right_img,
    const double timestamp,
    const cv::Mat& mask = cv::Mat{}
);

// Feed RGB-D frame
std::shared_ptr<Mat44_t> feed_RGBD_frame(
    const cv::Mat& rgb_img,
    const cv::Mat& depth_img,
    const double timestamp,
    const cv::Mat& mask = cv::Mat{}
);

// Save outputs
void save_frame_trajectory(const std::string& path,
                          const std::string& format) const;
void save_map_database(const std::string& path) const;

// Shutdown
void shutdown();
```

### What It Contains

```cpp
class system {
private:
    // Feature extractors (ORB detector + descriptor)
    std::shared_ptr<feature::orb_extractor> extractor_left_;
    std::shared_ptr<feature::orb_extractor> extractor_right_;

    // Core modules
    std::unique_ptr<tracking_module> tracker_;         // Real-time tracking
    std::unique_ptr<mapping_module> mapper_;           // Background mapping
    std::unique_ptr<global_optimization_module> global_optimizer_;

    // Data storage
    data::map_database* map_db_;          // Keyframes + landmarks
    data::bow_database* bow_db_;          // Place recognition
    data::camera_database* cam_db_;       // Camera parameters

    // Publishers (for external access)
    std::shared_ptr<publish::map_publisher> map_publisher_;
    std::shared_ptr<publish::frame_publisher> frame_publisher_;

    // Background thread
    std::thread mapping_thread_;          // Runs mapper_->run()
};
```

---

## 2. Feature Extraction (Visual Features)

**File:** [src/stella_vslam/feature/orb_extractor.h](src/stella_vslam/feature/orb_extractor.h)

**Purpose:** Extract ORB (Oriented FAST and Rotated BRIEF) features from images

### What is ORB?

**ORB** = Visual feature detector that finds distinctive corner points in images
- **Keypoint:** 2D position (x, y) + scale + orientation
- **Descriptor:** 256-bit binary string representing the appearance around keypoint
- **Fast:** Suitable for real-time SLAM
- **Invariant:** Robust to rotation, scale, and lighting changes

### Key Method

```cpp
class orb_extractor {
public:
    // Main extraction function
    void extract(
        const cv::Mat& image,                    // Input image
        const cv::Mat& mask,                     // Mask (optional)
        std::vector<cv::KeyPoint>& keypts,       // Output: keypoints
        cv::Mat& descriptors                     // Output: descriptors
    );

    // Parameters
    const orb_params* orb_params_;  // Config (num features, scale, levels)
    std::vector<cv::Mat> image_pyramid_;  // Multi-scale pyramid
};
```

### Configuration (orb_params)

```yaml
Feature:
  max_num_keypoints: 2000    # Max features per image
  scale_factor: 1.2          # Scale between pyramid levels
  num_levels: 8              # Number of pyramid levels
  ini_fast_threshold: 20     # FAST corner threshold
  min_fast_threshold: 7      # Minimum threshold (adaptive)
```

### Output Example

For a single image:
```
Keypoints: 1,247 detected
  - Keypoint #0: (x=245, y=183, scale=1.44, angle=67°)
  - Keypoint #1: (x=512, y=378, scale=1.0, angle=142°)
  - ...

Descriptors: 1,247 x 256-bit binary vectors
  - Descriptor #0: 10110100101... (256 bits)
  - Descriptor #1: 01001110010... (256 bits)
  - ...
```

---

## 3. Tracking Module (Real-time Pose Estimation)

**File:** [src/stella_vslam/tracking_module.h](src/stella_vslam/tracking_module.h)

**Purpose:** Track camera frame-by-frame, estimate pose, decide when to create keyframes

### Main Workflow

```cpp
class tracking_module {
public:
    // Main tracking function (called for each frame)
    std::shared_ptr<Mat44_t> feed_frame(data::frame curr_frm);

private:
    // Tracking states
    enum class tracker_state_t {
        NotInitialized,  // Waiting for initialization
        Initializing,    // First few frames
        Tracking,        // Normal operation
        Lost             // Tracking failure
    };

    // Core tracking methods
    bool initialize();                    // Initialize with first frames
    bool track_current_frame();          // Estimate current pose
    void update_local_map();             // Get nearby keyframes/landmarks
    void search_local_landmarks();       // Find visible landmarks
    bool new_keyframe_is_needed();       // Decide keyframe insertion
    void insert_new_keyframe();          // Create and queue keyframe
};
```

### Tracking Pipeline (Every Frame)

```
1. FEATURE EXTRACTION
   Image → orb_extractor::extract() → Keypoints + Descriptors

2. INITIAL POSE ESTIMATION
   - Match features to previous frame OR local map
   - PnP (Perspective-n-Point) solver → Camera pose estimate

3. LOCAL MAP UPDATE
   - module::local_map_updater::acquire_local_map()
   - Get nearby keyframes (covisibility graph)
   - Get observable landmarks

4. POSE OPTIMIZATION
   - Match more features to local landmarks
   - Bundle adjustment (optimize pose with 3D-2D matches)
   - Refine camera pose

5. KEYFRAME DECISION
   - Check tracking quality
   - Check time since last keyframe
   - Check scene change
   → If needed: Create keyframe and queue to mapping

6. OUTPUT
   - Camera pose (4x4 matrix)
   - Tracking state ("Tracking", "Lost", "Initializing")
```

### Modules Used by Tracking

| Module | File | Purpose |
|--------|------|---------|
| `initializer` | `module/initializer.h` | Two-view initialization |
| `frame_tracker` | `module/frame_tracker.h` | Frame-to-frame matching |
| `relocalizer` | `module/relocalizer.h` | Recover from tracking loss |
| `keyframe_inserter` | `module/keyframe_inserter.h` | Keyframe creation |
| `local_map_updater` | `module/local_map_updater.h` | Local map management |

---

## 4. Mapping Module (3D Map Building)

**File:** [src/stella_vslam/mapping_module.h](src/stella_vslam/mapping_module.h)

**Purpose:** Background thread that builds and optimizes the 3D map

### Main Workflow

```cpp
class mapping_module {
public:
    // Background thread main loop
    void run();

    // Queue a keyframe for processing
    std::shared_future<void> async_add_keyframe(
        const std::shared_ptr<data::keyframe>& keyfrm
    );

private:
    // Core mapping operations
    void mapping_with_new_keyframe();    // Main mapping function
    void store_new_keyframe();           // Add to database
    void create_new_landmarks();         // Triangulate 3D points
    void triangulate_with_two_keyframes(); // 3D point creation
    void fuse_landmark_duplication();    // Merge duplicate landmarks
    void update_new_keyframe();          // Update connections

    // Optimization
    optimize::local_bundle_adjuster* local_bundle_adjuster_;

    // Cleaning
    module::local_map_cleaner* local_map_cleaner_;
};
```

### Mapping Pipeline (For Each Keyframe)

```
KEYFRAME RECEIVED FROM TRACKING
         ↓
1. STORE KEYFRAME
   → map_database::add_keyframe()
   → Update BoW database for place recognition
   → Update covisibility graph

2. CREATE NEW LANDMARKS
   ├─→ Get neighbor keyframes (top 10 by covisibility)
   │
   ├─→ For each neighbor pair:
   │   ├─→ Match ORB features between keyframes
   │   ├─→ two_view_triangulator::triangulate()
   │   │   - Compute 3D position from 2D correspondences
   │   │   - Validate depth, parallax, reprojection error
   │   └─→ Create data::landmark
   │       └─→ map_database::add_landmark()
   │
   └─→ Result: New 3D points added to map

3. FUSE DUPLICATE LANDMARKS
   → Find landmarks representing same 3D point
   → Merge observations
   → Remove duplicates

4. UPDATE KEYFRAME CONNECTIONS
   → Update covisibility graph
   → Update essential graph (spanning tree)

5. LOCAL BUNDLE ADJUSTMENT (Optimization)
   → Optimize poses of local keyframes
   → Optimize positions of local landmarks
   → Minimize reprojection error

6. LOCAL MAP CLEANING
   ├─→ Remove invalid landmarks:
   │   - Low observation count
   │   - High outlier ratio
   │
   └─→ Remove redundant keyframes:
       - Similar viewpoint
       - Most landmarks seen by other keyframes
```

### Key Components

**File:** [src/stella_vslam/module/two_view_triangulator.h](src/stella_vslam/module/two_view_triangulator.h)

```cpp
class two_view_triangulator {
public:
    // Triangulate 3D point from two views
    bool triangulate(
        const Vec3_t& bearing_1,        // Ray from keyframe 1
        const Vec3_t& bearing_2,        // Ray from keyframe 2
        const Mat44_t& cam_pose_cw_1,   // Pose of keyframe 1
        const Mat44_t& cam_pose_cw_2,   // Pose of keyframe 2
        Vec3_t& pos_w                   // Output: 3D position
    ) const;

private:
    bool check_depth_is_positive();      // Point in front of camera?
    bool check_reprojection_error();     // Triangulation accurate?
    bool check_scale_factors();          // Scale consistency?
};
```

**Purpose:** Creates 3D world points from 2D feature matches

**How it works:**
```
Keyframe 1 (pose P1)          Keyframe 2 (pose P2)
     📷                             📷
      |                              |
      | ray1                    ray2 |
      |                              |
      └──────────────┬───────────────┘
                     │
                     ● 3D Point (triangulated)
```

Mathematical approach:
1. Each keypoint = 2D pixel position
2. Convert to normalized ray (bearing vector)
3. Intersect rays from two camera positions
4. Result = 3D position in world coordinates

---

## 5. Data Structures

### Frame (Temporary)

**File:** [src/stella_vslam/data/frame.h](src/stella_vslam/data/frame.h)

```cpp
class frame {
public:
    // Frame ID
    unsigned int id_;

    // Timestamp
    double timestamp_;

    // Camera pose (estimated)
    Mat44_t pose_cw_;  // Camera-to-world transformation

    // Observations (features detected in this frame)
    frame_observation frm_obs_;

    // Landmarks associated with keypoints
    std::vector<std::shared_ptr<landmark>> landmarks_;

    // Reference keyframe (for tracking)
    std::shared_ptr<keyframe> ref_keyfrm_;
};

struct frame_observation {
    // Keypoints (2D positions)
    std::vector<cv::KeyPoint> undist_keypts_;

    // Descriptors (ORB features)
    cv::Mat descriptors_;

    // Bearing vectors (normalized 3D rays)
    eigen_alloc_vector<Vec3_t> bearings_;

    // Stereo depth (if stereo/RGBD)
    std::vector<float> stereo_x_right_;
    std::vector<float> depths_;
};
```

**Purpose:** Represents a single camera frame (temporary, discarded unless promoted to keyframe)

---

### Keyframe (Permanent)

**File:** [src/stella_vslam/data/keyframe.h](src/stella_vslam/data/keyframe.h)

```cpp
class keyframe {
public:
    // Keyframe ID
    unsigned int id_;

    // Timestamp
    double timestamp_;

    // Camera pose (optimized)
    Mat44_t pose_cw_;
    Mat44_t pose_wc_;  // Cached inverse

    // Frame observation (features)
    frame_observation frm_obs_;

    // Landmarks observed
    std::vector<std::shared_ptr<landmark>> landmarks_;

    // Graph connections
    std::map<std::shared_ptr<keyframe>, int> connected_keyfrms_and_weights_;

    // Covisibility graph (neighboring keyframes)
    std::vector<std::shared_ptr<keyframe>> ordered_covisibilities_;

    // Essential graph (spanning tree)
    std::shared_ptr<keyframe> spanning_parent_;
    std::set<std::shared_ptr<keyframe>> spanning_children_;

    // Loop edges
    std::set<std::shared_ptr<keyframe>> loop_edges_;
};
```

**Purpose:** Selected frames stored permanently in the map

**Why keyframes?**
- Not every frame is useful (redundant information)
- Keyframes = important viewpoints with new information
- Reduces map size while maintaining accuracy

---

### Landmark (3D Point)

**File:** [src/stella_vslam/data/landmark.h](src/stella_vslam/data/landmark.h)

```cpp
class landmark {
public:
    // Landmark ID
    unsigned int id_;

    // 3D world position
    Vec3_t pos_w_;

    // Observations (keyframes that see this landmark)
    std::map<std::shared_ptr<keyframe>, unsigned int> observations_;

    // Representative descriptor (most distinctive)
    cv::Mat descriptor_;

    // Viewing properties
    Vec3_t mean_normal_;       // Average viewing direction
    float min_valid_dist_;     // Min observation distance
    float max_valid_dist_;     // Max observation distance

    // Statistics
    unsigned int num_observations() const;
    unsigned int num_valid_observations() const;

    // Quality tracking
    unsigned int num_observable_;  // Should be visible
    unsigned int num_observed_;    // Actually observed
};
```

**Purpose:** Represents a 3D point in the world

**Observation example:**
```
Landmark #42 (pos = [1.5, 2.3, 0.8])
  Observed by:
    - Keyframe #10 (feature index 147)
    - Keyframe #12 (feature index 89)
    - Keyframe #15 (feature index 234)

  Observations: 3
  Descriptor: 01101001... (256 bits)
  Mean normal: [0.2, 0.5, 0.8] (normalized)
```

---

### Map Database

**File:** [src/stella_vslam/data/map_database.h](src/stella_vslam/data/map_database.h)

```cpp
class map_database {
public:
    // Add elements
    void add_keyframe(const std::shared_ptr<keyframe>& keyfrm);
    void add_landmark(const std::shared_ptr<landmark>& lm);

    // Remove elements
    void erase_keyframe(const std::shared_ptr<keyframe>& keyfrm);
    void erase_landmark(const std::shared_ptr<landmark>& lm);

    // Retrieve elements
    std::shared_ptr<keyframe> get_keyframe(unsigned int id) const;
    std::shared_ptr<landmark> get_landmark(unsigned int id) const;

    // Get all elements
    std::vector<std::shared_ptr<keyframe>> get_all_keyframes() const;
    std::vector<std::shared_ptr<landmark>> get_all_landmarks() const;

    // Statistics
    unsigned int get_num_keyframes() const;
    unsigned int get_num_landmarks() const;

private:
    // Storage
    std::unordered_map<unsigned int, std::shared_ptr<keyframe>> keyframes_;
    std::unordered_map<unsigned int, std::shared_ptr<landmark>> landmarks_;
};
```

**Purpose:** Central storage for the entire map

---

## 6. Simple Visual Mapper in Action

### Minimal Example (C++)

```cpp
#include "stella_vslam/system.h"
#include "stella_vslam/config.h"
#include <opencv2/opencv.hpp>

int main() {
    // 1. Create SLAM system
    auto cfg = std::make_shared<stella_vslam::config>("config.yaml");
    stella_vslam::system SLAM(cfg, "vocab.fbow");

    // 2. Start SLAM
    SLAM.startup();

    // 3. Open camera
    cv::VideoCapture cap(0);

    // 4. Process frames
    cv::Mat frame;
    double timestamp = 0.0;

    while (cap.read(frame)) {
        // Feed frame to visual mapper
        auto cam_pose = SLAM.feed_monocular_frame(frame, timestamp);

        if (cam_pose) {
            // Camera pose available (4x4 matrix)
            std::cout << "Position: "
                     << (*cam_pose)(0,3) << ", "
                     << (*cam_pose)(1,3) << ", "
                     << (*cam_pose)(2,3) << std::endl;
        }

        timestamp += 0.033;  // 30 FPS
    }

    // 5. Save results
    SLAM.save_map_database("map.msg");
    SLAM.save_frame_trajectory("trajectory.txt", "TUM");

    // 6. Shutdown
    SLAM.shutdown();

    return 0;
}
```

That's it! The visual mapper handles everything internally:
- Feature extraction
- Tracking
- Mapping
- Optimization

---

## 7. Data Flow Summary

### Input → Processing → Output

```
┌─────────────────────────────────────────────────────┐
│  INPUT: Camera Image (640x480, grayscale/color)    │
└───────────────────┬─────────────────────────────────┘
                    │
                    ▼
┌─────────────────────────────────────────────────────┐
│  FEATURE EXTRACTION (orb_extractor)                 │
│  • Multi-scale pyramid (8 levels)                   │
│  • FAST corner detection                            │
│  • ORB descriptor computation                       │
│  Output: ~2000 keypoints + descriptors              │
└───────────────────┬─────────────────────────────────┘
                    │
                    ▼
┌─────────────────────────────────────────────────────┐
│  TRACKING (tracking_module)                         │
│  • Match features to local map                      │
│  • PnP pose estimation                              │
│  • Pose optimization                                │
│  Output: Camera pose (Mat44_t)                      │
└───────────────────┬─────────────────────────────────┘
                    │
                    ▼ (if keyframe needed)
┌─────────────────────────────────────────────────────┐
│  MAPPING (mapping_module) [Background Thread]       │
│  • Triangulate new landmarks                        │
│  • Local bundle adjustment                          │
│  • Remove redundant elements                        │
│  Output: Updated map (keyframes + landmarks)        │
└───────────────────┬─────────────────────────────────┘
                    │
                    ▼
┌─────────────────────────────────────────────────────┐
│  OUTPUT                                             │
│  • Camera pose (every frame, ~30 Hz)                │
│  • 3D map (keyframes + landmarks, growing)          │
│  • Tracking state ("Tracking", "Lost", etc.)        │
└─────────────────────────────────────────────────────┘
```

---

## 8. For Your PiCar Project

### How to Use the Visual Mapper

**Option 1: Use Existing Executables**

```bash
# Real-time SLAM with PiCar camera
run_camera_slam \
  -v ~/vocab/orb_vocab.fbow \
  -c ~/Documents/openvslam/pi_camera_640x480.yaml \
  -n 0
```

**Option 2: Integrate into Python**

Create a Python wrapper (via pybind11 or similar):

```python
import stella_vslam
import cv2

# Initialize
slam = stella_vslam.System("config.yaml", "vocab.fbow")
slam.startup()

# Process frames
cap = cv2.VideoCapture(0)
timestamp = 0.0

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Feed to visual mapper
    pose = slam.feed_monocular_frame(frame, timestamp)

    if pose is not None:
        x, y, z = pose[0,3], pose[1,3], pose[2,3]
        print(f"Position: ({x:.2f}, {y:.2f}, {z:.2f})")

    timestamp += 0.033

# Save and shutdown
slam.save_map_database("picar_map.msg")
slam.shutdown()
```

**Option 3: Use Publishers (C++ API)**

```cpp
// Get publishers for external access
auto map_pub = SLAM.get_map_publisher();
auto frame_pub = SLAM.get_frame_publisher();

// In your control loop
while (running) {
    SLAM.feed_monocular_frame(frame, timestamp);

    // Get current pose
    auto pose = map_pub->get_current_cam_pose();

    // Get landmarks for obstacle detection
    auto landmarks = map_pub->get_landmarks();

    // Use for navigation...
}
```

---

## 9. Key Takeaways

### The Visual Mapper is Simple Because:

1. **Single entry point:** `system::feed_monocular_frame()`
2. **Automatic processing:** Feature extraction, tracking, mapping all automatic
3. **Background operation:** Mapping runs in separate thread (non-blocking)
4. **Clean output:** Just pose + map, no complex intermediate data

### What You Get:

| Output | Type | Update Rate | Use For |
|--------|------|-------------|---------|
| Camera pose | Mat44_t (4x4) | ~30 Hz | Localization, control |
| 3D landmarks | Vec3_t positions | Growing | Obstacle detection, planning |
| Keyframes | Poses + features | ~1-5 Hz | Path history, relocalization |
| Tracking state | String | ~30 Hz | Quality monitoring |

### For Navigation:

1. **Pose** → Controller (PID steering, speed)
2. **Landmarks** → Obstacle map (occupancy grid)
3. **Keyframes** → Reference trajectory (path planning)

---

## 10. File Reference Quick List

| Component | Header File |
|-----------|-------------|
| **System** | [src/stella_vslam/system.h](src/stella_vslam/system.h) |
| **Tracking** | [src/stella_vslam/tracking_module.h](src/stella_vslam/tracking_module.h) |
| **Mapping** | [src/stella_vslam/mapping_module.h](src/stella_vslam/mapping_module.h) |
| **ORB Extractor** | [src/stella_vslam/feature/orb_extractor.h](src/stella_vslam/feature/orb_extractor.h) |
| **Frame** | [src/stella_vslam/data/frame.h](src/stella_vslam/data/frame.h) |
| **Keyframe** | [src/stella_vslam/data/keyframe.h](src/stella_vslam/data/keyframe.h) |
| **Landmark** | [src/stella_vslam/data/landmark.h](src/stella_vslam/data/landmark.h) |
| **Map Database** | [src/stella_vslam/data/map_database.h](src/stella_vslam/data/map_database.h) |
| **Triangulator** | [src/stella_vslam/module/two_view_triangulator.h](src/stella_vslam/module/two_view_triangulator.h) |
| **Map Publisher** | [src/stella_vslam/publish/map_publisher.h](src/stella_vslam/publish/map_publisher.h) |

---

## Summary

The **stella_vslam visual mapper** is a complete monocular/stereo SLAM system with:

- **Simple interface:** One function to feed frames
- **Automatic processing:** Features, tracking, mapping all handled
- **Real-time operation:** 30 Hz tracking, background mapping
- **Clean outputs:** Pose, map, state

Perfect for robotics applications like your PiCar navigation project!

---

**End of Visual Mapper Library Guide**
