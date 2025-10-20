#!/bin/bash
# Helper script to save SLAM maps with organized naming

# Usage: ./save_slam_map.sh <map_name> <frame_directory>
# Example: ./save_slam_map.sh living_room /tmp/vilib_frames

if [ $# -lt 1 ]; then
    echo "Usage: $0 <map_name> [frame_directory]"
    echo ""
    echo "Examples:"
    echo "  $0 living_room"
    echo "  $0 kitchen /tmp/picar_slam_20251019_153000"
    echo ""
    exit 1
fi

MAP_NAME="$1"
FRAME_DIR="${2:-/tmp/vilib_frames}"
MAPS_DIR="$HOME/slam_maps"
TIMESTAMP=$(date +%Y%m%d_%H%M%S)

# Create maps directory if it doesn't exist
mkdir -p "$MAPS_DIR"

# Generate output filename
OUTPUT_FILE="$MAPS_DIR/${MAP_NAME}_${TIMESTAMP}.msg"

echo "============================================================"
echo "SLAM Map Builder"
echo "============================================================"
echo "Map name:     $MAP_NAME"
echo "Frame dir:    $FRAME_DIR"
echo "Output file:  $OUTPUT_FILE"
echo "============================================================"
echo ""

# Check if frame directory exists
if [ ! -d "$FRAME_DIR" ]; then
    echo "Error: Frame directory not found: $FRAME_DIR"
    exit 1
fi

# Count frames
FRAME_COUNT=$(ls -1 "$FRAME_DIR"/frame_*.png 2>/dev/null | wc -l)
if [ "$FRAME_COUNT" -eq 0 ]; then
    echo "Error: No frames found in $FRAME_DIR"
    exit 1
fi

echo "Found $FRAME_COUNT frames"
echo ""
echo "Starting SLAM..."
echo ""

# Run SLAM
run_image_slam \
  -v ~/vocab/orb_vocab.fbow \
  -d "$FRAME_DIR" \
  -c ~/Documents/openvslam/pi_camera_640x480.yaml \
  -o "$OUTPUT_FILE"

if [ $? -eq 0 ]; then
    echo ""
    echo "============================================================"
    echo "SUCCESS! Map saved to:"
    echo "  $OUTPUT_FILE"
    echo ""
    echo "To use this map for localization:"
    echo "  run_image_slam -v ~/vocab/orb_vocab.fbow \\"
    echo "    -d <new_frame_dir> \\"
    echo "    -c ~/Documents/openvslam/pi_camera_640x480.yaml \\"
    echo "    -i $OUTPUT_FILE \\"
    echo "    --disable-mapping"
    echo "============================================================"
else
    echo ""
    echo "Error: SLAM failed"
    exit 1
fi
