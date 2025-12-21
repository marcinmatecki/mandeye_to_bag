#!/bin/bash

# Script to convert Mandeye data to ROS1 bag using Docker
# Usage: ./mandeye-to-bag-ros1.sh <input_mandeye_directory> <output_bag_path_or_prefix> [--lines <number>]

set -e

if [ "$#" -lt 2 ]; then
    echo "Usage: $0 <input_mandeye_directory> <output_bag_path_or_prefix> [--lines <number>]"
    echo ""
    echo "Arguments:"
    echo "  input_mandeye_directory   - Directory containing Mandeye data (.laz and .csv files)"
    echo "  output_bag_path_or_prefix - Output path for ROS1 bag ('.bag' will be appended if missing)"
    echo ""
    echo "Options:"
    echo "  --lines <number>          - Number of lines (default 8, for mid360)"
    echo ""
    echo "Example:"
    echo "  $0 /path/to/mandeye/data /path/to/output/mandeye.bag"
    echo "  $0 ./mandeye_data ./out/mandeye --lines 8"
    exit 1
fi

INPUT_DIR="$1"
OUTPUT_BAG="$2"
shift 2

# Get absolute paths
INPUT_DIR=$(realpath "$INPUT_DIR")
OUTPUT_BAG=$(realpath -m "$OUTPUT_BAG")

# If output doesn't end with .bag, append it
if [[ "$OUTPUT_BAG" != *.bag ]]; then
    OUTPUT_BAG="${OUTPUT_BAG}.bag"
fi

OUTPUT_DIR=$(dirname "$OUTPUT_BAG")
OUTPUT_NAME=$(basename "$OUTPUT_BAG")

# Collect additional arguments (forward to converter)
EXTRA_ARGS=""
while [[ $# -gt 0 ]]; do
    EXTRA_ARGS="$EXTRA_ARGS $1"
    shift
done

# Check if input directory exists
if [ ! -d "$INPUT_DIR" ]; then
    echo "Error: Input directory '$INPUT_DIR' does not exist"
    exit 1
fi

# Create output directory if it doesn't exist
mkdir -p "$OUTPUT_DIR"

echo "Converting Mandeye data to ROS1 bag..."
echo "  Input directory: $INPUT_DIR"
echo "  Output bag: $OUTPUT_DIR/$OUTPUT_NAME"
echo "  Extra arguments: $EXTRA_ARGS"
echo ""

# Run Docker container
docker run --rm \
    -v "$INPUT_DIR:/input:ro" \
    -v "$OUTPUT_DIR:/output" \
    -u $(id -u):$(id -g) \
    mandeye2bag_ros1 \
    bash -c "source /catkin_ws/devel/setup.bash && \
             rosrun mandeye_to_rosbag1 mandeye_to_rosbag \
             /input \
             /output/$OUTPUT_NAME \
             $EXTRA_ARGS"

echo ""
echo "Conversion complete! ROS1 bag saved to: $OUTPUT_DIR/$OUTPUT_NAME"


