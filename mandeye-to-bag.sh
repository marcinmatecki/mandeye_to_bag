#!/bin/bash

# Script to convert Mandeye data to ROS2 bag using Docker
# Usage: ./mandeye-to-bag.sh <input_mandeye_directory> <output_bag_name> [options]

set -e

if [ "$#" -lt 2 ]; then
    echo "Usage: $0 <input_mandeye_directory> <output_bag_name> [--lines <number>] [--type <pointcloud2/livox>]"
    echo ""
    echo "Arguments:"
    echo "  input_mandeye_directory  - Directory containing Mandeye data (.laz and .csv files)"
    echo "  output_bag_name         - Name/path for the output ROS2 bag"
    echo ""
    echo "Options:"
    echo "  --lines <number>        - Number of lines (default 8, for mid360)"
    echo "  --type <type>           - Message type: pointcloud2 or livox (default pointcloud2)"
    echo ""
    echo "Example:"
    echo "  $0 /path/to/mandeye/data /path/to/output/my_bag"
    echo "  $0 ./mandeye_data ./output_bag --lines 8 --type pointcloud2"
    exit 1
fi

INPUT_DIR="$1"
OUTPUT_BAG="$2"
shift 2

# Get absolute paths
INPUT_DIR=$(realpath "$INPUT_DIR")
OUTPUT_DIR=$(dirname "$(realpath -m "$OUTPUT_BAG")")
OUTPUT_NAME=$(basename "$OUTPUT_BAG")

# Collect additional arguments
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

echo "Converting Mandeye data to ROS2 bag..."
echo "  Input directory: $INPUT_DIR"
echo "  Output bag: $OUTPUT_DIR/$OUTPUT_NAME"
echo "  Extra arguments: $EXTRA_ARGS"
echo ""

# Run Docker container
docker run --rm \
    -v "$INPUT_DIR:/input:ro" \
    -v "$OUTPUT_DIR:/output" \
    -u $(id -u):$(id -g) \
    mandeye2bag \
    bash -c "source /test_ws/install/setup.bash && \
             ros2 run mandeye_to_rosbag2 mandeye_to_rosbag2_node \
             /input \
             /output/$OUTPUT_NAME \
             $EXTRA_ARGS"

echo ""
echo "Conversion complete! ROS2 bag saved to: $OUTPUT_DIR/$OUTPUT_NAME"

