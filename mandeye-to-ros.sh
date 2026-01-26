#!/bin/bash
set -e

DATASET_HOST_PATH="$1"
BAG_OUTPUT_HOST="$2"
ROS_VERSION="$3"   # ros1 | ros2

if [[ -z "$DATASET_HOST_PATH" || -z "$BAG_OUTPUT_HOST" || -z "$ROS_VERSION" ]]; then
  echo "Usage: $0 <dataset_path> <output_path> <ros1|ros2>"
  exit 1
fi

IMAGE_NAME_ROS1=mandeye-ws_noetic
IMAGE_NAME_ROS2=mandeye-ws_humble

case "$ROS_VERSION" in
  ros1)
    IMAGE_NAME="$IMAGE_NAME_ROS1"
    ROS_SETUP="source /opt/ros/noetic/setup.bash && source /mandeye_ws/devel/setup.bash"
    RUN_CMD="rosrun mandeye_to_rosbag1 mandeye_to_rosbag"
    BAG_NAME="converted.bag"
    ;;
  ros2)
    IMAGE_NAME="$IMAGE_NAME_ROS2"
    ROS_SETUP="source /opt/ros/humble/setup.bash && source /mandeye_ws/install/setup.bash"
    RUN_CMD="ros2 run mandeye_to_rosbag2 mandeye_to_rosbag2_node"
    BAG_NAME="converted"
    ;;
esac

DATASET_HOST_PATH=$(realpath "$DATASET_HOST_PATH")
BAG_OUTPUT_HOST=$(realpath "$BAG_OUTPUT_HOST")

DATASET_CONTAINER_PATH=/mandeye_ws/dataset
BAG_OUTPUT_CONTAINER=/mandeye_ws/output

mkdir -p "$BAG_OUTPUT_HOST"

xhost +local:docker >/dev/null

docker run -it --rm \
  --network host \
  -v "$DATASET_HOST_PATH":"$DATASET_CONTAINER_PATH":ro \
  -v "$BAG_OUTPUT_HOST":"$BAG_OUTPUT_CONTAINER" \
  "$IMAGE_NAME" \
  /bin/bash -c "
    set -e &&
    $ROS_SETUP &&
    $RUN_CMD \
      $DATASET_CONTAINER_PATH \
      $BAG_OUTPUT_CONTAINER/$BAG_NAME
  "
