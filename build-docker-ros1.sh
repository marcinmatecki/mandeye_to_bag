#!/bin/bash

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

docker build \
  -t mandeye2bag_ros1 \
  -f "$SCRIPT_DIR/Dockerfile.ros1" \
  "$SCRIPT_DIR"

