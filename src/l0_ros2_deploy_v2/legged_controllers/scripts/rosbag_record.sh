#!/bin/bash

# Create bag directory if it doesn't exist
BAG_DIR="$1"
shift

# Create directory
mkdir -p "$BAG_DIR"

# Run rosbag record with remaining arguments
exec rosbag record "$@"
