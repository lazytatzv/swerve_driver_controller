#!/usr/bin/env bash
set -e

echo "=== Installing ROS 2 dependencies for Swerve Drive ==="
apt update
apt install -y \
  ros-lyrical-ros2-control \
  ros-lyrical-ros2-control-cmake \
  ros-lyrical-ros2-controllers \
  ros-lyrical-joint-state-broadcaster \
  ros-lyrical-generate-parameter-library \
  ros-lyrical-control-toolbox \
  ros-lyrical-realtime-tools \
  ros-lyrical-backward-ros \
  ros-lyrical-foxglove-bridge \
  ros-lyrical-xacro

echo "=== All dependencies installed successfully! ==="
