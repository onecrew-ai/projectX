#!/bin/bash
set -e

echo ">>> Sourcing ROS2..."
source /opt/ros/${ROS_DISTRO:-jazzy}/setup.bash

echo ">>> Updating submodules..."
git submodule update --init --recursive

echo ">>> Installing rosdep dependencies..."
sudo rosdep init 2>/dev/null || true
rosdep update
rosdep install --from-paths src --ignore-src -r -y

echo ">>> Building workspace..."
colcon build --symlink-install

echo ">>> Done!"
echo "Run: source install/setup.bash"
echo "✔ ROS2 workspace successfully built and ready."
