#!/bin/bash
echo "Setting up ROS 2 package..."
cd src/lunar_simulation

# Source ROS 2
source /opt/ros/jazzy/setup.bash

# Install dependencies
sudo apt install python3-colcon-common-extensions -y
sudo apt install ros-jazzy-gazebo-ros -y

echo "✅ ROS 2 package structure created"
echo "To build: colcon build"
echo "To run: ros2 launch lunar_simulation simulation.launch.py"
