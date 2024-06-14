#!/bin/bash

# Navigate to the source directory
cd ~/yahboomcar_ros2_ws/yahboomcar_ws/src

# Create the ROS2 package
ros2 pkg create pkg_videoprocessor_py --build-type ament_python --dependencies rclpy --node-name videoprocessor

# Download and replace the videoprocessor.py file
wget -O ~/yahboomcar_ros2_ws/yahboomcar_ws/src/pkg_videoprocessor_py/pkg_videoprocessor_py/videoprocessor.py https://github.com/HackingGate/my-yahboom-ros-humble/raw/main/docker/videoprocessor.py

# Navigate to the workspace directory
cd ~/yahboomcar_ros2_ws/yahboomcar_ws

# Build the package
colcon build --packages-select pkg_videoprocessor_py

# Source the setup script
source install/setup.bash
