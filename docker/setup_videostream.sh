#!/bin/bash

# Navigate to the source directory
cd ~/yahboomcar_ros2_ws/yahboomcar_ws/src

# Create the ROS2 package
ros2 pkg create pkg_videostream_py --build-type ament_python --dependencies rclpy --node-name videostream

# Download and replace the videostream.py file
wget -O ~/yahboomcar_ros2_ws/yahboomcar_ws/src/pkg_videostream_py/pkg_videostream_py/videostream.py https://github.com/HackingGate/my-yahboom-ros-humble/raw/main/docker/videostream.py

# Navigate to the workspace directory
cd ~/yahboomcar_ros2_ws/yahboomcar_ws

# Build the package
colcon build --packages-select pkg_videostream_py

# Source the setup script
source install/setup.bash
