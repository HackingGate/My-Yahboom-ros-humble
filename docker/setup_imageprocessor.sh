#!/bin/bash

# Navigate to the source directory
cd ~/yahboomcar_ros2_ws/yahboomcar_ws/src

# Create the ROS2 package
ros2 pkg create pkg_imageprocessor_py --build-type ament_python --dependencies rclpy --node-name imageprocessor

# Download and replace the imageprocessor.py file
wget -O ~/yahboomcar_ros2_ws/yahboomcar_ws/src/pkg_imageprocessor_py/pkg_imageprocessor_py/imageprocessor.py https://github.com/HackingGate/my-yahboom-ros-humble/raw/main/docker/imageprocessor.py

# Navigate to the workspace directory
cd ~/yahboomcar_ros2_ws/yahboomcar_ws

# Build the package
colcon build --packages-select pkg_imageprocessor_py

# Source the setup script
source install/setup.bash
