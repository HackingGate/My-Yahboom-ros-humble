FROM yahboomtechnology/ros-humble:4.1.2

# Copy your script into the container
COPY gmapping.sh /root/gmapping.sh

# Make sure your script is executable
RUN chmod +x /root/gmapping.sh

# Change directory and create the ROS2 package
RUN cd ~/yahboomcar_ros2_ws/yahboomcar_ws/src && \
    ros2 pkg create pkg_imageprocessor_py --build-type ament_python --dependencies rclpy --node-name imageprocessor

# Copy imageprocessor.py to the correct location
COPY imageprocessor.py ~/yahboomcar_ros2_ws/yahboomcar_ws/src/pkg_imageprocessor_py/pkg_imageprocessor_py/imageprocessor.py

# Build the package
RUN cd ~/yahboomcar_ros2_ws/yahboomcar_ws && \
    colcon build --packages-select pkg_imageprocessor_py

# Source the setup script
RUN echo "source ~/yahboomcar_ros2_ws/yahboomcar_ws/install/setup.bash" >> ~/.bashrc
