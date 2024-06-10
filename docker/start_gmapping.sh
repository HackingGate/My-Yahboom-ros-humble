#!/bin/bash

# Source ROS 2 and your workspace
source /opt/ros/humble/setup.bash
source /root/yahboomcar_ws/install/setup.bash

echo "Environment setup successful"

# Start the supervisor service
systemctl start supervisor
echo "Supervisor service started"

# Run your Python script
python3 ~/publish_ip.py &
echo "IP publish script started"

# Give time for the environment to settle
sleep 1

source ~/yahboomcar_ros2_ws/yahboomcar_ws/install/setup.bash

# Function to run launch files and log output
run_launch() {
    package=$1
    launch_file=$2
    log_file=$3
    echo "Starting $launch_file from package $package..."
    nohup ros2 launch $package $launch_file > $log_file 2>&1 &
    sleep 1
    if pgrep -f "ros2 launch $package $launch_file" > /dev/null; then
        echo "$launch_file from $package started successfully."
    else
        echo "Failed to start $launch_file from $package. Check $log_file for details."
    fi
}

# Run the ROS 2 launch commands in the background
run_launch "yahboomcar_bringup" "yahboomcar_bringup_launch.py" "/root/yahboomcar_bringup.log"
run_launch "yahboomcar_nav" "map_gmapping_app_launch.xml" "/root/yahboomcar_nav.log"
run_launch "usb_cam" "camera.launch.py" "/root/usb_cam.log"
nohup ros2 run pkg_imageprocessor_py imageprocessor > /root/imageprocessor.log 2>&1 &

# Allow some time for the nodes to start properly
sleep 1

# Check if the ROS nodes are running
echo "ROS nodes running:"
ros2 node list

# Check the available topics
echo "ROS topics available:"
ros2 topic list

# Display the log files for debugging
echo "Contents of yahboomcar_bringup.log:"
cat /root/yahboomcar_bringup.log

echo "Contents of yahboomcar_nav.log:"
cat /root/yahboomcar_nav.log

echo "Contents of usb_cam.log:"
cat /root/usb_cam.log

echo "Contents of imageprocessor.log:"
cat /root/imageprocessor.log

# Keep the container running
tail -f /dev/null

wait
exit 0
