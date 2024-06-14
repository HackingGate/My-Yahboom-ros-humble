FROM yahboomtechnology/ros-humble:4.1.2

# Update and install ffmpeg
RUN apt-get update && apt-get install -y ffmpeg

# Install ffmpeg-python using pip
RUN pip install ffmpeg-python

# Copy your scripts into the container
COPY start_gmapping.sh /root/start_gmapping.sh
COPY setup_imageprocessor.sh /root/setup_imageprocessor.sh
COPY setup_videostream.sh /root/setup_videostream.sh

# Make sure your scripts are executable
RUN chmod +x /root/start_gmapping.sh
RUN chmod +x /root/setup_imageprocessor.sh
RUN chmod +x /root/setup_videostream.sh

# Copy the usb_cam configuration file into the container
COPY usb_cam_config_params_1.yaml /opt/ros/humble/share/usb_cam/config/params_1.yaml
