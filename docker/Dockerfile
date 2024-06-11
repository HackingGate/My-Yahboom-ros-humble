FROM yahboomtechnology/ros-humble:4.1.2

# Copy your script into the container
COPY start_gmapping.sh /root/start_gmapping.sh
COPY setup_imageprocessor.sh /root/setup_imageprocessor.sh
COPY setup_videostream.sh /root/setup_videostream.sh

# Make sure your script is executable
RUN chmod +x /root/start_gmapping.sh
RUN chmod +x /root/setup_imageprocessor.sh
RUN chmod +x /root/setup_videostream.sh
