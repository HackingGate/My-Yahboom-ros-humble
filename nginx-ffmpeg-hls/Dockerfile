# Use the official nginx image as a base
FROM nginx:latest

# Install FFmpeg
RUN apt-get update && \
    apt-get install -y ffmpeg

# Copy the custom nginx configuration file
COPY nginx.conf /etc/nginx/nginx.conf

# Create the directory to store HLS segments
RUN mkdir -p /var/www/hls

# Set the working directory
WORKDIR /var/www/hls

# Expose the HTTP port
EXPOSE 80

# Start Nginx
CMD ["nginx", "-g", "daemon off;"]
