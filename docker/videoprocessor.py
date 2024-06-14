import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
import subprocess
import os
import threading
import base64

class VideoProcessor(Node):
    def __init__(self):
        super().__init__('video_processor')
        self.subscription = self.create_subscription(
            CompressedImage,
            '/usb_cam/image_raw/compressed',
            self.listener_callback,
            10)
        self.publisher_ = self.create_publisher(String, '/compressed_video/rtp', 10)
        self.lock = threading.Lock()

    def listener_callback(self, msg):
        threading.Thread(target=self.process_and_publish_video, args=(msg,)).start()

    def process_and_publish_video(self, msg):
        try:
            # Decode base64-encoded image data
            image_data = base64.b64decode(msg.data)
            
            # Define the path for the temporary image file
            temp_image_path = '/tmp/temp_image.jpg'
            
            # Write the decoded image data to the temporary file
            with open(temp_image_path, 'wb') as f:
                f.write(image_data)

            # Check if the file is successfully written
            if not os.path.exists(temp_image_path) or os.path.getsize(temp_image_path) == 0:
                self.get_logger().error("Temporary image file is missing or empty.")
                return

            # Define the ffmpeg command for encoding to VP9 and streaming via RTP
            ffmpeg_command = [
                'ffmpeg', '-y', '-re', '-i', temp_image_path, '-c:v', 'libvpx-vp9',
                '-strict', 'experimental', '-f', 'rtp', 'rtp://localhost:1234'
            ]

            # Execute the ffmpeg command
            process = subprocess.Popen(ffmpeg_command, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            stdout, stderr = process.communicate()

            if process.returncode != 0:
                self.get_logger().error(f"FFmpeg command failed: {stderr.decode('utf-8')}")
            else:
                rtp_address = 'rtp://localhost:1234'
                self.get_logger().info(f"Published RTP address: {rtp_address}")
                rtp_msg = String()
                rtp_msg.data = rtp_address
                with self.lock:
                    self.publisher_.publish(rtp_msg)

            # Remove the temporary image file
            os.remove(temp_image_path)
        except Exception as e:
            self.get_logger().error(f"Failed to process video: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    video_processor = VideoProcessor()
    rclpy.spin(video_processor)
    video_processor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
