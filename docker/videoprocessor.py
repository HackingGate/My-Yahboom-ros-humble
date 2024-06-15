import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from cv_bridge import CvBridge
import subprocess
import os
import threading
import cv2

class VideoProcessor(Node):
    def __init__(self):
        super().__init__('video_processor')
        self.subscription = self.create_subscription(
            CompressedImage,
            '/usb_cam/image_raw/compressed',
            self.listener_callback,
            10)
        self.publisher_ = self.create_publisher(String, '/compressed_video/rtp', 10)
        self.bridge = CvBridge()
        self.image_save_dir = '/tmp'  # Temporary directory for saving images
        self.output_video_path = os.path.join(self.image_save_dir, 'output_video.mp4')  # Path to save the output video
        self.lock = threading.Lock()

    def listener_callback(self, msg):
        threading.Thread(target=self.process_and_save_video, args=(msg,)).start()

    def process_and_save_video(self, msg):
        try:
            # Convert compressed image message to OpenCV image
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg)
            
            # Define the path for the temporary image file
            temp_image_path = os.path.join(self.image_save_dir, 'temp_image.jpg')
            
            # Save the image to the temporary file
            cv2.imwrite(temp_image_path, cv_image)

            # Check if the file is successfully written
            if not os.path.exists(temp_image_path) or os.path.getsize(temp_image_path) == 0:
                self.get_logger().error("Temporary image file is missing or empty.")
                return

            # Define the ffmpeg command for encoding to VP9 and saving as MP4
            ffmpeg_command = [
                'ffmpeg', '-y', '-framerate', '20', '-i', temp_image_path, '-r', '20', '-c:v', 'libvpx-vp9',
                '-strict', 'experimental', '-f', 'mp4', self.output_video_path
            ]

            # Execute the ffmpeg command
            process = subprocess.Popen(ffmpeg_command, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            stdout, stderr = process.communicate()

            if process.returncode != 0:
                self.get_logger().error(f"FFmpeg command failed: {stderr.decode('utf-8')}")
            else:
                self.get_logger().info(f"Saved output video to {self.output_video_path}")

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
