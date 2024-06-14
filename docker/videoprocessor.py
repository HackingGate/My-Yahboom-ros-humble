import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import subprocess
import os
import threading

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
        self.lock = threading.Lock()

    def listener_callback(self, msg):
        threading.Thread(target=self.process_and_publish_video, args=(msg,)).start()

    def process_and_publish_video(self, msg):
        try:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg)
            temp_image_path = '/tmp/temp_image.png'
            cv2.imwrite(temp_image_path, cv_image)

            # Ensure the image is correctly written
            if not os.path.exists(temp_image_path):
                self.get_logger().error("Failed to write image to temporary file.")
                return

            ffmpeg_command = [
                'ffmpeg', '-y', '-re', '-i', temp_image_path, '-c:v', 'libvpx-vp9',
                '-strict', 'experimental', '-f', 'rtp', 'rtp://localhost:1234'
            ]

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
