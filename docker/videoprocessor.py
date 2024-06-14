import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import threading
import subprocess
import time
import os

class VideoProcessor(Node):
    def __init__(self):
        super().__init__('video_processor')
        self.subscription = self.create_subscription(
            CompressedImage,
            '/usb_cam/image_raw/compressed',
            self.listener_callback,
            10)
        self.rtp_publisher = self.create_publisher(String, '/compressed_video/rtp', 10)
        self.bridge = CvBridge()
        self.compression_quality = 50  # Set the desired compression quality (0-100)
        self.lock = threading.Lock()  # To ensure thread safety
        self.rtp_address = "rtp://localhost:1234"
        self.get_logger().info("VideoProcessor node started")
        self.last_frame_time = 0
        self.frame_rate = 10  # Limit to 10 frames per second

    def listener_callback(self, msg):
        current_time = time.time()
        if current_time - self.last_frame_time >= 1.0 / self.frame_rate:
            threading.Thread(target=self.process_and_publish_video, args=(msg,)).start()
            self.last_frame_time = current_time

    def process_and_publish_video(self, msg):
        try:
            # Convert compressed image message to OpenCV image
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg)

            # Enhance the image for low-light conditions
            enhanced_image = self.enhance_low_light_image(cv_image)

            # Encode the enhanced image to VP9 using FFmpeg
            self.encode_and_stream_rtp(enhanced_image)

            # Publish the RTP address when the stream starts
            rtp_msg = String()
            rtp_msg.data = self.rtp_address
            self.rtp_publisher.publish(rtp_msg)
            self.get_logger().info(f"Published RTP address: {self.rtp_address}")

        except Exception as e:
            self.get_logger().error(f"Failed to process video: {str(e)}")

    def enhance_low_light_image(self, img):
        # Convert to YUV color space
        yuv_img = cv2.cvtColor(img, cv2.COLOR_BGR2YUV)
        
        # Histogram equalization on the Y channel
        yuv_img[:, :, 0] = cv2.equalizeHist(yuv_img[:, :, 0])
        
        # Convert back to BGR color space
        enhanced_img = cv2.cvtColor(yuv_img, cv2.COLOR_YUV2BGR)
        
        # Additional enhancements (optional)
        enhanced_img = cv2.fastNlMeansDenoisingColored(enhanced_img, None, 10, 10, 7, 21)
        enhanced_img = cv2.convertScaleAbs(enhanced_img, alpha=1.2, beta=15)
        
        return enhanced_img

    def encode_and_stream_rtp(self, img):
        # Save the image to a temporary file
        temp_img_path = '/tmp/temp_image.png'
        cv2.imwrite(temp_img_path, img)

        # Ensure the image is valid
        if not os.path.exists(temp_img_path) or os.path.getsize(temp_img_path) == 0:
            self.get_logger().error("Failed to save a valid PNG image")
            return

        # Use FFmpeg to encode the image to VP9 and stream over RTP
        cmd = [
            'ffmpeg',
            '-y',
            '-re',
            '-i', temp_img_path,
            '-c:v', 'libvpx-vp9',
            '-strict', 'experimental',
            '-f', 'rtp',
            self.rtp_address
        ]
        subprocess.run(cmd, check=True)

def main(args=None):
    rclpy.init(args=args)
    video_processor = VideoProcessor()
    rclpy.spin(video_processor)
    video_processor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
