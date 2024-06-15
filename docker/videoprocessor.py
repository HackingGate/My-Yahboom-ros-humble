import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from cv_bridge import CvBridge
import ffmpeg
import numpy as np
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
        self.process = None
        self.frame_size = None
        self.rtp_url = self.generate_rtp_url()

    def generate_rtp_url(self):
        port = 5004  # Default RTP port
        return f"rtp://localhost:{port}"

    def start_ffmpeg_process(self, width, height):
        return (
            ffmpeg
            .input('pipe:', format='rawvideo', pix_fmt='yuv422p', s=f'{width}x{height}')
            .output(self.rtp_url, format='rtp', vcodec='libx264', pix_fmt='yuv422p', r=30)
            .run_async(pipe_stdin=True, pipe_stdout=True, pipe_stderr=True, overwrite_output=True)
        )

    def listener_callback(self, msg):
        try:
            # Convert compressed image message to OpenCV image
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_UNCHANGED)

            # Get frame size
            height, width, _ = cv_image.shape
            if self.frame_size is None:
                self.frame_size = (width, height)
                self.process = self.start_ffmpeg_process(width, height)
                self.publish_rtp_url()
            elif self.frame_size != (width, height):
                self.frame_size = (width, height)
                self.restart_ffmpeg_process(width, height)

            # Write frame to ffmpeg process
            self.process.stdin.write(
                cv_image
                .astype(np.uint8)
                .tobytes()
            )
        except Exception as e:
            self.get_logger().error(f"Failed to process video frame: {str(e)}")
            self.restart_ffmpeg_process(width, height)

    def publish_rtp_url(self):
        rtp_msg = String()
        rtp_msg.data = self.rtp_url
        self.publisher_.publish(rtp_msg)

    def restart_ffmpeg_process(self, width, height):
        self.get_logger().info("Restarting ffmpeg process.")
        if self.process:
            self.process.stdin.close()
            self.process.wait()
        self.process = self.start_ffmpeg_process(width, height)
        self.publish_rtp_url()

    def destroy_node(self):
        if self.process:
            self.process.stdin.close()
            self.process.wait()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    video_processor = VideoProcessor()
    rclpy.spin(video_processor)
    video_processor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
