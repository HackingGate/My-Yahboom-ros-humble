import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from cv_bridge import CvBridge
import ffmpeg
import numpy as np

class VideoProcessor(Node):
    def __init__(self):
        super().__init__('video_processor')
        self.subscription = self.create_subscription(
            CompressedImage,
            '/usb_cam/image_raw/compressed',
            self.listener_callback,
            10)
        self.url_publisher = self.create_publisher(String, '/compressed_video/hls', 10)  # Changed topic to ll-hls
        self.bridge = CvBridge()
        self.process = None
        self.frame_size = None
        self.streaming_url = 'http://localhost:8000/hls/stream.m3u8'  # LL-HLS streaming URL
        self.publish_streaming_url()
        self.encoder = 'libx264'  # Using software encoder due to hardware limitations

    def start_ffmpeg_process(self, width, height):
        self.get_logger().info(f"Starting ffmpeg process to stream video to {self.streaming_url}")
        try:
            process = (
                ffmpeg
                .input('pipe:', format='rawvideo', pix_fmt='bgr24', s=f'{width}x{height}', r=20)
                .output(
                    self.streaming_url,
                    vcodec=self.encoder,  # Use the chosen encoder
                    pix_fmt='yuv420p',
                    r=20,
                    hls_time=1,  # Segment duration for LL-HLS
                    hls_list_size=3,  # Number of segments to keep in playlist
                    hls_flags='delete_segments+program_date_time',  # LL-HLS flags
                    f='hls',
                    hls_segment_type='fmp4',  # LL-HLS segment type
                    hls_playlist_type='event',  # Playlist type for LL-HLS
                    hls_allow_cache=0,  # Disable caching for low-latency
                )
                .overwrite_output()
                .run_async(pipe_stdin=True)
            )
            self.get_logger().info(f"Started FFmpeg process with encoder {self.encoder}")
            return process
        except ffmpeg.Error as e:
            self.get_logger().error(f"FFmpeg error with {self.encoder}: {e}")
            raise

    def listener_callback(self, msg):
        try:
            # Convert compressed image message to OpenCV image
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg)  # This loads the image in BGR format

            # Get frame size
            height, width, _ = cv_image.shape
            if self.frame_size is None:
                self.frame_size = (width, height)
                self.process = self.start_ffmpeg_process(width, height)
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

    def restart_ffmpeg_process(self, width, height):
        self.get_logger().info("Restarting ffmpeg process.")
        if self.process:
            self.process.stdin.close()
            self.process.wait()
        self.process = self.start_ffmpeg_process(width, height)

    def publish_streaming_url(self):
        self.get_logger().info(f"Publishing streaming URL: {self.streaming_url}")
        url_msg = String()
        url_msg.data = self.streaming_url
        self.url_publisher.publish(url_msg)

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
