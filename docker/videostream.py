import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import ffmpeg
import numpy as np
import cv2

class VideoStream(Node):
    def __init__(self):
        super().__init__('video_stream')
        self.subscription = self.create_subscription(
            CompressedImage,
            '/usb_cam/image_raw/compressed',
            self.listener_callback,
            10)
        self.publisher_ = self.create_publisher(CompressedImage, '/compressed_video/h264', 10)
        self.bridge = CvBridge()
        self.process = None
        self.frame_size = None

    def start_ffmpeg_process(self, width, height):
        return (
            ffmpeg
            .input('pipe:', format='rawvideo', pix_fmt='yuv422p', s=f'{width}x{height}')
            .output('pipe:', format='h264', vcodec='libx264', pix_fmt='yuv422p', r=30)
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
            elif self.frame_size != (width, height):
                self.frame_size = (width, height)
                self.restart_ffmpeg_process(width, height)

            # Write frame to ffmpeg process
            self.process.stdin.write(
                cv_image
                .astype(np.uint8)
                .tobytes()
            )

            # Read encoded frame from ffmpeg process
            out_frame = self.process.stdout.read(width * height * 2)  # For yuv422p, size is width*height*2
            if len(out_frame) == 0:
                self.get_logger().error("No data read from ffmpeg process.")
                stderr_output = self.process.stderr.read()
                self.get_logger().error(f"ffmpeg stderr: {stderr_output.decode()}")
                self.restart_ffmpeg_process(width, height)
                return

            # Create a new CompressedImage message
            compressed_image_msg = CompressedImage()
            compressed_image_msg.header = msg.header
            compressed_image_msg.format = 'h264'
            compressed_image_msg.data = out_frame

            # Publish the compressed video frame
            self.publisher_.publish(compressed_image_msg)
        except Exception as e:
            self.get_logger().error(f"Failed to process video frame: {str(e)}")
            self.restart_ffmpeg_process(width, height)

    def restart_ffmpeg_process(self, width, height):
        self.get_logger().info("Restarting ffmpeg process.")
        if self.process:
            self.process.stdin.close()
            self.process.wait()
        self.process = self.start_ffmpeg_process(width, height)

    def destroy_node(self):
        if self.process:
            self.process.stdin.close()
            self.process.wait()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    video_stream = VideoStream()
    rclpy.spin(video_stream)
    video_stream.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
