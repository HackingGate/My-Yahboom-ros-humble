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
        self.publisher_ = self.create_publisher(CompressedImage, '/compressed_video/mp4', 10)
        self.bridge = CvBridge()
        self.width = 320
        self.height = 240
        self.process = self.start_ffmpeg_process()

    def start_ffmpeg_process(self):
        return (
            ffmpeg
            .input('pipe:', format='rawvideo', pix_fmt='rgb24', s=f'{self.width}x{self.height}')
            .output('pipe:', format='matroska', vcodec='libx265', pix_fmt='yuv420p', r=30)
            .run_async(pipe_stdin=True, pipe_stdout=True, pipe_stderr=True, overwrite_output=True)
        )

    def listener_callback(self, msg):
        try:
            # Convert compressed image message to OpenCV image
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, 'bgr8')

            # Resize the image to 320x240 if necessary
            if cv_image.shape[1] != self.width or cv_image.shape[0] != self.height:
                cv_image = cv2.resize(cv_image, (self.width, self.height))

            # Convert BGR to RGB
            cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)

            # Write frame to ffmpeg process
            self.process.stdin.write(
                cv_image
                .astype(np.uint8)
                .tobytes()
            )

            # Read encoded frame from ffmpeg process
            out_frame = self.process.stdout.read(self.width * self.height * 3 // 2)  # For yuv420p, size is width*height*3/2
            if len(out_frame) == 0:
                self.get_logger().error("No data read from ffmpeg process.")
                stderr_output = self.process.stderr.read()
                self.get_logger().error(f"ffmpeg stderr: {stderr_output.decode()}")
                self.restart_ffmpeg_process()
                return

            # Create a new CompressedImage message
            compressed_image_msg = CompressedImage()
            compressed_image_msg.header = msg.header
            compressed_image_msg.format = 'mkv'  # Changed format to mkv
            compressed_image_msg.data = out_frame

            # Publish the compressed video frame
            self.publisher_.publish(compressed_image_msg)
        except Exception as e:
            self.get_logger().error(f"Failed to process video frame: {str(e)}")
            self.restart_ffmpeg_process()

    def restart_ffmpeg_process(self):
        self.get_logger().info("Restarting ffmpeg process.")
        self.process.stdin.close()
        self.process.wait()
        self.process = self.start_ffmpeg_process()

    def destroy_node(self):
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
